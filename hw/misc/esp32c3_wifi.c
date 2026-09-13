#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qemu/guest-random.h"
#include "qapi/error.h"
#include "sysemu/sysemu.h"
#include "hw/hw.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "hw/misc/esp32c3_wifi.h"
#include "exec/address-spaces.h"
#include "esp32_wlan_packet.h"
#include "hw/qdev-properties.h"

#define DEBUG 0

#if DEBUG

static const char * get_reg_name(hwaddr addr){
    switch (addr)
    {
        case A_C3_WIFI_DMA_IN_STATUS:
           return "C3_WIFI_DMA_IN_STATUS";
           break;
        case A_C3_WIFI_DMA_INLINK:
           return "C3_WIFI_DMA_INLINK";
           break;
        case A_C3_WIFI_NEXT_RX_DSCR:
           return "C3_WIFI_NEXT_RX_DSCR";
           break;
        case A_C3_WIFI_LAST_RX_DSCR:
           return "C3_WIFI_LAST_RX_DSCR";
           break;
        case A_C3_WIFI_DMA_INT_STATUS:
           return "C3_WIFI_DMA_INT_STATUS";
           break;
        case A_C3_WIFI_DMA_INT_CLR:
           return "C3_WIFI_DMA_INT_CLR";
           break;
        case A_C3_WIFI_STATUS:
           return "C3_WIFI_STATUS";
           break;
        case A_C3_WIFI_DMA_OUTLINK:
           return "C3_WIFI_DMA_OUTLINK";
           break; 
        case A_C3_WIFI_DMA_OUT_STATUS:
           return "C3_WIFI_DMA_OUT_STATUS";
           break; 
        case A_C3_WIFI_TX_CONFIG:
           return "C3_WIFI_TX_CONFIG";
           break;         
        case A_C3_WIFI_TX_CLR:
           return "C3_WIFI_TX_CLR";
           break;         
        case A_C3_WIFI_TX_DURATION:
           return "C3_WIFI_TX_DURATION";
           break; 
        case A_C3_WIFI_OFFSET_REG:
           return "C3_WIFI_OFFSET_REG";
           break;   
    }
    return "**************";
}

#endif          

static uint64_t esp32C3_wifi_read(void *opaque, hwaddr addr, unsigned int size)
{
    
    Esp32WifiState *s = ESP32_WIFI(opaque);
    uint32_t r = s->mem[addr/4];
    
    switch(addr) {
        case A_C3_WIFI_DMA_INLINK:
            r=s->dma_inlink_address;
            break;
        case A_C3_WIFI_DMA_IN_STATUS:
            r= r & ~0x1;
            break;
        case A_C3_WIFI_DMA_INT_STATUS:
        case A_C3_WIFI_DMA_INT_CLR:
            r=s->raw_interrupt;
            break;
        case A_C3_WIFI_STATUS:
        case A_C3_WIFI_DMA_OUT_STATUS:
            r=1;
            break;           
    }

#if DEBUG
    printf("esp32C3_wifi_read  %25s(0x%04lx)= 0x%08x\n",get_reg_name(addr),(unsigned long) addr,r);
#endif    

    return r;
}
static void set_interrupt(Esp32WifiState *s,int e) {
    s->raw_interrupt |= e;
    qemu_set_irq(s->irq, 1);
}

void Esp32_WLAN_frame_delivered(Esp32WifiState *s){
    s->raw_interrupt |= 0x80;
    qemu_set_irq(s->irq, 1);
}

static void esp32C3_wifi_write(void *opaque, hwaddr addr, uint64_t value,
                                 unsigned int size) {
    Esp32WifiState *s = ESP32_WIFI(opaque);
#if DEBUG    
    printf("esp32C3_wifi_write %25s(0x%04lx)= 0x%08lx\n",get_reg_name(addr),(unsigned long) addr, (unsigned long) value);
#endif
    switch (addr) {
        case A_C3_WIFI_DMA_INLINK:
            s->dma_inlink_address = value;
            s->dma_inlink_ptr = value;
            s->mem[R_C3_WIFI_NEXT_RX_DSCR] = value; 
            s->mem[R_C3_WIFI_LAST_RX_DSCR] = value;
            break;
        case A_C3_WIFI_DMA_INT_CLR:
            s->raw_interrupt &= ~value;
            if(s->raw_interrupt == 0)
                qemu_set_irq(s->irq, 0);
            break;
        case A_C3_WIFI_DMA_OUTLINK:
            if (value & 0xc0000000) {                        
                // do a DMA transfer to the hardware from esp32 memory
                mac80211_frame frame;
                dma_list_item item;
                unsigned memaddr = (0x3fc00000 | (value & 0xfffff));
                address_space_read(&address_space_memory, memaddr,
                            MEMTXATTRS_UNSPECIFIED, &item, 12);
                address_space_read(&address_space_memory, item.address,
                            MEMTXATTRS_UNSPECIFIED, &frame, item.length);
                // frame from esp32 to ap
                frame.frame_length=item.length;
                frame.next_frame=0;
                Esp32_WLAN_handle_frame(s, &frame);
                set_interrupt(s,0x80);
            }
    }
    s->mem[addr/4]=value;
}

static int match_mac_address(uint8_t *a1,uint8_t *a2) {
    if(!memcmp(a1,a2,6)) return 1;
    if(!memcmp(a1,BROADCAST,6)) return 1;
    return 0;
}
// frame from ap to esp32
void Esp32_sendFrame(Esp32WifiState *s, mac80211_frame *frame,int length, int signal_strength) {    
    if(s->dma_inlink_address==0) return;
    uint8_t *header=malloc(sizeof(wifi_pkt_rx_ctrl_c3_t)+length);
    memset(header,0,sizeof(wifi_pkt_rx_ctrl_c3_t)+length);
    wifi_pkt_rx_ctrl_c3_t *pkt=(wifi_pkt_rx_ctrl_c3_t *)header;
    *pkt=(wifi_pkt_rx_ctrl_c3_t){
        .rssi=(signal_strength+(rand()%10)+96),
        .rate=11,
        .sig_len=length,
        .sig_len_copy=length,
        .legacy_length=length,
        .noise_floor=-97,
        .channel=esp32_wifi_channel,
        .timestamp=qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)/1000,
    };
    // These 4 bits are set if the mac addresses previously stored at 0x40 and 0x48
    // match the destination or bssid addresses in the frame
    if(match_mac_address(frame->destination_address,(uint8_t *)s->mem+A_WIFI_MAC_ADDR_FST_0)) 
        pkt->damatch0=1;
    if(match_mac_address(frame->destination_address,(uint8_t *)s->mem+A_WIFI_MAC_ADDR_FST_1)) 
        pkt->damatch1=1;
    if(match_mac_address(frame->bssid_address,(uint8_t *)s->mem+A_WIFI_MAC_ADDR_FST_0)) 
        pkt->bssidmatch0=1;
    if(match_mac_address(frame->bssid_address,(uint8_t *)s->mem+A_WIFI_MAC_ADDR_FST_1)) 
        pkt->bssidmatch1=1;
    //printf("...%x %x\n",header[3],frame->destination_address[0]);

    memcpy(header+sizeof(wifi_pkt_rx_ctrl_c3_t),frame,length);
    length+=sizeof(wifi_pkt_rx_ctrl_c3_t);
    // do a DMA transfer from the hardware to esp32 memory
    dma_list_item item;
    address_space_read(&address_space_memory, s->dma_inlink_ptr, MEMTXATTRS_UNSPECIFIED, &item, 12);
    address_space_write(&address_space_memory, item.address, MEMTXATTRS_UNSPECIFIED, header, length);
    item.length=length;
    item.eof=1;
    address_space_write(&address_space_memory, s->dma_inlink_ptr, MEMTXATTRS_UNSPECIFIED,&item,4);
    s->dma_inlink_ptr=item.next;
    if(s->dma_inlink_ptr == 0) s->dma_inlink_ptr = s->dma_inlink_address;
    set_interrupt(s, 0x1004000);
    free(header);
}

static const MemoryRegionOps esp32C3_wifi_ops = {
    .read =  esp32C3_wifi_read,
    .write = esp32C3_wifi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32c3_wifi_reset_enter(Object *obj, ResetType type)
{
    Esp32WifiState *s = ESP32_WIFI(obj);

    s->dma_inlink_address=0;
    s->dma_inlink_ptr=0;
    memset(s->mem,0,sizeof(s->mem));
    Esp32_WLAN_reset_ap(s);
}

static void esp32C3_wifi_realize(DeviceState *dev, Error **errp)
{
    Esp32WifiState *s = ESP32_WIFI(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    s->dma_inlink_address=0;
    s->dma_inlink_ptr=0;

    memory_region_init_io(&s->iomem, OBJECT(dev), &esp32C3_wifi_ops, s,
                          TYPE_ESP32_WIFI, 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    memset(s->mem,0,sizeof(s->mem));
    Esp32_WLAN_setup_ap(dev, s);
    
}
static Property esp32C3_wifi_properties[] = {
    DEFINE_NIC_PROPERTIES(Esp32WifiState, conf),
    DEFINE_PROP_END_OF_LIST(),
};

static void esp32C3_wifi_class_init(ObjectClass *klass, void *data)
{
	ResettablePhases rp;
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = esp32C3_wifi_realize;
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
    dc->desc = "Esp32C3 WiFi";
    device_class_set_props(dc, esp32C3_wifi_properties);
    ResettableClass *rc = RESETTABLE_CLASS(klass);
    resettable_class_set_parent_phases(rc, esp32c3_wifi_reset_enter, NULL, NULL, &rp);
}


static const TypeInfo esp32C3_wifi_info = {
    .name = TYPE_ESP32_WIFI,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32WifiState),
    .class_init    = esp32C3_wifi_class_init,
};

static void esp32C3_wifi_register_types(void)
{
    type_register_static(&esp32C3_wifi_info);
}

type_init(esp32C3_wifi_register_types)
