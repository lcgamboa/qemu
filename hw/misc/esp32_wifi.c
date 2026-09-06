#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qemu/guest-random.h"
#include "qapi/error.h"
#include "sysemu/sysemu.h"
#include "hw/hw.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "hw/misc/esp32_wifi.h"
#include "exec/address-spaces.h"
#include "esp32_wlan_packet.h"
#include "hw/qdev-properties.h"

#define DEBUG 0

#if DEBUG

static const char * get_reg_name(hwaddr addr){
    switch (addr)
    {
        case A_WIFI_BSSID_ADDR_FST_0:
          return "WIFI_BSSID_ADDR_FST_0";
          break;
        case A_WIFI_BSSID_ADDR_SND_0:
          return "WIFI_BSSID_ADDR_SND_0";
          break;
        case A_WIFI_BSSID_ADDR_SND_1:
          return "WIFI_BSSID_ADDR_SND_1";
          break;
        case A_WIFI_BSSID_FILTER_FST_0:
          return "WIFI_BSSID_FILTER_FST_0";
          break;
        case A_WIFI_BSSID_FILTER_SND_0:
          return "WIFI_BSSID_FILTER_SND_0";
          break;
        case A_WIFI_BSSID_FILTER_FST_1:
          return "WIFI_BSSID_FILTER_FST_1";
          break;
        case A_WIFI_BSSID_FILTER_SND_1:
          return "WIFI_BSSID_FILTER_SND_1";
          break;
        case A_WIFI_MAC_ADDR_FST_0:
          return "WIFI_MAC_ADDR_FST_0";
          break;
        case A_WIFI_MAC_ADDR_SND_0:
          return "WIFI_MAC_ADDR_SND_0";
          break;
        case A_WIFI_MAC_ADDR_FST_1:
          return "WIFI_MAC_ADDR_FST_1";
          break;
        case A_WIFI_MAC_ADDR_SND_1:
          return "WIFI_MAC_ADDR_SND_1";
          break;
        case A_WIFI_MAC_FILTER_FST_0:
          return "WIFI_MAC_FILTER_FST_0";
          break;
        case A_WIFI_MAC_FILTER_SND_0:
          return "WIFI_MAC_FILTER_SND_0";
          break;
        case A_WIFI_MAC_FILTER_FST_1:
          return "WIFI_MAC_FILTER_FST_1";
          break;
        case A_WIFI_MAC_FILTER_SND_1:
          return "WIFI_MAC_FILTER_SND_1";
          break;
        case A_WIFI_RXBUF_INIT_BITMASK:
          return "WIFI_RXBUF_INIT_BITMASK";
          break;
        case A_WIFI_DMA_IN_STATUS:
          return "WIFI_DMA_IN_STATUS";
          break ;
        case  A_WIFI_DMA_INLINK:
          return "WIFI_DMA_INLINK";
          break;       
        case A_WIFI_NEXT_RX_DSCR:
          return "WIFI_NEXT_RX_DSCR";
          break;
        case A_WIFI_LAST_RX_DSCR:
          return "WIFI_LAST_RX_DSCR";
          break;
        case A_WIFI_LAST_RXBUF_INIT_09C:
          return "WIFI_LAST_RXBUF_INIT_09C";
          break;
        case A_WIFI_RX_POLICY_0:
          return "WIFI_RX_POLICY_0";
          break;
        case A_WIFI_RX_POLICY_1:
          return "WIFI_RX_POLICY_1";
          break;
        case A_WIFI_RX_POLICY_2:
          return "WIFI_RX_POLICY_2";
          break;
        case A_WIFI_RX_POLICY_3:
          return "WIFI_RX_POLICY_3";
          break;
        case A_WIFI_PROMISC_MISC_BITMASK0:
          return "WIFI_PROMISC_MISC_BITMASK0";
          break;
        case A_WIFI_PROMISC_MISC_BITMASK1:
          return "WIFI_PROMISC_MISC_BITMASK1";
          break;
        case A_WIFI_PROMISC_MISC_BITMASK2:
          return "WIFI_PROMISC_MISC_BITMASK2";
          break;
        case A_WIFI_PROMISC_MISC_BITMASK3:
          return "WIFI_PROMISC_MISC_BITMASK3";
          break;
        case A_WIFI_RXBUF_INIT_HIGH_ADDR_0:
          return "WIFI_RXBUF_INIT_HIGH_ADDR_0";
          break;
        case A_WIFI_LAST_RXBUF_INIT_148:
          return "WIFI_LAST_RXBUF_INIT_148";
          break;
        case A_WIFI_LAST_RXBUF_INIT_14C:
          return "WIFI_LAST_RXBUF_INIT_14C";
          break;
        case A_WIFI_LAST_RXBUF_INIT_158:
          return "WIFI_LAST_RXBUF_INIT_158";
          break;
        case A_WIFI_LAST_RXBUF_INIT_164:
          return "WIFI_LAST_RXBUF_INIT_164";
          break;
        case A_WIFI_ANTENNA_INIT_284:
          return "WIFI_ANTENNA_INIT_284";
          break;
        case A_WIFI_AUTOACK_INIT_400:
          return "WIFI_AUTOACK_INIT_400";
          break;
        case A_WIFI_AUTOACK_INIT_404:
          return "WIFI_AUTOACK_INIT_404";
          break;
        case A_WIFI_AUTOACK_INIT_408:
          return "WIFI_AUTOACK_INIT_408";
          break;
        case A_WIFI_AUTOACK_INIT_40C:
          return "WIFI_AUTOACK_INIT_40C";
          break;
        case A_WIFI_AUTOACK_INIT_410:
          return "WIFI_AUTOACK_INIT_410";
          break;
        case A_WIFI_AUTOACK_INIT_414:
          return "WIFI_AUTOACK_INIT_414";
          break;
        case A_WIFI_LOW_RATE_418:
          return "WIFI_LOW_RATE_418";
          break;
        case A_WIFI_LOW_RATE_41C:
          return "WIFI_LOW_RATE_41C";
          break;
        case A_WIFI_MAYBE_TIMESTAMP:
          return "WIFI_MAYBE_TIMESTAMP";
          break;
        case A_WIFI_PROMISC_CONTROL_PKT:
          return "WIFI_PROMISC_CONTROL_PKT";
          break;
        case A_WIFI_DMA_INT_STATUS:
          return "WIFI_DMA_INT_STATUS";
          break;
        case A_WIFI_DMA_INT_CLR:
          return "WIFI_DMA_INT_CLR";
          break;
        case A_WIFI_MAYBE_PWR_CTL:
          return "WIFI_MAYBE_PWR_CTL";
          break;
        case A_WIFI_TXQ_CLR_STATE_COLL_TIMEOUT:
          return "WIFI_TXQ_CLR_STATE_COLL_TIMEOUT";
          break;
        case A_WIFI_TXQ_STATE_COLL_TIMEOUT:
          return "WIFI_TXQ_STATE_COLL_TIMEOUT";
          break;
        case A_WIFI_TXQ_CLR_STATE_COMPLETE:
          return "WIFI_TXQ_CLR_STATE_COMPLETE";
          break;
        case A_WIFI_TXQ_STATE_COMPLETE:
          return "WIFI_TXQ_STATE_COMPLETE";
          break;
        case A_WIFI_TX_CONFIG_0:
          return "WIFI_TX_CONFIG_0";
          break;
        case A_WIFI_DMA_OUTLINK:
          return "WIFI_DMA_OUTLINK";
          break;
        case A_WIFI_DMA_OUT_STATUS:
          return "WIFI_DMA_OUT_STATUS";
          break ;
        case A_WIFI_TXRX_INIT_10C:
          return "WIFI_TXRX_INIT_10C" ;
          break;
        case A_WIFI_TXRX_INIT_114:
          return "WIFI_TXRX_INIT_114" ;
          break;       
        case A_WIFI_TXRX_INIT_C1C:
          return "WIFI_TXRX_INIT_C1C" ;
          break;
        case A_WIFI_TXRX_INIT_C20:
          return "WIFI_TXRX_INIT_C20" ;
          break;
        case A_WIFI_TXRX_INIT_C24:
          return "WIFI_TXRX_INIT_C24" ;
          break;
        case A_WIFI_TXRX_INIT_C54:
          return "WIFI_TXRX_INIT_C54" ;
          break;
        case A_WIFI_TXRX_INIT_C5C:
          return "WIFI_TXRX_INIT_C5C" ;
          break;
        case A_WIFI_TXRX_INIT_C6C:
          return "WIFI_TXRX_INIT_C6C" ;
          break;
        case A_WIFI_TXRX_INIT_C74:
          return "WIFI_TXRX_INIT_C74" ;
          break;
        case A_WIFI_TXRX_INIT_C78:
          return "WIFI_TXRX_INIT_C78" ;
          break;
        case A_WIFI_TXRX_INIT_C88:
          return "WIFI_TXRX_INIT_C88" ;
          break;
        case A_WIFI_TXRX_INIT_CAC:
          return "WIFI_TXRX_INIT_CAC" ;
          break;
        case A_WIFI_TXRX_INIT_D78:
          return "WIFI_TXRX_INIT_D78";
          break;      
        case A_WIFI_TXRX_INIT_288:
          return "WIFI_TXRX_INIT_288";
          break;
    }
    return "**************";
}

static void macprint(const uint8_t *p, const char * name) {
    printf("%s: %02x:%02x:%02x:%02x:%02x:%02x\n",name, p[0],p[1],p[2],p[3],p[4],p[5]);
}

#endif

static uint64_t esp32_wifi_read(void *opaque, hwaddr addr, unsigned int size)
{
    
    Esp32WifiState *s = ESP32_WIFI(opaque);
    uint32_t r = s->mem[addr/4];
    
    switch(addr) {
        case A_WIFI_DMA_INLINK:
            r=s->dma_inlink_address;
            break;
        case A_WIFI_DMA_IN_STATUS:
            r=0;
            break;
        case A_WIFI_DMA_INT_STATUS:
        case A_WIFI_DMA_INT_CLR:
            r=s->raw_interrupt;
            break;
        case A_WIFI_TXQ_STATE_COMPLETE:
        case A_WIFI_DMA_OUT_STATUS:
            r=1;
            break;           
    }
#if DEBUG
    printf("esp32_wifi_read  %20s(0x%04lx)= 0x%08x\n",get_reg_name(addr),(unsigned long) addr,r);
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

static void esp32_wifi_write(void *opaque, hwaddr addr, uint64_t value,
                                 unsigned int size) {
    Esp32WifiState *s = ESP32_WIFI(opaque);
#if DEBUG    
    printf("esp32_wifi_write %20s(0x%04lx)= 0x%08lx\n",get_reg_name(addr),(unsigned long) addr, (unsigned long) value);
#endif
    
    switch (addr) {
        case A_WIFI_DMA_INLINK:
            s->dma_inlink_address = value;
            s->dma_inlink_offset = value;
            break;
        case A_WIFI_DMA_INT_CLR:
            s->raw_interrupt &= ~value;
            if(s->raw_interrupt==0)
                qemu_set_irq(s->irq, 0);
            break;
        case A_WIFI_DMA_OUTLINK:
            if (value & 0xc0000000) {
                // do a DMA transfer to the hardware from esp32 memory
                mac80211_frame frame;
                dma_list_item item;
                unsigned memaddr = (0x3ff00000 | (value & 0xfffff));
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
    uint8_t *header=malloc(sizeof(wifi_pkt_rx_ctrl_t)+length);
    memset(header,0,sizeof(wifi_pkt_rx_ctrl_t)+length);
    wifi_pkt_rx_ctrl_t *pkt=(wifi_pkt_rx_ctrl_t *)header;
    *pkt=(wifi_pkt_rx_ctrl_t){
        .rssi=(signal_strength+(rand()%10)+96),
        .rate=11,
        .sig_len=length,
        .sig_len_copy=length,
        .legacy_length=length,
        .noise_floor=-97,
        .channel=esp32_wifi_channel,
        .timestamp=(qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)/1000),
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

#if DEBUG     
    macprint(frame->destination_address,"1 -destination_address");
    macprint(frame->bssid_address,"2 -bssid_address");
    macprint((uint8_t *)s->mem+A_WIFI_MAC_ADDR_FST_0,"3 -WIFI_MAC_ADDR_FST_0");
    macprint((uint8_t *)s->mem+A_WIFI_MAC_ADDR_FST_1,"4 -A_WIFI_MAC_ADDR_FST_1");
    printf("match 1_3=%i 1_4=%i 2_3=%i 2_4=%i\n", pkt->damatch0, pkt->damatch1, pkt->bssidmatch0, pkt->bssidmatch1);
#endif

    memcpy(header+sizeof(wifi_pkt_rx_ctrl_t),frame,length);
    length+=sizeof(wifi_pkt_rx_ctrl_t);
    // do a DMA transfer from the hardware to esp32 memory
    dma_list_item item;
    address_space_read(&address_space_memory, s->dma_inlink_offset, MEMTXATTRS_UNSPECIFIED, &item, 12);
    address_space_write(&address_space_memory, item.address, MEMTXATTRS_UNSPECIFIED, header, length);
    item.length=length;
    item.eof=1;
    address_space_write(&address_space_memory, s->dma_inlink_offset, MEMTXATTRS_UNSPECIFIED,&item,4);
    s->dma_inlink_offset=item.next;
    if(s->dma_inlink_offset == 0) s->dma_inlink_offset = s->dma_inlink_address;
    set_interrupt(s,0x1000024);
    free(header);
}

static const MemoryRegionOps esp32_wifi_ops = {
    .read =  esp32_wifi_read,
    .write = esp32_wifi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32_wifi_realize(DeviceState *dev, Error **errp)
{
    Esp32WifiState *s = ESP32_WIFI(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    s->dma_inlink_address=0;
    s->dma_inlink_offset=0;

    memory_region_init_io(&s->iomem, OBJECT(dev), &esp32_wifi_ops, s,
                          TYPE_ESP32_WIFI, 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    memset(s->mem,0,sizeof(s->mem));
    Esp32_WLAN_setup_ap(dev, s);
}
static Property esp32_wifi_properties[] = {
    DEFINE_NIC_PROPERTIES(Esp32WifiState, conf),
    DEFINE_PROP_END_OF_LIST(),
};

static void esp32_wifi_reset_enter(Object *obj, ResetType type)
{
    Esp32WifiState *s = ESP32_WIFI(obj);
    if(s)
	    s->ap_state=0;
}

static void esp32_wifi_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettablePhases rp;
    dc->realize = esp32_wifi_realize;
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
    dc->desc = "Esp32 WiFi";
    device_class_set_props(dc, esp32_wifi_properties);
    ResettableClass *rc = RESETTABLE_CLASS(klass);
    resettable_class_set_parent_phases(rc, esp32_wifi_reset_enter, NULL, NULL,
                                   &rp);
}

static const TypeInfo esp32_wifi_info = {
    .name = TYPE_ESP32_WIFI,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32WifiState),
    .class_init    = esp32_wifi_class_init,
};

static void esp32_wifi_register_types(void)
{
    type_register_static(&esp32_wifi_info);
}

type_init(esp32_wifi_register_types)
