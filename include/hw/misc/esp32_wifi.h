#pragma once

#include "hw/hw.h"
#include "hw/registerfields.h"
#include "hw/sysbus.h"
//#include "hw/misc/esp32_reg.h"
#include "sysemu/sysemu.h"
#include "net/net.h"

#define TYPE_ESP32_WIFI "esp32_wifi"
#define ESP32_WIFI(obj) OBJECT_CHECK(Esp32WifiState, (obj), TYPE_ESP32_WIFI)

typedef struct dma_list_item {
    unsigned size:12;
    unsigned length:12;
    unsigned :6;
    unsigned eof:1;
    unsigned owner:1;
    uint32_t address;
    uint32_t next;
} QEMU_PACKED dma_list_item;

typedef struct Esp32WifiState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    int raw_interrupt;
    qemu_irq irq;
    uint32_t mem[1024];
    unsigned int dma_inlink_address;
    unsigned int dma_inlink_offset;
    uint32_t ap_state;
    int inject_queue_size;
    struct mac80211_frame *inject_queue;
    int inject_timer_running;
    unsigned int inject_sequence_number;
    int beacon_ap;

    hwaddr receive_queue_address;
    uint32_t receive_queue_count;
    NICConf conf;
    NICState *nic;
    // various timers
    QEMUTimer *beacon_timer;
    QEMUTimer *inject_timer;
    QEMUTimer *wait_ack_timer;
    uint8_t ipaddr[4];              
    uint8_t macaddr[6];             

    uint8_t ap_ipaddr[4];               
    uint8_t ap_macaddr[6];     

    uint8_t associated_ap_macaddr[6]; 

    uint8_t softap_macaddr[6];       

    uint8_t mode;  

} Esp32WifiState;

#define Esp32_Mode_Station 0
#define Esp32_Mode_SoftAP  1


void Esp32_WLAN_handle_frame(Esp32WifiState *s, struct mac80211_frame *frame);
void Esp32_WLAN_setup_ap(DeviceState *dev,Esp32WifiState *s);
void Esp32_WLAN_reset_ap(Esp32WifiState *s);
void Esp32_sendFrame(Esp32WifiState *s, struct mac80211_frame *frame,int length, int signal_strength);
void Esp32_WLAN_frame_delivered(Esp32WifiState *s);

REG32(WIFI_BSSID_ADDR_FST_0, 0x000);
REG32(WIFI_BSSID_ADDR_SND_0, 0x004);
REG32(WIFI_BSSID_ADDR_FST_1, 0x008);
REG32(WIFI_BSSID_ADDR_SND_1, 0x00c);
REG32(WIFI_BSSID_FILTER_FST_0, 0x020);
REG32(WIFI_BSSID_FILTER_SND_0, 0x024);
REG32(WIFI_BSSID_FILTER_FST_1, 0x028);
REG32(WIFI_BSSID_FILTER_SND_1, 0x02c);

REG32(WIFI_MAC_ADDR_FST_0, 0x040);
REG32(WIFI_MAC_ADDR_SND_0, 0x044);
REG32(WIFI_MAC_ADDR_FST_1, 0x048);
REG32(WIFI_MAC_ADDR_SND_1, 0x04c);
REG32(WIFI_MAC_FILTER_FST_0, 0x060);
REG32(WIFI_MAC_FILTER_SND_0, 0x064);
REG32(WIFI_MAC_FILTER_FST_1, 0x068);
REG32(WIFI_MAC_FILTER_SND_1, 0x06c);

REG32(WIFI_RXBUF_INIT_BITMASK, 0x80); // only set in mac_rxbuf_init
REG32(WIFI_DMA_IN_STATUS, 0x84);
REG32(WIFI_DMA_INLINK, 0x88);
REG32(WIFI_NEXT_RX_DSCR, 0x8c);
REG32(WIFI_LAST_RX_DSCR, 0x90);
REG32(WIFI_LAST_RXBUF_INIT_09C, 0x09C); // only set in hal_init and mac_last_rxbuf_init
REG32(WIFI_RX_POLICY_0, 0xd8);
REG32(WIFI_RX_POLICY_1, 0xdc);
REG32(WIFI_RX_POLICY_2, 0xe0);
REG32(WIFI_RX_POLICY_3, 0xe4);
REG32(WIFI_PROMISC_MISC_BITMASK0, 0xf8);
REG32(WIFI_PROMISC_MISC_BITMASK1, 0xfc);
REG32(WIFI_PROMISC_MISC_BITMASK2, 0x100);
REG32(WIFI_PROMISC_MISC_BITMASK3, 0x104);

REG32(WIFI_RXBUF_INIT_HIGH_ADDR_0, 0x118); // only set in mac_rxbuf_init
REG32(WIFI_RXBUF_INIT_LOW_ADDR_0, 0x11c);  // only set in mac_rxbuf_init
REG32(WIFI_RXBUF_INIT_HIGH_ADDR_1, 0x120); // only set in mac_rxbuf_init
REG32(WIFI_RXBUF_INIT_LOW_ADDR_1, 0x124);  // only set in mac_rxbuf_init

REG32(WIFI_LAST_RXBUF_INIT_148, 0x148); // only set in mac_last_rxbuf_init
REG32(WIFI_LAST_RXBUF_INIT_14C, 0x14C); // only set in mac_last_rxbuf_init
REG32(WIFI_LAST_RXBUF_INIT_158, 0x158); // only set in mac_last_rxbuf_init
REG32(WIFI_LAST_RXBUF_INIT_164, 0x164); // only set in mac_last_rxbuf_init

REG32(WIFI_ANTENNA_INIT_284, 0x284);

REG32(WIFI_AUTOACK_INIT_400, 0x400); // only set in hal_mac_rate_autoack_init
REG32(WIFI_AUTOACK_INIT_404, 0x404); // only set in hal_mac_rate_autoack_init
REG32(WIFI_AUTOACK_INIT_408, 0x408); // only set in hal_mac_rate_autoack_init
REG32(WIFI_AUTOACK_INIT_40C, 0x40c); // only set in hal_mac_rate_autoack_init
REG32(WIFI_AUTOACK_INIT_410, 0x410); // only set in hal_mac_rate_autoack_init
REG32(WIFI_AUTOACK_INIT_414, 0x414); // only set in hal_mac_rate_autoack_init

REG32(WIFI_LOW_RATE_418, 0x418);
REG32(WIFI_LOW_RATE_41C, 0x41C);

// 0x800 - 0x814 are cryptography registers, likely for WPA and WEP

REG32(WIFI_MAYBE_TIMESTAMP, 0xc00);
REG32(WIFI_PROMISC_CONTROL_PKT, 0xc40);
REG32(WIFI_DMA_INT_STATUS, 0xc48);
REG32(WIFI_DMA_INT_CLR, 0xc4c);
REG32(WIFI_MAYBE_PWR_CTL, 0xcb8);
REG32(WIFI_TXQ_CLR_STATE_COLL_TIMEOUT, 0xcbc);
REG32(WIFI_TXQ_STATE_COLL_TIMEOUT, 0xcc0);
REG32(WIFI_TXQ_CLR_STATE_COMPLETE, 0xcc4);
REG32(WIFI_TXQ_STATE_COMPLETE, 0xcc8);
REG32(WIFI_TX_CONFIG_0, 0xd1c);
REG32(WIFI_DMA_OUTLINK, 0xd20);
REG32(WIFI_DMA_OUT_STATUS, 0xd24);


// Wifi registers only used for initialization
REG32(WIFI_TXRX_INIT_10C, 0x10c); // only set in mac_txrx_init
REG32(WIFI_TXRX_INIT_114, 0x114); // only set in mac_txrx_init
REG32(WIFI_TXRX_INIT_C1C, 0xc1c); // only set in mac_txrx_init
REG32(WIFI_TXRX_INIT_C20, 0xc20); // only set in mac_txrx_init
REG32(WIFI_TXRX_INIT_C24, 0xc24); // only set in mac_txrx_init
REG32(WIFI_TXRX_INIT_C54, 0xc54); // only set in mac_txrx_init
REG32(WIFI_TXRX_INIT_C5C, 0xc5c); // only set in mac_txrx_init
REG32(WIFI_TXRX_INIT_C6C, 0xc6c); // only set in mac_txrx_init
REG32(WIFI_TXRX_INIT_C74, 0xc74); // only set in mac_txrx_init
REG32(WIFI_TXRX_INIT_C78, 0xc78); // only set in mac_txrx_init
REG32(WIFI_TXRX_INIT_C88, 0xc88); // only set in mac_txrx_init
REG32(WIFI_TXRX_INIT_CAC, 0xcac); // only set in mac_txrx_init
REG32(WIFI_TXRX_INIT_D78, 0xd78); // only set in mac_txrx_init

REG32(WIFI_TXRX_INIT_288, 0x288); // set in mac_txrx_init and hal_deinit

