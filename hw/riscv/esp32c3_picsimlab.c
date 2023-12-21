/*
 * ESP32-C3 SoC and machine
 *
 * Copyright (c) 2019-2022 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "hw/qdev-properties.h"
#include "qemu/units.h"
#include "qemu/datadir.h"
#include "qapi/error.h"
#include "hw/hw.h"
#include "hw/boards.h"
#include "hw/loader.h"
#include "hw/riscv/riscv_hart.h"
#include "target/riscv/esp_cpu.h"
#include "hw/riscv/boot.h"
#include "hw/riscv/numa.h"
#include "sysemu/device_tree.h"
#include "sysemu/sysemu.h"
#include "sysemu/kvm.h"
#include "sysemu/runstate.h"
#include "sysemu/reset.h"
#include "net/net.h"
#include "hw/misc/esp32c3_reg.h"
#include "hw/misc/esp32c3_rtc_cntl.h"
#include "hw/misc/esp32c3_cache.h"
#include "hw/char/esp32c3_uart.h"
#include "hw/gpio/esp32c3_gpio.h"
#include "hw/nvram/esp32c3_efuse.h"
#include "hw/riscv/esp32c3_clk.h"
#include "hw/riscv/esp32c3_intmatrix.h"
#include "hw/misc/esp32c3_sha.h"
#include "hw/timer/esp32c3_timg.h"
#include "hw/timer/esp32c3_systimer.h"
#include "hw/ssi/esp32c3_spi.h"
#include "hw/misc/esp32c3_rtc_cntl.h"
#include "hw/misc/esp32c3_aes.h"
#include "hw/misc/esp32c3_rsa.h"
#include "hw/misc/esp32c3_hmac.h"
#include "hw/misc/esp32c3_ds.h"
#include "hw/misc/esp32c3_jtag.h"
#include "hw/dma/esp32c3_gdma.h"
#include "hw/display/esp_rgb.h"
#include "hw/misc/esp32c3_iomux.h"
#include "hw/misc/esp32c3_saradc.h"
#include "hw/misc/esp32c3_ledc.h"
#include "hw/irq.h"
#include "hw/misc/unimp.h"

#include "hw/misc/esp32c3_wifi.h"
#include "hw/misc/esp32_phya.h"
#include "hw/misc/esp32c3_ana.h"
#include "hw/misc/esp32_fe.h"
#include "hw/misc/esp32c3_pwrmng.h"

#ifdef _WIN32
#include "chardev/char-win.h"
#else
#include "chardev/char-fd.h"
#endif
#include "chardev/char-serial.h"

#define ESP32C3_IO_WARNING          0

#define ESP32C3_RESET_ADDRESS       0x40000000
#define ESP32C3_RESET_GPIO_NAME     "esp32c3.machine.reset_gpio"
#define MB (1024*1024)


/* Define a new "class" which derivates from "MachineState" */
struct Esp32C3MachineState {
    MachineState parent;

    /* Attributes specific to our class */
    EspRISCVCPU soc;
    BusState periph_bus;
    MemoryRegion iomem;

    qemu_irq cpu_reset;

    DeviceState *eth; /* Ethernet controller */
    ESP32C3IntMatrixState intmatrix;
    ESP32C3UARTState uart[ESP32C3_UART_COUNT];
    ESP32C3GPIOState gpio;
    ESP32C3CacheState cache;
    ESP32C3EfuseState efuse;
    ESP32C3ClockState clock;
    ESP32C3GdmaState gdma;
    ESP32C3AesState aes;
    ESP32C3ShaState sha;
    ESP32C3RsaState rsa;
    ESP32C3HmacState hmac;
    ESP32C3DsState ds;
    ESP32C3TimgState timg[2];
    ESP32C3SysTimerState systimer;
    ESP32C3SpiState spi1;
    ESP32C3RtcCntlState rtccntl;
    ESP32C3UsbJtagState jtag;
    ESPRgbState rgb;
    Esp32C3IomuxState iomux;
    Esp32c3SarAdcState saradc;
    Esp32C3LEDCState ledc;

    Esp32WifiState wifi;
    Esp32PhyaState phya;
    Esp32C3AnaState ana;
    Esp32FeState fe;
    Esp32c3PwrMngState pwrmng;
};

/* Fake register used by ESP-IDF application to determine whether the code is running on real hardware or on QEMU */
#define A_SYSCON_ORIGIN_REG     0x3F8
/* Temporary macro for generating a random value from register SYSCON_RND_DATA_REG */
#define A_SYSCON_RND_DATA_REG 0x0B0

/* Temporary macro to mark the CPU as in non-debugging mode */
#define A_ASSIST_DEBUG_CORE_0_DEBUG_MODE_REG    0x098

/* Create a macro which defines the name of our new machine class */
#define TYPE_ESP32C3_MACHINE MACHINE_TYPE_NAME("esp32c3-picsimlab")

/* This will create a macro ESP32_MACHINE, which can be used to check and cast a generic MachineClass
 * to the specific class we defined above: Esp32C3MachineState. */
OBJECT_DECLARE_SIMPLE_TYPE(Esp32C3MachineState, ESP32C3_MACHINE)

/* Memory entries for ESP32-C3 */
enum MemoryRegions {
    ESP32C3_MEMREGION_IROM,
    ESP32C3_MEMREGION_DROM,
    ESP32C3_MEMREGION_DRAM,
    ESP32C3_MEMREGION_IRAM,
    ESP32C3_MEMREGION_RTCFAST,
    ESP32C3_MEMREGION_DCACHE,
    ESP32C3_MEMREGION_ICACHE,
    ESP32C3_MEMREGION_FRAMEBUF,
};

#define ESP32C3_INTERNAL_SRAM0_SIZE (16*1024)

static const struct MemmapEntry {
    hwaddr base;
    hwaddr size;
} esp32c3_memmap[] = {
    [ESP32C3_MEMREGION_IROM]    = { 0x40000000,  0x60000 },
    [ESP32C3_MEMREGION_DROM]    = { 0x3ff00000,  0x20000 },
    [ESP32C3_MEMREGION_DRAM]    = { 0x3fc80000,  0x60000 },
    /* Merge SRAM0 and SRAM1 into a single entry */
    [ESP32C3_MEMREGION_IRAM]    = { 0x4037c000,  0x60000 + ESP32C3_INTERNAL_SRAM0_SIZE },
    [ESP32C3_MEMREGION_RTCFAST] = { 0x50000000,   0x2000 },
    [ESP32C3_MEMREGION_DCACHE]  = { 0x3c000000, 0x800000 },
    [ESP32C3_MEMREGION_ICACHE]  = { 0x42000000, 0x800000 },
    /* Virtual Framebuffer, used for the graphical interface */
    [ESP32C3_MEMREGION_FRAMEBUF] = { 0x20000000, ESP_RGB_MAX_VRAM_SIZE }
};

static Esp32C3MachineState *global_s = NULL;

static qemu_irq *pout_irq; //TODO alloc this in object state
static qemu_irq *pdir_irq; //TODO alloc this in object state
static qemu_irq *psync_irq;
//static qemu_irq *spi_cs_irq;

static qemu_irq pin_irq[100];

typedef struct {
    void (*picsimlab_write_pin)(int pin, int value);
    void (*picsimlab_dir_pin)(int pin, int value);
    int (*picsimlab_i2c_event)(const uint8_t id, const uint8_t addr, const uint16_t event);
    uint8_t (*picsimlab_spi_event)(const uint8_t id, const uint16_t event);
    void (*picsimlab_uart_tx_event)(const uint8_t id, const uint8_t value);
    const short int * pinmap;    
} callbacks_t;

//prototypes
void qemu_picsimlab_register_callbacks(void * arg);
void qemu_picsimlab_set_apin(int chn,int value);
void qemu_picsimlab_set_pin(int pin,int value);
uint32_t * qemu_picsimlab_get_internals(int cfg);
uint32_t  qemu_picsimlab_get_TIOCM(void);
int qemu_picsimlab_flash_dump( int64_t offset, void *buf, int bytes);
void qemu_picsimlab_uart_receive(const int id, const uint8_t *buf, int size);

#define QEMU_INTERNAL_STRAP 0
#define QEMU_INTERNAL_GPIO_IN_SEL 1
#define QEMU_INTERNAL_GPIO_OUT_SEL 2
#define QEMU_INTERNAL_IOMUX_GPIOS 3

uint32_t *qemu_picsimlab_get_internals(int cfg) {
  switch (cfg) {
  case QEMU_INTERNAL_STRAP:
    return &global_s->gpio.parent.strap_mode;
    break;
  case QEMU_INTERNAL_GPIO_IN_SEL:
    return global_s->gpio.parent.gpio_in_sel;
    break;
  case QEMU_INTERNAL_GPIO_OUT_SEL:
    return global_s->gpio.parent.gpio_out_sel;
    break;
  case QEMU_INTERNAL_IOMUX_GPIOS:
    return global_s->iomux.parent.muxgpios;
    break;
  default:
    printf("Invalid internal request\n"); 
    break;
  }
  return NULL;
}

void uart_receive(void *opaque, const uint8_t *buf, int size);
int uart_can_receive(void *opaque);

void qemu_picsimlab_uart_receive(const int id, const uint8_t *buf, int size){
   uart_receive((void *) &global_s->uart[id], buf, size);
}

uint32_t qemu_picsimlab_get_TIOCM(void)
{  
   ESP32UARTState *s = ESP32_UART(&(global_s->uart[0].parent));

   uint32_t state = 0;
   
#ifdef _WIN32
   WinChardev *ws = WIN_CHARDEV(s->chr.chr);

   if (ws->file != INVALID_HANDLE_VALUE) {
     long unsigned int wstate;
     if(GetCommModemStatus(ws->file, &wstate)){
       if(wstate & MS_DSR_ON) state |= CHR_TIOCM_DSR;
       if(wstate & MS_CTS_ON) state |= CHR_TIOCM_CTS;
     }
     else{
       printf("GetCommModemStatus Erro %li\n", GetLastError());
     }
    }
#else
   qemu_chr_fe_ioctl(&s->chr, CHR_IOCTL_SERIAL_GET_TIOCM, &state); 
#endif
  
   return state;
}


static void place_holder(int pin,int value){};
static int place_holder2(const uint8_t id, const uint8_t addr, const uint16_t event){return 0;};
static uint8_t place_holder3(const uint8_t id, const uint16_t event){return 0;};
static void place_holder4(const uint8_t id, const uint8_t value){};

void (*picsimlab_write_pin)(int pin,int value) = place_holder;
void (*picsimlab_dir_pin)(int pin,int value) = place_holder;
int (*picsimlab_i2c_event)(const uint8_t id, const uint8_t addr, const uint16_t event) = place_holder2;
uint8_t (*picsimlab_spi_event)(const uint8_t id, const uint16_t event) = place_holder3;
void (*picsimlab_uart_tx_event)(const uint8_t id, const uint8_t value) = place_holder4;
static const short int * pinmap = NULL;

void qemu_picsimlab_register_callbacks(void * arg)
{
  const callbacks_t * callbacks = (const callbacks_t *) arg;

  picsimlab_write_pin = callbacks->picsimlab_write_pin;
  picsimlab_dir_pin = callbacks->picsimlab_dir_pin; 
  picsimlab_i2c_event = callbacks->picsimlab_i2c_event;
  picsimlab_spi_event = callbacks->picsimlab_spi_event;
  picsimlab_uart_tx_event = callbacks->picsimlab_uart_tx_event;
  pinmap = callbacks->pinmap;
}

void qemu_picsimlab_set_pin(int pin,int value)
{
   //qemu_mutex_lock_iothread ();
   if (value){
      qemu_irq_raise (pin_irq[pin]);
   }
   else{
      qemu_irq_lower (pin_irq[pin]);
   }
   //qemu_mutex_unlock_iothread ();
}

void qemu_picsimlab_set_apin(int chn,int value)
{
   Esp32c3SarAdcState *s = ESP32C3_SARADC(&(global_s->saradc));
   s->ADC_values[chn] = value;
}

int qemu_picsimlab_flash_dump( int64_t offset, void *buf, int bytes)
{
    if (global_s->cache.flash_blk){
       return blk_pread(global_s->cache.flash_blk,offset,bytes,buf,0);
    }
   return 0;
}

static void
pout_irq_handler(void *opaque, int n, int level)
{
   picsimlab_write_pin(n,level);
}

static void
pdir_irq_handler(void *opaque, int n, int dir)
{
   picsimlab_dir_pin(n, dir);
}

static void
psync_irq_handler(void *opaque, int n, int dir)
{
   picsimlab_dir_pin(-1, dir);
}
/*
static void
spi_cs_irq_handler(void *opaque, int n, int level)
{
    picsimlab_spi_event(n >> 2, ((((n & 3)<<1)|level)<<8)|0x01);
}
*/

static bool addr_in_range(hwaddr addr, hwaddr start, hwaddr end)
{
    return addr >= start && addr < end;
}

static uint64_t esp32c3_io_read(void *opaque, hwaddr addr, unsigned int size)
{
    if (addr_in_range(addr + ESP32C3_IO_START_ADDR, DR_REG_RTC_I2C_BASE, DR_REG_RTC_I2C_BASE + 0x100)) {
        return (uint32_t) 0xffffff;
    } else if (addr + ESP32C3_IO_START_ADDR == DR_REG_SYSCON_BASE + A_SYSCON_ORIGIN_REG) {
        /* Return "QEMU" as a 32-bit value */
        return 0x51454d55;
    } else if (addr + ESP32C3_IO_START_ADDR == DR_REG_SYSCON_BASE + A_SYSCON_RND_DATA_REG) {
        /* Return a random 32-bit value */
        static bool init = false;
        if (!init) {
            srand(time(NULL));
            init = true;
        }
        return rand();
    } else if (addr + ESP32C3_IO_START_ADDR == DR_REG_ASSIST_DEBUG_BASE + A_ASSIST_DEBUG_CORE_0_DEBUG_MODE_REG) {
        return 0;
    } else {
#if ESP32C3_IO_WARNING
        if (addr_in_range(addr + ESP32C3_IO_START_ADDR, DR_REG_SYSCON_BASE,DR_REG_SYSCON_BASE + 0x1000)){
            warn_report("[ESP32-C3] SYSCON          Unsupported read to $%08lx\n", ESP32C3_IO_START_ADDR + addr);  
        }
        else if (addr_in_range(addr + ESP32C3_IO_START_ADDR, DR_REG_FE_BASE,DR_REG_FE_BASE + 0x1000)){
            warn_report("[ESP32-C3] FE              Unsupported read to $%08lx\n", ESP32C3_IO_START_ADDR + addr);  
        }
        else if (addr_in_range(addr + ESP32C3_IO_START_ADDR, DR_REG_BB_BASE,DR_REG_BB_BASE + 0x1000)){
            warn_report("[ESP32-C3] BB              Unsupported read to $%08lx\n", ESP32C3_IO_START_ADDR + addr);  
        }
        else if (addr_in_range(addr + ESP32C3_IO_START_ADDR, DR_REG_NRX_BASE,DR_REG_NRX_BASE + 0x1000)){
            warn_report("[ESP32-C3] NRX             Unsupported read to $%08lx\n", ESP32C3_IO_START_ADDR + addr);  
        }
        else if (addr_in_range(addr + ESP32C3_IO_START_ADDR, DR_REG_SPI0_BASE,DR_REG_SPI0_BASE + 0x1000)){
            warn_report("[ESP32-C3] SPI0            Unsupported read to $%08lx\n", ESP32C3_IO_START_ADDR + addr);  
        }
        else if (addr_in_range(addr + ESP32C3_IO_START_ADDR, DR_REG_SENSITIVE_BASE,DR_REG_SENSITIVE_BASE + 0x1000)){
            warn_report("[ESP32-C3] SENSITIVE       Unsupported read to $%08lx\n", ESP32C3_IO_START_ADDR + addr);  
        }
        else if (addr_in_range(addr + ESP32C3_IO_START_ADDR, DR_REG_ASSIST_DEBUG_BASE,DR_REG_ASSIST_DEBUG_BASE + 0x1000)){
            warn_report("[ESP32-C3] ASSIST DEBUG    Unsupported read to $%08lx\n", ESP32C3_IO_START_ADDR + addr);  
        }        
        else if (addr_in_range(addr + ESP32C3_IO_START_ADDR, DR_REG_FE2_BASE,DR_REG_FE2_BASE + 0x1000)){
            warn_report("[ESP32-C3] FE2             Unsupported read to $%08lx\n", ESP32C3_IO_START_ADDR + addr);  
        }
        else{
            warn_report("[ESP32-C3] <UNKNOWN>       Unsupported read to $%08lx\n", ESP32C3_IO_START_ADDR + addr);
        }
#endif
    }
    return 0;
}

static void esp32c3_io_write(void *opaque, hwaddr addr, uint64_t value, unsigned int size)
{
#if ESP32C3_IO_WARNING
        if (addr_in_range(addr + ESP32C3_IO_START_ADDR, DR_REG_SYSCON_BASE,DR_REG_SYSCON_BASE + 0x1000)){
            warn_report("[ESP32-C3] SYSCON          Unsupported write to $%08lx = %08lx\n", ESP32C3_IO_START_ADDR + addr, value);  
        }
        else if (addr_in_range(addr + ESP32C3_IO_START_ADDR, DR_REG_FE_BASE,DR_REG_FE_BASE + 0x1000)){
            warn_report("[ESP32-C3] FE              Unsupported write to $%08lx = %08lx\n", ESP32C3_IO_START_ADDR + addr, value);  
        }
        else if (addr_in_range(addr + ESP32C3_IO_START_ADDR, DR_REG_BB_BASE,DR_REG_BB_BASE + 0x1000)){
            warn_report("[ESP32-C3] BB              Unsupported write to $%08lx = %08lx\n", ESP32C3_IO_START_ADDR + addr, value);  
        }
        else if (addr_in_range(addr + ESP32C3_IO_START_ADDR, DR_REG_NRX_BASE,DR_REG_NRX_BASE + 0x1000)){
            warn_report("[ESP32-C3] NRX             Unsupported write to $%08lx = %08lx\n", ESP32C3_IO_START_ADDR + addr, value);  
        }
        else if (addr_in_range(addr + ESP32C3_IO_START_ADDR, DR_REG_SPI0_BASE,DR_REG_SPI0_BASE + 0x1000)){
            warn_report("[ESP32-C3] SPI0            Unsupported write to $%08lx = %08lx\n", ESP32C3_IO_START_ADDR + addr, value);  
        }
        else if (addr_in_range(addr + ESP32C3_IO_START_ADDR, DR_REG_SENSITIVE_BASE,DR_REG_SENSITIVE_BASE + 0x1000)){
            warn_report("[ESP32-C3] SENSITIVE       Unsupported write to $%08lx = %08lx\n", ESP32C3_IO_START_ADDR + addr, value);  
        }
        else if (addr_in_range(addr + ESP32C3_IO_START_ADDR, DR_REG_ASSIST_DEBUG_BASE,DR_REG_ASSIST_DEBUG_BASE + 0x1000)){
            warn_report("[ESP32-C3] ASSIST DEBUG    Unsupported write to $%08lx = %08lx\n", ESP32C3_IO_START_ADDR + addr, value);  
        }        
        else if (addr_in_range(addr + ESP32C3_IO_START_ADDR, DR_REG_FE2_BASE,DR_REG_FE2_BASE + 0x1000)){
            warn_report("[ESP32-C3] FE2             Unsupported write to $%08lx = %08lx\n", ESP32C3_IO_START_ADDR + addr, value);  
        }
        else{
            warn_report("[ESP32-C3] <UNKNOWN>       Unsupported write to $%08lx = %08lx\n", ESP32C3_IO_START_ADDR + addr, value);
        }
#endif
} 


/* Define operations for I/OS */
static const MemoryRegionOps esp32c3_io_ops = {
    .read =  esp32c3_io_read,
    .write = esp32c3_io_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};


/**
 * @brief Callback invoked when SoC's ESP32C3_RESET_GPIO_NAME pin is toggled
 */
static void esp32c3_reset_request(void* opaque, int n, int level)
{
    if (level) {
        ShutdownCause cause = SHUTDOWN_CAUSE_GUEST_RESET;
        qemu_system_reset_request(cause);
    }
}


static void esp32c3_init_spi_flash(Esp32C3MachineState *ms, BlockBackend* blk)
{
    DeviceState *spi_master = DEVICE(&ms->spi1);
    BusState* spi_bus = qdev_get_child_bus(spi_master, "spi");
    const char* flash_model = NULL;
    int64_t image_size = blk_getlength(blk);

    switch (image_size) {
        case 2 * MB:
            flash_model = "w25x16";
            break;
        case 4 * MB:
            flash_model = "gd25q32";
            break;
        case 8 * MB:
            flash_model = "gd25q64";
            break;
        case 16 * MB:
            flash_model = "is25lp128";
            break;
        default:
            error_report("Drive size error: only 2, 4, 8, and 16MB images are supported");
            return;
    }

    /* Create the SPI flash model */
    DeviceState *flash_dev = qdev_new(flash_model);
    qdev_prop_set_drive(flash_dev, "drive", blk);

    /* Realize the SPI flash, its "drive" (blk) property must already be set! */
    qdev_realize(flash_dev, spi_bus, &error_fatal);
    qdev_connect_gpio_out_named(spi_master, SSI_GPIO_CS, 0,
                                qdev_get_gpio_in_named(flash_dev, SSI_GPIO_CS, 0));
}

static void esp32c3_add_unimp_device(MemoryRegion *dest, const char* name, hwaddr dport_base_addr, size_t size, uint32_t default_value)
{
    create_unimplemented_device_default_value(name, dport_base_addr, size, default_value);
}

static void esp32c3_init_openeth(Esp32C3MachineState *ms)
{
    
    SysBusDevice* sbd = NULL;
    MemoryRegion* sys_mem = get_system_memory();
    for(int i=0;i<nb_nics;i++) {
        const char* type_openeth = "open_eth";
        NICInfo *nd = &nd_table[i];
        if (nd->used && nd->model && strcmp(nd->model, type_openeth) == 0)
        {
            const char* type_openeth = "open_eth";
            /* Create a new OpenCores Ethernet component */
            DeviceState *open_eth_dev = qdev_new(type_openeth);
            ms->eth = open_eth_dev;
            qdev_set_nic_properties(open_eth_dev, nd);
            sbd = SYS_BUS_DEVICE(open_eth_dev);
            sysbus_realize(sbd, &error_fatal);

            /* OpenCores Ethernet has two memory regions: one for registers and one for descriptors,
             * we need to provide one I/O range for each of them */
            memory_region_add_subregion_overlap(sys_mem, DR_REG_EMAC_BASE, sysbus_mmio_get_region(sbd, 0), 0);
            memory_region_add_subregion_overlap(sys_mem, DR_REG_EMAC_BASE + 0x400, sysbus_mmio_get_region(sbd, 1), 0);

            sysbus_connect_irq(sbd, 0, qdev_get_gpio_in(DEVICE(&ms->intmatrix), ETS_ETH_MAC_INTR_SOURCE));
        }

        if (nd->used && nd->model && strcmp(nd->model, TYPE_ESP32C3_WIFI) == 0) {
            
            //get macaddres from efuse file
            device_cold_reset(DEVICE(&ms->efuse));
            char * mptr = (char *)&ms->efuse.efuses.rd_mac_spi_sys_0;
            for(int i=0; i < 6 ; i++){
                ms->wifi.macaddr[i]=mptr[(5-i)];
            }
            qdev_set_nic_properties(DEVICE(&ms->wifi), nd);
            sbd = SYS_BUS_DEVICE(DEVICE(&ms->wifi));
            sysbus_realize_and_unref(sbd, &error_fatal);

            MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ms->wifi), 0);
            memory_region_add_subregion_overlap(sys_mem, DR_REG_WIFI_BASE, mr, 0);

            sysbus_connect_irq(SYS_BUS_DEVICE(&ms->wifi), 0,
                           qdev_get_gpio_in(DEVICE(&ms->intmatrix), ETS_WIFI_MAC_INTR_SOURCE));
        } 
    }
}


static void esp32c3_machine_init(MachineState *machine)
{
    /* First thing to do is to check if a drive format and a file ahve been passed through the command line.
     * In fact, we will emulate the SPI flash if `if=mtd` was given. To know this, we will need to use the
     * Global API's function `driver_get`. */
    BlockBackend* blk = NULL;
    DriveInfo *dinfo = drive_get(IF_MTD, 0, 0);
    if (dinfo) {
        /* MTD was given! We need to initialize and emulate SPI flash */
        qemu_log("Adding SPI flash device\n");
        blk = blk_by_legacy_dinfo(dinfo);
    } else {
        qemu_log("Not initializing SPI Flash\n");
    }

    /* Re-use the macro that checks and casts any generic/parent class to the real child instance */
    Esp32C3MachineState *ms = ESP32C3_MACHINE(machine);
    global_s = ms;

    /* Initialize SoC */
    object_initialize_child(OBJECT(ms), "soc", &ms->soc, TYPE_ESP_RISCV_CPU);
    qdev_prop_set_uint64(DEVICE(&ms->soc), "resetvec", ESP32C3_RESET_ADDRESS);

    /* Initialize the memory mapping */
    const struct MemmapEntry *memmap = esp32c3_memmap;
    MemoryRegion *sys_mem = get_system_memory();

    /* Initialize the IROM */
    MemoryRegion *irom = g_new(MemoryRegion, 1);
    memory_region_init_rom(irom, NULL, "esp32c3.irom", memmap[ESP32C3_MEMREGION_IROM].size, &error_fatal);
    memory_region_add_subregion(sys_mem, memmap[ESP32C3_MEMREGION_IROM].base, irom);

    /* Initialize the DROM as an alias to IROM. */
    MemoryRegion *drom = g_new(MemoryRegion, 1);
    const hwaddr offset_in_orig = 0x40000;
    memory_region_init_alias(drom, NULL, "esp32c3.drom", irom, offset_in_orig, memmap[ESP32C3_MEMREGION_DROM].size);
    memory_region_add_subregion(sys_mem, memmap[ESP32C3_MEMREGION_DROM].base, drom);

    /* Initialize the IRAM */
    MemoryRegion *iram = g_new(MemoryRegion, 1);
    memory_region_init_ram(iram, NULL, "esp32c3.iram", memmap[ESP32C3_MEMREGION_IRAM].size, &error_fatal);
    memory_region_add_subregion(sys_mem, memmap[ESP32C3_MEMREGION_IRAM].base, iram);

    /* Initialize DRAM as an alias to IRAM (not including Internal SRAM 0) */
    MemoryRegion *dram = g_new(MemoryRegion, 1);
    /* DRAM mirrors IRAM for SRAM 1, skip the SRAM 0 area */
    memory_region_init_alias(dram, NULL, "esp32c3.dram", iram,
                             ESP32C3_INTERNAL_SRAM0_SIZE, memmap[ESP32C3_MEMREGION_DRAM].size);
    memory_region_add_subregion(sys_mem, memmap[ESP32C3_MEMREGION_DRAM].base, dram);

    /* Initialize RTC Fast Memory as regular RAM */
    MemoryRegion *rtcram = g_new(MemoryRegion, 1);
    memory_region_init_ram(rtcram, NULL, "esp32c3.rtcram", memmap[ESP32C3_MEMREGION_RTCFAST].size, &error_fatal);
    memory_region_add_subregion(sys_mem, memmap[ESP32C3_MEMREGION_RTCFAST].base, rtcram);

    qdev_realize(DEVICE(&ms->soc), NULL, &error_fatal);

    memory_region_init_io(&ms->iomem, OBJECT(&ms->soc), &esp32c3_io_ops,
                          NULL, "esp32c3.iomem", 0xd1000);
    memory_region_add_subregion(sys_mem, ESP32C3_IO_START_ADDR, &ms->iomem);


    /* Initialize the peripheral bus */
    qbus_init(&ms->periph_bus, sizeof(ms->periph_bus),
              TYPE_SYSTEM_BUS, DEVICE(&ms->soc), "esp32c3-periph-bus");

    /* Initialize the main I/O of the CPU that waits for "reset" requests */
    qdev_init_gpio_in_named(DEVICE(&ms->soc), esp32c3_reset_request, ESP32C3_RESET_GPIO_NAME, 1);

    /* Initialize the I/O peripherals */
    for (int i = 0; i < ESP32C3_UART_COUNT; ++i) {
        char name[16];
        snprintf(name, sizeof(name), "uart%d", i);
        object_initialize_child(OBJECT(machine), name, &ms->uart[i], TYPE_ESP32C3_UART);

        snprintf(name, sizeof(name), "serial%d", i);
        object_property_add_alias(OBJECT(machine), name, OBJECT(&ms->uart[i]), "chardev");
        qdev_prop_set_chr(DEVICE(&ms->uart[i]), "chardev", serial_hd(i));
    }

    object_initialize_child(OBJECT(machine), "intmatrix", &ms->intmatrix, TYPE_ESP32C3_INTMATRIX);
    object_initialize_child(OBJECT(machine), "gpio", &ms->gpio, TYPE_ESP32C3_GPIO);
    object_initialize_child(OBJECT(machine), "extmem", &ms->cache, TYPE_ESP32C3_CACHE);
    object_initialize_child(OBJECT(machine), "efuse", &ms->efuse, TYPE_ESP32C3_EFUSE);
    object_initialize_child(OBJECT(machine), "clock", &ms->clock, TYPE_ESP32C3_CLOCK);
    object_initialize_child(OBJECT(machine), "sha", &ms->sha, TYPE_ESP32C3_SHA);
    object_initialize_child(OBJECT(machine), "aes", &ms->aes, TYPE_ESP32C3_AES);
    object_initialize_child(OBJECT(machine), "gdma", &ms->gdma, TYPE_ESP32C3_GDMA);
    object_initialize_child(OBJECT(machine), "rsa", &ms->rsa, TYPE_ESP32C3_RSA);
    object_initialize_child(OBJECT(machine), "hmac", &ms->hmac, TYPE_ESP32C3_HMAC);
    object_initialize_child(OBJECT(machine), "ds", &ms->ds, TYPE_ESP32C3_DS);
    object_initialize_child(OBJECT(machine), "timg0", &ms->timg[0], TYPE_ESP32C3_TIMG);
    object_initialize_child(OBJECT(machine), "timg1", &ms->timg[1], TYPE_ESP32C3_TIMG);
    object_initialize_child(OBJECT(machine), "systimer", &ms->systimer, TYPE_ESP32C3_SYSTIMER);
    object_initialize_child(OBJECT(machine), "spi1", &ms->spi1, TYPE_ESP32C3_SPI);
    object_initialize_child(OBJECT(machine), "rtccntl", &ms->rtccntl, TYPE_ESP32C3_RTC_CNTL);
    object_initialize_child(OBJECT(machine), "jtag", &ms->jtag, TYPE_ESP32C3_JTAG);
    object_initialize_child(OBJECT(machine), "rgb", &ms->rgb, TYPE_ESP_RGB);
    object_initialize_child(OBJECT(machine), "iomux", &ms->iomux, TYPE_ESP32C3_IOMUX);
    object_initialize_child(OBJECT(machine), "saradc", &ms->saradc, TYPE_ESP32C3_SARADC);
    object_initialize_child(OBJECT(machine), "ledc", &ms->ledc, TYPE_ESP32C3_LEDC);
    object_initialize_child(OBJECT(machine), "phya", &ms->phya, TYPE_ESP32_PHYA);
    object_initialize_child(OBJECT(machine), "ana", &ms->ana, TYPE_ESP32C3_ANA);
    object_initialize_child(OBJECT(machine), "fe", &ms->fe, TYPE_ESP32_FE);
    object_initialize_child(OBJECT(machine), "pwrmng", &ms->pwrmng, TYPE_ESP32C3_PWR_MANAGER);

    for(int i=0;i<nb_nics;i++)
        if (nd_table[i].used && nd_table[i].model && strcmp(nd_table[i].model, TYPE_ESP32C3_WIFI) == 0){
            object_initialize_child(OBJECT(machine), "wifi", &ms->wifi, TYPE_ESP32_WIFI);
        }

    /* Realize all the I/O peripherals we depend on */

    /* Interrupt matrix realization */
    DeviceState* intmatrix_dev = DEVICE(&ms->intmatrix);
    {
        /* Store the current Machine CPU in the interrupt matrix */
        object_property_set_link(OBJECT(&ms->intmatrix), "cpu", OBJECT(&ms->soc), &error_abort);
        sysbus_realize(SYS_BUS_DEVICE(&ms->intmatrix), &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ms->intmatrix), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_INTERRUPT_BASE, mr, 0);

        /* Connect all the interrupt matrix 31 output lines to the CPU 31 input IRQ lines.
         * The lines are indexed starting at 1.
         */
        for (int i = 0; i <= ESP32C3_CPU_INT_COUNT; i++) {
            qemu_irq cpu_input = qdev_get_gpio_in_named(DEVICE(&ms->soc), ESP_CPU_IRQ_LINES_NAME, i);
            qdev_connect_gpio_out_named(intmatrix_dev, ESP32C3_INT_MATRIX_OUTPUT_NAME, i, cpu_input);
        }
    }

    /* eFuses realization */
    {
        sysbus_realize(SYS_BUS_DEVICE(&ms->efuse), &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ms->efuse), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_EFUSE_BASE, mr, 0);
        sysbus_connect_irq(SYS_BUS_DEVICE(&ms->efuse), 0,
                       qdev_get_gpio_in(intmatrix_dev, ETS_EFUSE_INTR_SOURCE));
    }

    /* Initialize OpenCores Ethernet controller now sicne it requires the interrupt matrix */
    esp32c3_init_openeth(ms);

    /* USB Serial JTAG realization */
    {
        sysbus_realize(SYS_BUS_DEVICE(&ms->jtag), &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ms->jtag), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_USB_SERIAL_JTAG_BASE, mr, 0);
    }

    /* IOMUX realization */
    {
        sysbus_realize(SYS_BUS_DEVICE(&ms->iomux), &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ms->iomux), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_IO_MUX_BASE, mr, 0);
    }

    /* RTC CNTL realization */
    {
        sysbus_realize(SYS_BUS_DEVICE(&ms->rtccntl), &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ms->rtccntl), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_RTCCNTL_BASE, mr, 0);
        /* Connect CNTL's reset-request GPIO to the SoC's reset GPIO */
        qdev_connect_gpio_out_named(DEVICE(&ms->rtccntl), ESP32C3_RTC_CPU_RESET_GPIO, 0,
                                    qdev_get_gpio_in_named(DEVICE(&ms->soc), ESP32C3_RESET_GPIO_NAME, 0));
    }

    /* SPI1 controller (SPI Flash) */
    {
        sysbus_realize(SYS_BUS_DEVICE(&ms->spi1), &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ms->spi1), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_SPI1_BASE, mr, 0);
        if (blk) {
            esp32c3_init_spi_flash(ms, blk);
        }
    }

    for (int i = 0; i < ESP32C3_UART_COUNT; ++i) {
        const hwaddr uart_base[] = { DR_REG_UART_BASE, DR_REG_UART1_BASE };
        sysbus_realize(SYS_BUS_DEVICE(&ms->uart[i]), &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ms->uart[i]), 0);
        memory_region_add_subregion_overlap(sys_mem, uart_base[i], mr, 0);
        sysbus_connect_irq(SYS_BUS_DEVICE(&ms->uart[i]), 0,
                           qdev_get_gpio_in(intmatrix_dev, ETS_UART0_INTR_SOURCE + i));
    }

    /* GPIO realization */
    {
        sysbus_realize(SYS_BUS_DEVICE(&ms->gpio), &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ms->gpio), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_GPIO_BASE, mr, 0);
        sysbus_connect_irq(SYS_BUS_DEVICE(&ms->gpio), 0,
                           qdev_get_gpio_in(intmatrix_dev, ETS_GPIO_INTR_SOURCE));
    }

    /* (Extmem) Cache realization */
    {
        if (blk) {
            ms->cache.flash_blk = blk;
        }
        sysbus_realize(SYS_BUS_DEVICE(&ms->cache), &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ms->cache), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_EXTMEM_BASE, mr, 0);

        memory_region_add_subregion_overlap(sys_mem, ms->cache.dcache_base, &ms->cache.dcache, 0);
        memory_region_add_subregion_overlap(sys_mem, ms->cache.icache_base, &ms->cache.icache, 0);
    }

    /* System clock realization */
    {
        sysbus_realize(SYS_BUS_DEVICE(&ms->clock), &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ms->clock), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_SYSTEM_BASE, mr, 0);
        /* Connect the IRQ lines to the interrupt matrix */
        for (int i = 0; i < ESP32C3_SYSTEM_CPU_INTR_COUNT; i++) {
            sysbus_connect_irq(SYS_BUS_DEVICE(&ms->clock), i,
                           qdev_get_gpio_in(intmatrix_dev, ETS_FROM_CPU_INTR0_SOURCE + i));
        }
    }

    /* Timer Groups realization */
    {
        sysbus_realize(SYS_BUS_DEVICE(&ms->timg[0]), &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ms->timg[0]), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_TIMERGROUP0_BASE, mr, 0);
        /* Connect the T0 interrupt line to the interrupt matrix */
        qdev_connect_gpio_out_named(DEVICE(&ms->timg[0]), ESP32C3_T0_IRQ_INTERRUPT, 0,
                                    qdev_get_gpio_in(intmatrix_dev, ETS_TG0_T0_LEVEL_INTR_SOURCE));
        /* Connect the Watchdog interrupt line to the interrupt matrix */
        qdev_connect_gpio_out_named(DEVICE(&ms->timg[0]), ESP32C3_WDT_IRQ_INTERRUPT, 0,
                                    qdev_get_gpio_in(intmatrix_dev, ETS_TG0_WDT_LEVEL_INTR_SOURCE));
        /* Connect the Watchdog reset request to the CNTL's WDT0 line */
        qdev_connect_gpio_out_named(DEVICE(&ms->timg[0]), ESP32C3_WDT_IRQ_RESET, 0,
                                    qdev_get_gpio_in(DEVICE(&ms->rtccntl), ESP32C3_TG0WDT_SYS_RESET));

    }
    {
        sysbus_realize(SYS_BUS_DEVICE(&ms->timg[1]), &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ms->timg[1]), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_TIMERGROUP1_BASE, mr, 0);
        /* Connect the T0 interrupt line to the interrupt matrix */
        qdev_connect_gpio_out_named(DEVICE(&ms->timg[1]), ESP32C3_T0_IRQ_INTERRUPT, 0,
                                    qdev_get_gpio_in(intmatrix_dev, ETS_TG1_T0_LEVEL_INTR_SOURCE));
        qdev_connect_gpio_out_named(DEVICE(&ms->timg[1]), ESP32C3_WDT_IRQ_INTERRUPT, 0,
                                    qdev_get_gpio_in(intmatrix_dev, ETS_TG1_WDT_LEVEL_INTR_SOURCE));
        qdev_connect_gpio_out_named(DEVICE(&ms->timg[1]), ESP32C3_WDT_IRQ_RESET, 0,
                                    qdev_get_gpio_in(DEVICE(&ms->rtccntl), ESP32C3_TG1WDT_SYS_RESET));
    }

    /* System timer */
    {
        sysbus_realize(SYS_BUS_DEVICE(&ms->systimer), &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ms->systimer), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_SYSTIMER_BASE, mr, 0);
        for (int i = 0; i < ESP32C3_SYSTIMER_IRQ_COUNT; i++) {
            sysbus_connect_irq(SYS_BUS_DEVICE(&ms->systimer), i,
                           qdev_get_gpio_in(intmatrix_dev, ETS_SYSTIMER_TARGET0_EDGE_INTR_SOURCE + i));
        }
    }

    /* GDMA Realization */
    {
        object_property_set_link(OBJECT(&ms->gdma), "soc_mr", OBJECT(dram), &error_abort);
        sysbus_realize(SYS_BUS_DEVICE(&ms->gdma), &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ms->gdma), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_GDMA_BASE, mr, 0);
        /* Connect the IRQs to the Interrupt Matrix */
        for (int i = 0; i < ESP32C3_GDMA_CHANNEL_COUNT; i++) {
            sysbus_connect_irq(SYS_BUS_DEVICE(&ms->gdma), i,
                               qdev_get_gpio_in(intmatrix_dev, ETS_DMA_CH0_INTR_SOURCE + i));
        }

    }

    /* SHA realization */
    {
        ms->sha.gdma = &ms->gdma;
        sysbus_realize(SYS_BUS_DEVICE(&ms->sha), &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ms->sha), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_SHA_BASE, mr, 0);
        sysbus_connect_irq(SYS_BUS_DEVICE(&ms->sha), 0,
                           qdev_get_gpio_in(intmatrix_dev, ETS_SHA_INTR_SOURCE));
    }

    /* AES realization */
    {
        ms->aes.gdma = &ms->gdma;
        sysbus_realize(SYS_BUS_DEVICE(&ms->aes), &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ms->aes), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_AES_BASE, mr, 0);
        sysbus_connect_irq(SYS_BUS_DEVICE(&ms->aes), 0,
                           qdev_get_gpio_in(intmatrix_dev, ETS_AES_INTR_SOURCE));
    }

    /* RSA realization */
    {
        sysbus_realize(SYS_BUS_DEVICE(&ms->rsa), &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ms->rsa), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_RSA_BASE, mr, 0);
        sysbus_connect_irq(SYS_BUS_DEVICE(&ms->rsa), 0,
                           qdev_get_gpio_in(intmatrix_dev, ETS_RSA_INTR_SOURCE));
    }

    /* HMAC realization */
    {
        ms->hmac.efuse = &ms->efuse;
        qdev_realize(DEVICE(&ms->hmac), &ms->periph_bus, &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ms->hmac), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_HMAC_BASE, mr, 0);
    }

    /* Digital Signature realization */
    {
        ms->ds.hmac = &ms->hmac;
        ms->ds.aes = &ms->aes;
        ms->ds.rsa = &ms->rsa;
        ms->ds.sha = &ms->sha;
        qdev_realize(DEVICE(&ms->ds), &ms->periph_bus, &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ms->ds), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_DIGITAL_SIGNATURE_BASE, mr, 0);
    }

    /* RGB display realization */
    {
        /* Give the internal RAM memory region to the display */
        ms->rgb.intram = dram;
        sysbus_realize(SYS_BUS_DEVICE(&ms->rgb), &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ms->rgb), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_FRAMEBUF_BASE, mr, 0);
        memory_region_add_subregion_overlap(sys_mem, esp32c3_memmap[ESP32C3_MEMREGION_FRAMEBUF].base, &ms->rgb.vram, 0);
    }

    /* SARADC realization */
    {
        qdev_realize(DEVICE(&ms->saradc), &ms->periph_bus, &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ms->saradc), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_APB_SARADC_BASE, mr, 0);
    }

    /* LEDC realization */
    {
        sysbus_realize(SYS_BUS_DEVICE(&ms->ledc), &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ms->ledc), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_LEDC_BASE, mr, 0);
    }

    //'esp32c3_wifi.c',   DR_REG_WIFI_BASE

    /* ANA realization */
    {
        qdev_realize(DEVICE(&ms->ana), &ms->periph_bus, &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ms->ana), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_RTC_I2C_BASE, mr, 0);
    }

    /* PHYA realization */
    {
        qdev_realize(DEVICE(&ms->phya), &ms->periph_bus, &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ms->phya), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_PHYA_BASE, mr, 0);
    }  

    /* FE realization */
    {
        qdev_realize(DEVICE(&ms->fe), &ms->periph_bus, &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ms->fe), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_FE_BASE, mr, 0);
    }   

    /* PWR MANAGER realization */
    {
        qdev_realize(DEVICE(&ms->pwrmng), &ms->periph_bus, &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ms->pwrmng), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_PWR_MANAGER_BASE, mr, 0);
    }

    esp32c3_add_unimp_device(sys_mem, "esp32c3.sensitive", DR_REG_SENSITIVE_BASE, 0x1000,0);
    esp32c3_add_unimp_device(sys_mem, "esp32c3.mmu", DR_REG_MMU_TABLE, 0x1000,0);
    esp32c3_add_unimp_device(sys_mem, "esp32c3.dedicategpio", DR_REG_DEDICATED_GPIO_BASE, 0x1000,0);
    esp32c3_add_unimp_device(sys_mem, "esp32c3.worldcntl", DR_REG_WORLD_CNTL_BASE, 0x1000,0);

    esp32c3_add_unimp_device(sys_mem, "esp32c3.spi0", DR_REG_SPI0_BASE, 0x1000,0);
    esp32c3_add_unimp_device(sys_mem, "esp32c3.fe2", DR_REG_FE2_BASE, 0x1000,0);
    esp32c3_add_unimp_device(sys_mem, "esp32c3.i2c", DR_REG_I2C_EXT_BASE, 0x1000,0);
    esp32c3_add_unimp_device(sys_mem, "esp32c3.uhci0", DR_REG_UHCI0_BASE, 0x1000,0);
    esp32c3_add_unimp_device(sys_mem, "esp32c3.rmt", DR_REG_RMT_BASE, 0x1000,0);
    esp32c3_add_unimp_device(sys_mem, "esp32c3.ledc", DR_REG_LEDC_BASE, 0x1000,0);
    esp32c3_add_unimp_device(sys_mem, "esp32c3.spi2", DR_REG_SPI2_BASE, 0x1000,0);
    esp32c3_add_unimp_device(sys_mem, "esp32c3.twai", DR_REG_TWAI_BASE, 0x1000,0);
    esp32c3_add_unimp_device(sys_mem, "esp32c3.i2s0", DR_REG_I2S0_BASE, 0x1000,0);

    esp32c3_add_unimp_device(sys_mem, "esp32c3.nrx", DR_REG_NRX_BASE  - 0x0C00, 0x1000,-1);
    esp32c3_add_unimp_device(sys_mem, "esp32c3.bb", DR_REG_BB_BASE , 0x1000,-1);
    esp32c3_add_unimp_device(sys_mem, "esp32c3.aes_xts", DR_REG_AES_XTS_BASE, 0x1000,0);

    /* Register reset function so that it is called when `system_reset` is invoked in QEMU monitor */
    //qemu_register_reset(esp32c3_reset_request, ms);

    /* Open and load the "bios", which is the ROM binary, also named "first stage bootloader" */
    char *rom_binary = qemu_find_file(QEMU_FILE_TYPE_BIOS, "esp32c3-rom.bin");
    if (rom_binary == NULL) {
        error_report("Error: -bios argument not set, and ROM code binary not found (1)");
        exit(1);
    }

    /* Load ROM file at the reset address */
    int size = load_image_targphys_as(rom_binary, ESP32C3_RESET_ADDRESS, 0x60000, CPU(&ms->soc)->as);
    if (size < 0) {
        error_report("Error: could not load ROM binary '%s'", rom_binary);
        exit(1);
    }

    g_free(rom_binary);


    //PICSimLab gpio map

//ESP32-DevKitC V4
    psync_irq = qemu_allocate_irqs (psync_irq_handler, NULL, 1);
    qdev_connect_gpio_out_named(DEVICE(&ms->gpio), ESP32_GPIOS_SYNC, 0 , psync_irq[0]);
    qdev_connect_gpio_out_named(DEVICE(&ms->iomux), ESP32_IOMUX_SYNC, 0 , psync_irq[0]);
    qdev_connect_gpio_out_named(DEVICE(&ms->ledc), ESP32C3_LEDC_SYNC, 0 , psync_irq[0]);

/*
    spi_cs_irq = qemu_allocate_irqs (spi_cs_irq_handler, NULL, 8);

    qdev_connect_gpio_out_named(DEVICE(&ms->spi[2]), SSI_GPIO_CS, 0 , spi_cs_irq[0]);
    qdev_connect_gpio_out_named(DEVICE(&ms->spi[2]), SSI_GPIO_CS, 1 , spi_cs_irq[1]);
    qdev_connect_gpio_out_named(DEVICE(&ms->spi[2]), SSI_GPIO_CS, 2 , spi_cs_irq[2]);

    qdev_connect_gpio_out_named(DEVICE(&ms->spi[3]), SSI_GPIO_CS, 0 , spi_cs_irq[4]);
    qdev_connect_gpio_out_named(DEVICE(&ms->spi[3]), SSI_GPIO_CS, 1 , spi_cs_irq[5]);
    qdev_connect_gpio_out_named(DEVICE(&ms->spi[3]), SSI_GPIO_CS, 2 , spi_cs_irq[6]);
*/

    if(pinmap){
      pdir_irq = qemu_allocate_irqs (pdir_irq_handler, NULL, pinmap[0]+1 );
      pout_irq = qemu_allocate_irqs (pout_irq_handler, NULL, pinmap[0]+1);
      for(int pin = 1; pin < (pinmap[0]+1); pin++){
        if(pinmap[pin] >= 0){
            qdev_connect_gpio_out_named(DEVICE(&ms->gpio), ESP32_GPIOS, pinmap[pin], pout_irq[pin]);
            qdev_connect_gpio_out_named(DEVICE(&ms->gpio), ESP32_GPIOS_DIR, pinmap[pin] , pdir_irq[pin]);
            pin_irq[pin]=qdev_get_gpio_in_named(DEVICE(&ms->gpio), ESP32_GPIOS_IN, pinmap[pin]);
        }
      } 
    }
}


/* Initialize machine type */
static void esp32c3_machine_class_init(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);
    mc->desc = "Espressif ESP32-C3 machine";
    mc->default_cpu_type = TYPE_ESP_RISCV_CPU;
    mc->init = esp32c3_machine_init;
    mc->max_cpus = 1;
    mc->default_cpus = 1;
    // 0x4f600
    mc->default_ram_size = 400 * 1024;
}

/* Create a new type of machine ("child class") */
static const TypeInfo esp32c3_info = {
    .name = TYPE_ESP32C3_MACHINE,
    /* Specify the parent class, i.e. the class we derivate from */
    .parent = TYPE_MACHINE,
    /* Real size in bytes of our machine instance */
    .instance_size = sizeof(Esp32C3MachineState),
    /* Override the init function to one we defined above */
    .class_init = esp32c3_machine_class_init,
};

static void esp32c3_machine_type_init(void)
{
    type_register_static(&esp32c3_info);
}

type_init(esp32c3_machine_type_init);
