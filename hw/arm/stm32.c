/*
 * STM32 Microcontroller
 *
 * Copyright (C) 2010 Andre Beckus
 *
 * Implementation based on ST Microelectronics "RM0008 Reference Manual Rev 10"
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include "hw/arm/stm32.h"
#include "exec/address-spaces.h"
#include "exec/gdbstub.h"
#include "hw/arm/armv7m.h"
#include "hw/misc/unimp.h"
#include "qapi/error.h"
#include "sysemu/blockdev.h" // drive_get

#include "sysemu/cpu-timers.h"

/* DEFINITIONS */

typedef struct Stm32_Dwt Stm32_Dwt;

struct Stm32_Dwt {
  MemoryRegion dwt_mem;
  uint64_t last_cycle;
  uint32_t CTRL;
  uint32_t CYCCNT;
};

struct STM32F103State {
  /*< private >*/
  SysBusDevice parent_obj;

  /*< public >*/
  char *cpu_type;
  char *kernel_file;
  uint64_t /*ram_addr_t*/ flash_size;
  uint64_t /*ram_addr_t*/ ram_size;
  uint32_t osc_freq;
  uint32_t osc32_freq;

  ARMv7MState armv7m;

  MemoryRegion sram;
  MemoryRegion flash;
  MemoryRegion flash_alias;

  Stm32_Dwt DWT;

  Clock *sysclk;
  Clock *refclk;
};
/* COMMON */

void stm32_hw_warn(const char *fmt, ...) {
  va_list ap;

  va_start(ap, fmt);
  fprintf(stderr, "qemu stm32: hardware warning: ");
  vfprintf(stderr, fmt, ap);
  fprintf(stderr, "\n");
  cpu_dump_state(first_cpu, stderr, 0);
  va_end(ap);
}

/* PERIPHERALS */

const char *stm32_periph_name_arr[] = {"RCC",
                                       "GPIOA",
                                       "GPIOB",
                                       "GPIOC",
                                       "GPIOD",
                                       "GPIOE",
                                       "GPIOF",
                                       "GPIOG",
                                       "GPIOH",
                                       "GPIOI",
                                       "GPIOJ",
                                       "GPIOK",
                                       "SYSCFG",
                                       "AFIO",
                                       "UART1",
                                       "UART2",
                                       "UART3",
                                       "UART4",
                                       "UART5",
                                       "UART6",
                                       "UART7",
                                       "UART8",
                                       "ADC1",
                                       "ADC2",
                                       "ADC3",
                                       "DAC",
                                       "TIM1",
                                       "TIM2",
                                       "TIM3",
                                       "TIM4",
                                       "TIM5",
                                       "TIM6",
                                       "TIM7",
                                       "TIM8",
                                       "TIM9",
                                       "TIM10",
                                       "TIM11",
                                       "TIM12",
                                       "TIM13",
                                       "TIM14",
                                       "BKP",
                                       "PWR",
                                       "I2C1",
                                       "I2C2",
                                       "I2C3",
                                       "I2S1",
                                       "I2S2",
                                       "WWDG",
                                       "IWDG"
                                       "CAN1",
                                       "CAN2",
                                       "CAN",
                                       "USB",
                                       "SPI1",
                                       "SPI2",
                                       "SPI3",
                                       "EXTI",
                                       "SDIO",
                                       "FSMC",
                                       "RTC",
                                       "STM32_COMP",
                                       "STM_LCD",
                                       "STM32_CRC",
                                       "STM32_DMA1",
                                       "STM32_DMA2",
                                       "STM32_DCMI_PERIPH",
                                       "STM32_CRYP_PERIPH",
                                       "STM32_HASH_PERIPH",
                                       "STM32_RNG_PERIPH",
                                       "STM32_FLASH",
                                       "STM32_FLASH_REGS"};

const char *stm32_periph_name(stm32_periph_t periph) {
  assert(periph < STM32_PERIPH_COUNT);

  return stm32_periph_name_arr[periph];
}

/* INITIALIZATION */

/* I copied sysbus_create_varargs and split it into two parts.  This is so that
 * you can set properties before calling the device init function.
 */

static DeviceState *stm32_init_periph(DeviceState *dev, stm32_periph_t periph,
                                      hwaddr addr, qemu_irq irq) {
  // qdev_init_nofail(dev);
  sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, addr);
  if (irq) {
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, irq);
  }
  return dev;
}

static void stm32_create_uart_dev(Object *stm32_container,
                                  stm32_periph_t periph, int uart_num,
                                  DeviceState *rcc_dev, DeviceState **gpio_dev,
                                  DeviceState *afio_dev, hwaddr addr,
                                  qemu_irq irq, Error **errp) {
  char child_name[8];
  DeviceState *uart_dev = qdev_new("stm32-uart");
  QDEV_PROP_SET_PERIPH_T(uart_dev, "periph", periph);
  stm32_uart_set_rcc(STM32_UART(uart_dev), STM32_RCC(rcc_dev));
  stm32_uart_set_gpio(STM32_UART(uart_dev), (Stm32Gpio **)(gpio_dev));
  stm32_uart_set_afio(STM32_UART(uart_dev), STM32_AFIO(afio_dev));
  stm32_uart_set_id(STM32_UART(uart_dev),uart_num);
  snprintf(child_name, sizeof(child_name), "uart[%i]", uart_num);
  object_property_add_child(stm32_container, child_name, OBJECT(uart_dev));
  if (!sysbus_realize(SYS_BUS_DEVICE(uart_dev), errp)) {
    return;
  }
  stm32_init_periph(uart_dev, periph, addr, irq);
}

static void stm32_create_timer_dev(Object *stm32_container,
                                   stm32_periph_t periph, int timer_num,
                                   DeviceState *rcc_dev, DeviceState **gpio_dev,
                                   DeviceState *afio_dev, hwaddr addr,
                                   qemu_irq irq, Error **errp) {
  char child_name[9];
  DeviceState *timer_dev = qdev_new("stm32-timer");
  QDEV_PROP_SET_PERIPH_T(timer_dev, "periph", periph);

  stm32_timer_set_rcc(STM32_TIMER(timer_dev), STM32_RCC(rcc_dev));
  stm32_timer_set_gpio(STM32_TIMER(timer_dev), (Stm32Gpio **)gpio_dev);
  stm32_timer_set_afio(STM32_TIMER(timer_dev), STM32_AFIO(afio_dev));
  snprintf(child_name, sizeof(child_name), "timer[%i]", timer_num);
  object_property_add_child(stm32_container, child_name, OBJECT(timer_dev));
  if (!sysbus_realize(SYS_BUS_DEVICE(timer_dev), errp)) {
    return;
  }
  stm32_init_periph(timer_dev, periph, addr, NULL);
  sysbus_connect_irq(SYS_BUS_DEVICE(timer_dev), 0, irq);
}

static void stm32_create_adc_dev(Object *stm32_container, stm32_periph_t periph,
                                 int adc_num, DeviceState *rcc_dev,
                                 DeviceState **gpio_dev, hwaddr addr,
                                 qemu_irq irq, Error **errp) {
  char child_name[8];
  DeviceState *adc_dev = qdev_new("stm32-adc");
  QDEV_PROP_SET_PERIPH_T(adc_dev, "periph", periph);
  // qdev_prop_set_ptr(adc_dev, "stm32_rcc", rcc_dev);      // jmf : pourquoi ?
  stm32_adc_set_rcc(STM32_ADC(adc_dev), STM32_RCC(rcc_dev));
  // object_property_set_link(OBJECT(adc_dev), "stm32_rcc", OBJECT(rcc_dev),
  // NULL); qdev_prop_set_ptr(adc_dev, "stm32_gpio", gpio_dev);
  // object_property_set_link(OBJECT(adc_dev), "stm32_gpio", OBJECT(gpio_dev),
  // NULL);
  stm32_adc_set_gpio(STM32_ADC(adc_dev), (Stm32Gpio **)gpio_dev);
  snprintf(child_name, sizeof(child_name), "adc[%i]", adc_num);
  object_property_add_child(stm32_container, child_name, OBJECT(adc_dev));
  if (!sysbus_realize(SYS_BUS_DEVICE(adc_dev), errp)) {
    return;
  }
  stm32_init_periph(adc_dev, periph, addr, irq);
}

static void stm32_create_rtc_dev(Object *stm32_container, stm32_periph_t periph,
                                 int rtc_num, DeviceState *rcc_dev, hwaddr addr,
                                 qemu_irq irq, Error **errp) {
  char child_name[8];
  DeviceState *rtc_dev = qdev_new("stm32-rtc");
  QDEV_PROP_SET_PERIPH_T(rtc_dev, "periph", periph);
  // qdev_prop_set_ptr(rtc_dev, "stm32_rcc", rcc_dev);      // jmf : pourquoi ?
  stm32_rtc_set_rcc(STM32_RTC(rtc_dev), STM32_RCC(rcc_dev));
  snprintf(child_name, sizeof(child_name), "rtc[%i]", rtc_num);
  object_property_add_child(stm32_container, child_name, OBJECT(rtc_dev));
  if (!sysbus_realize(SYS_BUS_DEVICE(rtc_dev), errp)) {
    return;
  }
  stm32_init_periph(rtc_dev, periph, addr, irq);
}

static void stm32_create_dac_dev(Object *stm32_container, stm32_periph_t periph,
                                 DeviceState *rcc_dev, DeviceState **gpio_dev,
                                 hwaddr addr, qemu_irq irq, Error **errp) {
  DeviceState *dac_dev = qdev_new("stm32-dac");
  QDEV_PROP_SET_PERIPH_T(dac_dev, "periph", periph);
  // qdev_prop_set_ptr(dac_dev, "stm32_rcc", rcc_dev);
  stm32_dac_set_rcc(STM32_DAC(dac_dev), STM32_RCC(rcc_dev));
  // qdev_prop_set_ptr(dac_dev, "stm32_gpio", gpio_dev);
  stm32_dac_set_gpio(STM32_DAC(dac_dev), (Stm32Gpio **)gpio_dev);
  object_property_add_child(stm32_container, "dac", OBJECT(dac_dev));
  if (!sysbus_realize(SYS_BUS_DEVICE(dac_dev), errp)) {
    return;
  }
  stm32_init_periph(dac_dev, periph, addr, irq);
}
/*
static uint64_t kernel_load_translate_fn(void *opaque, uint64_t from_addr) {
    if (from_addr == STM32_FLASH_ADDR_START) {
        return 0x00000000;
    }
    return from_addr;
}
*/
static void stm32f103_soc_initfn(Object *obj) {
  STM32F103State *s = STM32F103_SOC(obj);

  object_initialize_child(obj, "armv7m", &s->armv7m, TYPE_ARMV7M);

  s->sysclk = qdev_init_clock_in(DEVICE(s), "sysclk", NULL, NULL, 0);
  s->refclk = qdev_init_clock_in(DEVICE(s), "refclk", NULL, NULL, 0);
}

void stm32_init(ram_addr_t flash_size, ram_addr_t ram_size,
                const char *kernel_filename, uint32_t osc_freq,
                uint32_t osc32_freq, Clock *sysclk) {
  DeviceState *dev;

  dev = qdev_new(TYPE_STM32F103_SOC);
  qdev_prop_set_string(dev, "cpu-type", ARM_CPU_TYPE_NAME("cortex-m3"));
  if (kernel_filename)
    qdev_prop_set_string(dev, "kernel-file", kernel_filename);
  qdev_prop_set_uint64(dev, "flash_size", flash_size);
  qdev_prop_set_uint64(dev, "ram_size", ram_size);
  qdev_prop_set_uint32(dev, "osc_freq", osc_freq);
  qdev_prop_set_uint32(dev, "osc32_freq", osc32_freq);
  qdev_connect_clock_in(dev, "sysclk", sysclk);
  sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
}

static MemTxResult dwt_write(void *opaque, hwaddr addr, uint64_t value,
                             unsigned size, MemTxAttrs attrs) {
  Stm32_Dwt *s = opaque;

  if (attrs.user) {
    return MEMTX_ERROR;
  }

  if (addr == 0) {
    s->CTRL = value;
  } else if (addr == 4) {
    s->CYCCNT = value;
    s->last_cycle = cpu_get_ticks();
  }
  return MEMTX_OK;
}

static MemTxResult dwt_read(void *opaque, hwaddr addr, uint64_t *data,
                            unsigned size, MemTxAttrs attrs) {
  Stm32_Dwt *s = opaque;

  if (attrs.user) {
    return MEMTX_ERROR;
  }

  *data = 0;

  if (addr == 0) {
    *data = s->CTRL;
  } else if (addr == 4) {
    if (s->CTRL & 0x0001) {
      uint64_t ticks = cpu_get_ticks();
      s->CYCCNT += ticks - s->last_cycle;
      s->last_cycle = ticks;
    }
    *data = s->CYCCNT;
  }
  return MEMTX_OK;
}

static const MemoryRegionOps dwt_ops = {
    .read_with_attrs = dwt_read,
    .write_with_attrs = dwt_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void stm32f103_soc_realize(DeviceState *dev_soc, Error **errp) {
  STM32F103State *s = STM32F103_SOC(dev_soc);
  DeviceState *armv7m;
  MemoryRegion *address_space_mem = get_system_memory();
  //    qemu_irq *pic;
  DriveInfo *dinfo;
  int i;

  if (clock_has_source(s->refclk)) {
    error_setg(errp, "refclk clock must not be wired up by the board code");
    return;
  }

  if (!clock_has_source(s->sysclk)) {
    error_setg(errp, "sysclk clock must be wired up by the board code");
    return;
  }

  /* The refclk always runs at frequency HCLK / 8 */
  clock_set_mul_div(s->refclk, 8, 1);
  clock_set_source(s->refclk, s->sysclk);

  Object *stm32_container = container_get(qdev_get_machine(), "/stm32");

  MemoryRegion *sram = g_new(MemoryRegion, 1);
  // MemoryRegion *flash = g_new(MemoryRegion, 1);

  /*
      pic = armv7m_translated_init(
                stm32_container,
                address_space_mem,
                flash_size,
                ram_size,
                kernel_filename,
                kernel_load_translate_fn,
                NULL,
                "cortex-m3");*/
  /* Flash programming is done via the SCU, so pretend it is ROM.  */
  // memory_region_init_rom(flash, NULL, "stm32.flash", s->flash_size,
  // &error_fatal); memory_region_add_subregion(address_space_mem, 0, flash);

  memory_region_init_ram(sram, NULL, "stm32.sram", s->ram_size, &error_fatal);
  memory_region_add_subregion(address_space_mem, 0x20000000, sram);

  if (s->kernel_file) // Use legacy mode without reset support
  {
    MemoryRegion *flash_alias_mem = g_malloc(sizeof(MemoryRegion));
    /* The STM32 family stores its Flash memory at some base address in memory
     * (0x08000000 for medium density devices), and then aliases it to the
     * boot memory space, which starts at 0x00000000 (the "System Memory" can
     * also be aliased to 0x00000000, but this is not implemented here). The
     * processor executes the code in the aliased memory at 0x00000000.  We need
     * to make a QEMU alias so that reads in the 0x08000000 area are passed
     * through to the 0x00000000 area. Note that this is the opposite of real
     * hardware, where the memory at 0x00000000 passes reads through the "real"
     * flash memory at 0x08000000, but it works the same either way. */
    /* TODO: Parameterize the base address of the aliased memory. */
    memory_region_init_alias(flash_alias_mem, NULL, "stm32-flash-alias-mem",
                             address_space_mem, 0, s->flash_size);
    memory_region_add_subregion(address_space_mem, STM32_FLASH_ADDR_START,
                                flash_alias_mem);
  } else { // Use new FLASH mode with reset support
    dinfo = drive_get(IF_PFLASH, 0, 0);
    if (dinfo) {
      stm32_flash_register(blk_by_legacy_dinfo(dinfo), STM32_FLASH_ADDR_START,
                           s->flash_size);
    }
    MemoryRegionSection mrs = memory_region_find(
        address_space_mem, STM32_FLASH_ADDR_START, 4 /*WORD_ACCESS_SIZE*/);
    MemoryRegion *flash_alias_mem = g_new(MemoryRegion, 1);
    memory_region_init_alias(flash_alias_mem, NULL, "stm32-flash-alias-mem",
                             mrs.mr, 0, s->flash_size);
    memory_region_add_subregion(address_space_mem, 0, flash_alias_mem);
  }

  /* Init ARMv7m */
  armv7m = DEVICE(&s->armv7m);
  qdev_prop_set_uint32(armv7m, "num-irq", 61);
  qdev_prop_set_string(armv7m, "cpu-type", s->cpu_type);
  qdev_prop_set_bit(armv7m, "enable-bitband", true);
  qdev_connect_clock_in(armv7m, "cpuclk", s->sysclk);
  qdev_connect_clock_in(armv7m, "refclk", s->refclk);
  object_property_set_link(OBJECT(&s->armv7m), "memory",
                           OBJECT(get_system_memory()), &error_abort);
  if (!sysbus_realize(SYS_BUS_DEVICE(&s->armv7m), errp)) {
    return;
  }

  DeviceState *rcc_dev = qdev_new("stm32-rcc");
  qdev_prop_set_uint32(rcc_dev, "osc_freq", s->osc_freq);
  qdev_prop_set_uint32(rcc_dev, "osc32_freq", s->osc32_freq);
  stm32_rcc_set_sysclk(STM32_RCC(rcc_dev), s->sysclk);
  object_property_add_child(stm32_container, "rcc", OBJECT(rcc_dev));
  if (!sysbus_realize(SYS_BUS_DEVICE(rcc_dev), errp)) {
    return;
  }
  stm32_init_periph(rcc_dev, STM32_RCC_PERIPH, 0x40021000,
                    qdev_get_gpio_in(armv7m, STM32_RCC_IRQ));

  DeviceState **gpio_dev =
      (DeviceState **)g_malloc0(sizeof(DeviceState *) * STM32_GPIO_COUNT);
  for (i = 0; i < STM32_GPIO_COUNT; i++) {
    char child_name[8];
    stm32_periph_t periph = STM32_GPIOA + i;
    gpio_dev[i] = qdev_new(TYPE_STM32_GPIO);
    QDEV_PROP_SET_PERIPH_T(gpio_dev[i], "periph", periph);
    // qdev_prop_set_ptr(gpio_dev[i], "stm32_rcc", rcc_dev);
    stm32_gpio_set_rcc(STM32_GPIO(gpio_dev[i]), STM32_RCC(rcc_dev));
    snprintf(child_name, sizeof(child_name), "gpio[%c]", 'a' + i);
    object_property_add_child(stm32_container, child_name, OBJECT(gpio_dev[i]));
    if (!sysbus_realize(SYS_BUS_DEVICE(gpio_dev[i]), errp)) {
      return;
    }
    stm32_init_periph(gpio_dev[i], periph, 0x40010800 + (i * 0x400), NULL);
  }

  DeviceState *exti_dev = qdev_new(TYPE_STM32_EXTI);
  object_property_add_child(stm32_container, "exti", OBJECT(exti_dev));
  if (!sysbus_realize(SYS_BUS_DEVICE(exti_dev), errp)) {
    return;
  }
  stm32_init_periph(exti_dev, STM32_EXTI_PERIPH, 0x40010400, NULL);
  SysBusDevice *exti_busdev = SYS_BUS_DEVICE(exti_dev);
  sysbus_connect_irq(exti_busdev, 0, qdev_get_gpio_in(armv7m, STM32_EXTI0_IRQ));
  sysbus_connect_irq(exti_busdev, 1, qdev_get_gpio_in(armv7m, STM32_EXTI1_IRQ));
  sysbus_connect_irq(exti_busdev, 2, qdev_get_gpio_in(armv7m, STM32_EXTI2_IRQ));
  sysbus_connect_irq(exti_busdev, 3, qdev_get_gpio_in(armv7m, STM32_EXTI3_IRQ));
  sysbus_connect_irq(exti_busdev, 4, qdev_get_gpio_in(armv7m, STM32_EXTI4_IRQ));
  sysbus_connect_irq(exti_busdev, 5,
                     qdev_get_gpio_in(armv7m, STM32_EXTI9_5_IRQ));
  sysbus_connect_irq(exti_busdev, 6,
                     qdev_get_gpio_in(armv7m, STM32_EXTI15_10_IRQ));
  sysbus_connect_irq(exti_busdev, 7, qdev_get_gpio_in(armv7m, STM32_PVD_IRQ));
  // sysbus_connect_irq(exti_busdev, 8,
  // qdev_get_gpio_in(armv7m,STM32_RTCAlarm_IRQ));
  sysbus_connect_irq(exti_busdev, 9,
                     qdev_get_gpio_in(armv7m, STM32_OTG_FS_WKUP_IRQ));

  DeviceState *afio_dev = qdev_new(TYPE_STM32_AFIO);
  //    qdev_prop_set_ptr(afio_dev, "stm32_rcc", rcc_dev);
  stm32_afio_set_rcc(STM32_AFIO(afio_dev), STM32_RCC(rcc_dev));
  object_property_set_link(OBJECT(afio_dev), "gpio[a]", OBJECT(gpio_dev[0]),
                           NULL);
  object_property_set_link(OBJECT(afio_dev), "gpio[b]", OBJECT(gpio_dev[1]),
                           NULL);
  object_property_set_link(OBJECT(afio_dev), "gpio[c]", OBJECT(gpio_dev[2]),
                           NULL);
  object_property_set_link(OBJECT(afio_dev), "gpio[d]", OBJECT(gpio_dev[3]),
                           NULL);
  object_property_set_link(OBJECT(afio_dev), "gpio[e]", OBJECT(gpio_dev[4]),
                           NULL);
  object_property_set_link(OBJECT(afio_dev), "gpio[f]", OBJECT(gpio_dev[5]),
                           NULL);
  object_property_set_link(OBJECT(afio_dev), "gpio[g]", OBJECT(gpio_dev[6]),
                           NULL);
  object_property_set_link(OBJECT(afio_dev), "exti", OBJECT(exti_dev), NULL);
  object_property_add_child(stm32_container, "afio", OBJECT(afio_dev));
  if (!sysbus_realize(SYS_BUS_DEVICE(afio_dev), errp)) {
    return;
  }
  stm32_init_periph(afio_dev, STM32_AFIO_PERIPH, 0x40010000, NULL);

  stm32_create_uart_dev(stm32_container, STM32_UART1, 1, rcc_dev, gpio_dev,
                        afio_dev, 0x40013800,
                        qdev_get_gpio_in(armv7m, STM32_UART1_IRQ), errp);
  stm32_create_uart_dev(stm32_container, STM32_UART2, 2, rcc_dev, gpio_dev,
                        afio_dev, 0x40004400,
                        qdev_get_gpio_in(armv7m, STM32_UART2_IRQ), errp);
  stm32_create_uart_dev(stm32_container, STM32_UART3, 3, rcc_dev, gpio_dev,
                        afio_dev, 0x40004800,
                        qdev_get_gpio_in(armv7m, STM32_UART3_IRQ), errp);
  stm32_create_uart_dev(stm32_container, STM32_UART4, 4, rcc_dev, gpio_dev,
                        afio_dev, 0x40004c00,
                        qdev_get_gpio_in(armv7m, STM32_UART4_IRQ), errp);
  stm32_create_uart_dev(stm32_container, STM32_UART5, 5, rcc_dev, gpio_dev,
                        afio_dev, 0x40005000,
                        qdev_get_gpio_in(armv7m, STM32_UART5_IRQ), errp);

  /* Timer 1 has four interrupts but only the TIM1 Update interrupt is
   * implemented. */
  /*qemu_irq tim1_irqs[] = { pic[TIM1_BRK_IRQn], pic[TIM1_UP_IRQn],
   * pic[TIM1_TRG_COM_IRQn], pic[TIM1_CC_IRQn]};*/
  stm32_create_timer_dev(stm32_container, STM32_TIM1, 1, rcc_dev, gpio_dev,
                         afio_dev, 0x40012C00,
                         qdev_get_gpio_in(armv7m, TIM1_UP_IRQn), errp);
  stm32_create_timer_dev(stm32_container, STM32_TIM2, 2, rcc_dev, gpio_dev,
                         afio_dev, 0x40000000,
                         qdev_get_gpio_in(armv7m, TIM2_IRQn), errp);
  stm32_create_timer_dev(stm32_container, STM32_TIM3, 3, rcc_dev, gpio_dev,
                         afio_dev, 0x40000400,
                         qdev_get_gpio_in(armv7m, TIM3_IRQn), errp);
  stm32_create_timer_dev(stm32_container, STM32_TIM4, 4, rcc_dev, gpio_dev,
                         afio_dev, 0x40000800,
                         qdev_get_gpio_in(armv7m, TIM4_IRQn), errp);
  stm32_create_timer_dev(stm32_container, STM32_TIM5, 5, rcc_dev, gpio_dev,
                         afio_dev, 0x40000C00,
                         qdev_get_gpio_in(armv7m, TIM5_IRQn), errp);
  stm32_create_adc_dev(stm32_container, STM32_ADC1, 1, rcc_dev, gpio_dev,
                       0x40012400, 0, errp);
  stm32_create_adc_dev(stm32_container, STM32_ADC2, 2, rcc_dev, gpio_dev,
                       0x40012800, 0, errp);
  stm32_create_rtc_dev(stm32_container, STM32_RTC, 1, rcc_dev, 0x40002800,
                       qdev_get_gpio_in(armv7m, STM32_RTC_IRQ), errp);
  stm32_create_rtc_dev(stm32_container, STM32_RTC, 2, rcc_dev, 0x40002800,
                       qdev_get_gpio_in(armv7m, STM32_RTCAlarm_IRQ), errp);
  stm32_create_dac_dev(stm32_container, STM32_DAC, rcc_dev, gpio_dev,
                       0x40007400, 0, errp);

  /* IWDG */
  DeviceState *iwdg_dev = qdev_new(TYPE_STM32_IWDG);
  // qdev_prop_set_ptr(iwdg_dev, "stm32_rcc", rcc_dev);
  stm32_iwdg_set_rcc(STM32_IWDG(iwdg_dev), STM32_RCC(rcc_dev));
  stm32_init_periph(iwdg_dev, STM32_IWDG, 0x40003000, NULL);

  /* CRC */
  DeviceState *crc = qdev_new("stm32-crc");
  stm32_init_periph(crc, STM32_CRC, 0x40023000, NULL);

  /* FLASH regs */
  DeviceState *flash_regs = qdev_new(TYPE_STM32_FLASH_REGS);
  stm32_init_periph(flash_regs, STM32_FLASH_REGS, 0x40022000, NULL);

  DeviceState *dma1 = qdev_new("stm32_dma");
  stm32_init_periph(dma1, STM32_DMA1, 0x40020000, NULL);
  sysbus_connect_irq(SYS_BUS_DEVICE(dma1), 0,
                     qdev_get_gpio_in(armv7m, STM32_DMA1_STREAM0_IRQ));
  sysbus_connect_irq(SYS_BUS_DEVICE(dma1), 1,
                     qdev_get_gpio_in(armv7m, STM32_DMA1_STREAM1_IRQ));
  sysbus_connect_irq(SYS_BUS_DEVICE(dma1), 2,
                     qdev_get_gpio_in(armv7m, STM32_DMA1_STREAM2_IRQ));
  sysbus_connect_irq(SYS_BUS_DEVICE(dma1), 3,
                     qdev_get_gpio_in(armv7m, STM32_DMA1_STREAM3_IRQ));
  sysbus_connect_irq(SYS_BUS_DEVICE(dma1), 4,
                     qdev_get_gpio_in(armv7m, STM32_DMA1_STREAM4_IRQ));
  sysbus_connect_irq(SYS_BUS_DEVICE(dma1), 5,
                     qdev_get_gpio_in(armv7m, STM32_DMA1_STREAM5_IRQ));
  sysbus_connect_irq(SYS_BUS_DEVICE(dma1), 6,
                     qdev_get_gpio_in(armv7m, STM32_DMA1_STREAM6_IRQ));
  sysbus_connect_irq(SYS_BUS_DEVICE(dma1), 7,
                     qdev_get_gpio_in(armv7m, STM32_DMA1_STREAM7_IRQ));

  /* I2C */
  DeviceState * i2c0 = qdev_new(TYPE_STM32_I2C);
  i2c0->id = (char*)stm32_periph_name_arr[STM32_I2C1];
  qdev_prop_set_int32(i2c0, "periph", STM32_I2C1);
  object_property_add_child(stm32_container, "i2c[1]", OBJECT(i2c0 ));
  if (!sysbus_realize(SYS_BUS_DEVICE(i2c0), errp)) {
    return;
  }
  stm32_init_periph(i2c0, STM32_I2C1, 0x40005400, qdev_get_gpio_in(armv7m, STM32_I2C1_EV_IRQ));
  sysbus_connect_irq(SYS_BUS_DEVICE(i2c0), 1, qdev_get_gpio_in(armv7m, STM32_I2C1_ER_IRQ));

  
  DeviceState * i2c1 = qdev_new(TYPE_STM32_I2C);
  i2c1->id = (char*)stm32_periph_name_arr[STM32_I2C2];
  qdev_prop_set_int32(i2c1, "periph", STM32_I2C2);
  object_property_add_child(stm32_container, "i2c[2]", OBJECT(i2c1 ));
  if (!sysbus_realize(SYS_BUS_DEVICE(i2c1), errp)) {
    return;
  }
  stm32_init_periph(i2c1, STM32_I2C2, 0x40005800, qdev_get_gpio_in(armv7m, STM32_I2C2_EV_IRQ));
  sysbus_connect_irq(SYS_BUS_DEVICE(i2c1), 1, qdev_get_gpio_in(armv7m, STM32_I2C2_ER_IRQ));
  
  
/*
  DeviceState * i2c2 = qdev_new(TYPE_STM32_I2C);
  i2c2->id = (char*)stm32_periph_name_arr[STM32_I2C3];
  qdev_prop_set_int32(i2c2, "periph", STM32_I2C3);
  object_property_add_child(stm32_container, "i2c[3]", OBJECT(i2c2 ));
  if (!sysbus_realize(SYS_BUS_DEVICE(i2c1), errp)) {
    return;
  }
  stm32_init_periph(i2c2, STM32_I2C2, 0x40005C00, qdev_get_gpio_in(armv7m, STM32_I2C3_EV_IRQ));
  sysbus_connect_irq(SYS_BUS_DEVICE(i2c2), 1, qdev_get_gpio_in(armv7m, STM32_I2C3_ER_IRQ));
*/

  /* SPI */
  DeviceState * spi0 = qdev_new(TYPE_STM32_SPI);
  spi0->id = (char*)stm32_periph_name_arr[STM32_SPI1];
  qdev_prop_set_int32(spi0, "periph", STM32_SPI1);
  object_property_add_child(stm32_container, "spi[1]", OBJECT(spi0));
  if (!sysbus_realize(SYS_BUS_DEVICE(spi0), errp)) {
    return;
  }
  stm32_init_periph(spi0, STM32_SPI1, 0x40013000, qdev_get_gpio_in(armv7m, STM32_SPI1_IRQ));
  
  DeviceState * spi1 = qdev_new(TYPE_STM32_SPI);
  spi1->id = (char*)stm32_periph_name_arr[STM32_SPI2];
  qdev_prop_set_int32(spi1, "periph", STM32_SPI2);
  object_property_add_child(stm32_container, "spi[2]", OBJECT(spi1));
  if (!sysbus_realize(SYS_BUS_DEVICE(spi1), errp)) {
    return;
  }
  stm32_init_periph(spi1, STM32_SPI2, 0x40003800, qdev_get_gpio_in(armv7m, STM32_SPI2_IRQ));
   

  create_unimplemented_device("WWDG", 0x40002C00, 0x400);
  create_unimplemented_device("BKP", 0x40006C00, 0x400);
  create_unimplemented_device("PWR", 0x40007000, 0x400);
  create_unimplemented_device("CEC", 0x40007800, 0x400);

  // DWT
  ARMv7MState *sc = ARMV7M(armv7m);
  memory_region_init_io(&s->DWT.dwt_mem, OBJECT(s), &dwt_ops, &s->DWT, "dwt",
                        0x5); // real size is 0xFFF, only use addr 0 and 4 for
                              // CTRL and CYCCNT fake device
  memory_region_add_subregion_overlap(&sc->container, 0xE0001000,
                                      &s->DWT.dwt_mem, 1);
}

static Property stm32f103_soc_properties[] = {
    DEFINE_PROP_STRING("cpu-type", STM32F103State, cpu_type),
    DEFINE_PROP_STRING("kernel-file", STM32F103State, kernel_file),
    DEFINE_PROP_UINT64("flash_size", STM32F103State, flash_size, 0x00020000),
    DEFINE_PROP_UINT64("ram_size", STM32F103State, ram_size, 0x00005000),
    DEFINE_PROP_UINT32("osc_freq", STM32F103State, osc_freq, 8000000),
    DEFINE_PROP_UINT32("osc32_freq", STM32F103State, osc32_freq, 32768),
    DEFINE_PROP_END_OF_LIST(),
};

static void stm32f103_soc_class_init(ObjectClass *klass, void *data) {
  DeviceClass *dc = DEVICE_CLASS(klass);

  dc->realize = stm32f103_soc_realize;
  device_class_set_props(dc, stm32f103_soc_properties);
}

static const TypeInfo stm32f103_soc_info = {
    .name = TYPE_STM32F103_SOC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(STM32F103State),
    .instance_init = stm32f103_soc_initfn,
    .class_init = stm32f103_soc_class_init,
};

static void stm32f103_soc_types(void) {
  type_register_static(&stm32f103_soc_info);
}

type_init(stm32f103_soc_types)
