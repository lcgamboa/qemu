/*
 * STM32 fa103c8 (Blue Pill) Development Board
 *
 * Copyright (C) 2018 Basel Alsayeh
 * Copyright (C) 2020 Luis CLaudio G Lopes
 *
 * Implementation based on
 * Olimex "STM-P103 Development Board Users Manual Rev. A, April 2008"
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
#include "hw/sysbus.h"
#include "hw/arm/armv7m.h"
// #include "hw/devices.h"
#include "ui/console.h"
#include "sysemu/sysemu.h"
#include "hw/boards.h"
#include "hw/irq.h"
#include "hw/arm/boot.h"
#include "hw/i2c/i2c.h"
#include "hw/ssi/ssi.h"

#define STM32_GPIOS_DIR "stm32_gpios_dir"
#define STM32_GPIOS_SYNC "stm32_gpios_sync"

typedef struct
{
   Stm32 *stm32;
   qemu_irq pin_irq[100];
   qemu_irq *pout_irq;
   qemu_irq *pdir_irq;
   qemu_irq *psync_irq;
   DeviceState *gpio_a;
   DeviceState *gpio_b;
   DeviceState *gpio_c;
   DeviceState *gpio_d;
   DeviceState *uart1;
   DeviceState *uart2;
   DeviceState *uart3;
   DeviceState *i2c1;
   DeviceState *i2c2;
   DeviceState *spi1;
   DeviceState *spi2;
   DeviceState *afio;
} Stm32_board;

Stm32_board *s;

extern unsigned short ADC_values[31];
// prototypes
void qemu_picsimlab_register_callbacks(void *arg);
void qemu_picsimlab_set_apin(int chn, int value);
void qemu_picsimlab_set_pin(int pin, int value);
uint32_t *qemu_picsimlab_get_internals(int cfg);
uint32_t qemu_picsimlab_get_TIOCM(void);
int qemu_picsimlab_flash_dump(int64_t offset, void *buf, int bytes);
void qemu_picsimlab_uart_receive(const int id, const uint8_t *buf, int size);

static uint32_t internal;
uint32_t *qemu_picsimlab_get_internals(int cfg)
{
   internal = stm32_afio_get_periph_map((Stm32Afio *)s->afio, cfg & 0x00FF);
   return &internal;
}

uint32_t qemu_picsimlab_get_TIOCM(void)
{
   // ESP32UARTState *s = ESP32_UART(&(global_s->uart[0]));

   uint32_t state = 0;
   // qemu_chr_fe_ioctl(&s->chr, CHR_IOCTL_SERIAL_GET_TIOCM, &state);

   return state;
}

int stm32_uart_can_receive(void *opaque);
void stm32_uart_receive(void *opaque, const uint8_t *buf, int size);

void qemu_picsimlab_uart_receive(const int id, const uint8_t *buf, int size)
{
   void *opaque = NULL;

   switch (id)
   {
   case 0:
      opaque = s->uart1;
      break;
   case 1:
      opaque = s->uart2;
      break;
   case 2:
      opaque = s->uart3;
      break;
   }
   stm32_uart_receive(opaque, buf, size);
}

typedef struct
{
   void (*picsimlab_write_pin)(int pin, int value);
   void (*picsimlab_dir_pin)(int pin, int value);
   int (*picsimlab_i2c_event)(const uint8_t id, const uint8_t addr, const uint16_t event);
   uint8_t (*picsimlab_spi_event)(const uint8_t id, const uint16_t event);
   void (*picsimlab_uart_tx_event)(const uint8_t id, const uint8_t val);
   const short int * pinmap;   
} callbacks_t;

void (*picsimlab_write_pin)(int pin, int value) = NULL;
void (*picsimlab_dir_pin)(int pin, int value) = NULL;
int (*picsimlab_i2c_event)(const uint8_t id, const uint8_t addr, const uint16_t event) = NULL;
uint8_t (*picsimlab_spi_event)(const uint8_t id, const uint16_t event) = NULL;
void (*picsimlab_uart_tx_event)(const uint8_t id, const uint8_t val) = NULL;
const short int * pinmap = NULL;

void qemu_picsimlab_register_callbacks(void *arg)
{
   const callbacks_t *callbacks = (const callbacks_t *)arg;

   picsimlab_write_pin = callbacks->picsimlab_write_pin;
   picsimlab_dir_pin = callbacks->picsimlab_dir_pin;
   picsimlab_i2c_event = callbacks->picsimlab_i2c_event;
   picsimlab_spi_event = callbacks->picsimlab_spi_event;
   picsimlab_uart_tx_event = callbacks->picsimlab_uart_tx_event;
   pinmap = callbacks->pinmap;   
}

void qemu_picsimlab_set_pin(int pin, int value)
{
   // qemu_mutex_lock_iothread ();
   if (value)
   {
      qemu_irq_raise(s->pin_irq[pin]);
   }
   else
   {
      qemu_irq_lower(s->pin_irq[pin]);
   }
   // qemu_mutex_unlock_iothread ();
}

void qemu_picsimlab_set_apin(int chn, int value)
{
   ADC_values[chn] = value;
}

int qemu_picsimlab_flash_dump(int64_t offset, void *buf, int bytes)
{
   cpu_physical_memory_read(0x8000000 + offset, buf, bytes);
   return bytes;
}

static void
pout_irq_handler(void *opaque, int n, int level)
{
   (*picsimlab_write_pin)(n, level);
}

static void
pdir_irq_handler(void *opaque, int n, int dir)
{
   (*picsimlab_dir_pin)(n, dir);
}

static void
psync_irq_handler(void *opaque, int n, int dir)
{
   (*picsimlab_dir_pin)(-1, -(-dir | n));
}

#define FLASH_SIZE 0x00020000
#define RAM_SIZE 0x00005000
/* Main SYSCLK frequency in Hz (24MHz) */
#define SYSCLK_FRQ 24000000ULL

static void
stm32_f103c8_picsimlab_init(MachineState *machine)
{

   Clock *sysclk;

   s = (Stm32_board *)g_malloc0(sizeof(Stm32_board));

   sysclk = clock_new(OBJECT(machine), "SYSCLK");
   clock_set_hz(sysclk, SYSCLK_FRQ);
   stm32_init(FLASH_SIZE,
              RAM_SIZE,
              machine->kernel_filename,
              8000000,
              32768,
              sysclk);

   s->gpio_a = DEVICE(object_resolve_path("/machine/stm32/gpio[a]", NULL));
   s->gpio_b = DEVICE(object_resolve_path("/machine/stm32/gpio[b]", NULL));
   s->gpio_c = DEVICE(object_resolve_path("/machine/stm32/gpio[c]", NULL));
   s->gpio_d = DEVICE(object_resolve_path("/machine/stm32/gpio[d]", NULL));
   s->uart1 = DEVICE(object_resolve_path("/machine/stm32/uart[1]", NULL));
   s->uart2 = DEVICE(object_resolve_path("/machine/stm32/uart[2]", NULL));
   s->uart3 = DEVICE(object_resolve_path("/machine/stm32/uart[3]", NULL));
   s->i2c1 = DEVICE(object_resolve_path("/machine/stm32/i2c[1]", NULL));
   s->i2c2 = DEVICE(object_resolve_path("/machine/stm32/i2c[2]", NULL));
   s->spi1 = DEVICE(object_resolve_path("/machine/stm32/spi[1]", NULL));
   s->spi2 = DEVICE(object_resolve_path("/machine/stm32/spi[2]", NULL));
   s->afio = DEVICE(object_resolve_path("/machine/stm32/afio", NULL));

   assert(s->gpio_a);
   assert(s->gpio_b);
   assert(s->gpio_c);
   assert(s->gpio_d);
   assert(s->uart2);
   assert(s->uart1);
   assert(s->uart3);
   assert(s->i2c1);
   assert(s->i2c2);
   assert(s->spi1);
   assert(s->spi2);
   assert(s->afio);

   s->psync_irq = qemu_allocate_irqs(psync_irq_handler, NULL, 4);
   qdev_connect_gpio_out_named(s->gpio_a, STM32_GPIOS_SYNC, 0, s->psync_irq[0]);
   qdev_connect_gpio_out_named(s->gpio_b, STM32_GPIOS_SYNC, 0, s->psync_irq[1]);
   qdev_connect_gpio_out_named(s->gpio_c, STM32_GPIOS_SYNC, 0, s->psync_irq[2]);
   qdev_connect_gpio_out_named(s->gpio_d, STM32_GPIOS_SYNC, 0, s->psync_irq[3]);

   s->pdir_irq = qemu_allocate_irqs(pdir_irq_handler, NULL, 49);
   s->pout_irq = qemu_allocate_irqs(pout_irq_handler, NULL, 49);

   if(pinmap){
      for(int pin = 1; pin < 49; pin++){
        if(pinmap[pin-1] >= 0 ){
           DeviceState * gport = NULL;

           switch ((pinmap[pin-1] & 0xF000) >> 12)
           {
           case 1:
             gport = s->gpio_a;
            break;
           case 2:
             gport = s->gpio_b;
            break;
           case 3:
             gport = s->gpio_c;
            break;
           case 4:
             gport = s->gpio_d;
            break;                                 
           } 

           if(gport){
             qdev_connect_gpio_out(gport, pinmap[pin-1] & 0x0FFF, s->pout_irq[pin]);
             qdev_connect_gpio_out_named(gport, STM32_GPIOS_DIR, pinmap[pin-1] & 0x0FFF, s->pdir_irq[pin]);
             s->pin_irq[pin] = qdev_get_gpio_in(gport, pinmap[pin-1] & 0x0FFF);
           }
        }
      } 
    }

   /* Connect RS232 to UART 1 */
   stm32_uart_connect(
       (Stm32Uart *)s->uart1,
       serial_hd(0),
       STM32_USART1_NO_REMAP);

   /* These additional UARTs have not been tested yet... */
   stm32_uart_connect(
       (Stm32Uart *)s->uart2,
       serial_hd(1),
       STM32_USART2_NO_REMAP);

   stm32_uart_connect(
       (Stm32Uart *)s->uart3,
       serial_hd(2),
       STM32_USART3_NO_REMAP);

   DeviceState *i2c_master1 = DEVICE(s->i2c1);
   I2CBus *i2c_bus1 = I2C_BUS(qdev_get_child_bus(i2c_master1, "i2c"));
   i2c_slave_create_simple(i2c_bus1, "picsimlab_i2c", 0x00);

   DeviceState *i2c_master2 = DEVICE(s->i2c2);
   I2CBus *i2c_bus2 = I2C_BUS(qdev_get_child_bus(i2c_master2, "i2c"));
   i2c_slave_create_simple(i2c_bus2, "picsimlab_i2c", 0x01);

   DeviceState *spi_master1 = DEVICE(s->spi1);
   SSIBus *spi_bus1 = (SSIBus *)qdev_get_child_bus(spi_master1, "ssi");
   ssi_create_peripheral(spi_bus1, "picsimlab_spi");

   DeviceState *spi_master2 = DEVICE(s->spi2);
   SSIBus *spi_bus2 = (SSIBus *)qdev_get_child_bus(spi_master2, "ssi");
   ssi_create_peripheral(spi_bus2, "picsimlab_spi");

   armv7m_load_kernel(ARM_CPU(first_cpu),
                      machine->kernel_filename,
                      FLASH_SIZE);
}

static void stm32_f103c8_picsimlab_machine_init(MachineClass *mc)
{
   mc->desc = "STM32F103C8 (Blue Pill) Dev Board (PICSimLab)";
   mc->init = stm32_f103c8_picsimlab_init;
   mc->block_default_type = IF_IDE;
   mc->ignore_memory_transaction_failures = true;
}

DEFINE_MACHINE("stm32-f103c8-picsimlab-new", stm32_f103c8_picsimlab_machine_init)
