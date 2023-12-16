/*
 * STM32 fa103c8 (Blue Pill) Development Board
 *
 * Copyright (C) 2018 Basel Alsayeh
 *
 * Implementation based on
 * Olimex "STM-P103 Development Board Users Manual Rev. A, April 2008"
 *
 * Andre Beckus
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
#include "ui/console.h"
#include "sysemu/sysemu.h"
#include "hw/boards.h"
#include "hw/arm/boot.h"
#include "hw/irq.h"

static void led_irq_handler(void *opaque, int n, int level)
{
    /* There should only be one IRQ for the LED */
    assert(n == 0);

    /* Assume that the IRQ is only triggered if the LED has changed state.
     * If this is not correct, we may get multiple LED Offs or Ons in a row.
     */
    switch (level) {
        case 0:
            printf("LED Off\n");
            break;
        case 1:
            printf("LED On\n");
            break;
    }
}

#define FLASH_SIZE 0x00020000
#define RAM_SIZE 0x00005000
#define SYSCLK_FRQ 24000000ULL

static void stm32_f103c8_init(MachineState *machine)
{
    qemu_irq *led_irq;
    Clock *sysclk;

    sysclk = clock_new(OBJECT(machine), "SYSCLK");
    clock_set_hz(sysclk, SYSCLK_FRQ);

    stm32_init (FLASH_SIZE,
             RAM_SIZE,
             machine->kernel_filename,
             8000000,
             32768,
             sysclk);

    DeviceState *gpio_a = DEVICE(object_resolve_path("/machine/stm32/gpio[a]", NULL));
    DeviceState *gpio_c = DEVICE(object_resolve_path("/machine/stm32/gpio[c]", NULL));
    DeviceState *uart2 = DEVICE(object_resolve_path("/machine/stm32/uart[2]", NULL));
    DeviceState *uart1 = DEVICE(object_resolve_path("/machine/stm32/uart[1]", NULL));
    DeviceState *uart3 = DEVICE(object_resolve_path("/machine/stm32/uart[3]", NULL));
    assert(gpio_a);
    assert(gpio_c);
    assert(uart2);
    assert(uart1);
    assert(uart3);

    /* Connect LED to GPIO C pin 13 */
    led_irq = qemu_allocate_irqs(led_irq_handler, NULL, 1);
    qdev_connect_gpio_out(gpio_c, 13, led_irq[0]);

    /* Connect button to GPIO A pin 0 */
    /*s->button_irq = qdev_get_gpio_in(gpio_a, 0);
    qemu_add_kbd_event_handler(stm32_p103_key_event, s);*/

    /* Connect RS232 to UART 1 */
    stm32_uart_connect(
            (Stm32Uart *)uart1,
            serial_hd(0),
            STM32_USART1_NO_REMAP);
    
    /* These additional UARTs have not been tested yet... */
    stm32_uart_connect(
            (Stm32Uart *)uart2,
            serial_hd(1),
            STM32_USART2_NO_REMAP);
    
    stm32_uart_connect(
            (Stm32Uart *)uart3,
            serial_hd(2),
            STM32_USART3_NO_REMAP);
            
    armv7m_load_kernel(ARM_CPU(first_cpu),
                       machine->kernel_filename,
                       0, FLASH_SIZE);
}

static void stm32_f103c8_machine_init(MachineClass *mc)
{
    mc->desc = "STM32F103C8 (Blue Pill) Dev Board";
    mc->init = stm32_f103c8_init;
    mc->block_default_type = IF_IDE;
    mc->ignore_memory_transaction_failures = true;
}

DEFINE_MACHINE("stm32-f103c8", stm32_f103c8_machine_init)

#if 0
static QEMUMachine stm32_f103c8_machine = {
    .name = "stm32-f103c8",
    .desc = "STM32F103C8 (Blue Pill) Dev Board",
    .init = stm32_f103c8_init,
};


static void stm32_f103c8_machine_init(void)
{
    qemu_register_machine(&stm32_f103c8_machine);
}

machine_init(stm32_f103c8_machine_init);
#endif
