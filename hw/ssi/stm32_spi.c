/*-
 * Copyright (c) 2013
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
/*
 * QEMU model of the stm32f2xx SPI controller.
 */
#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "hw/arm/stm32.h"
#include "hw/ssi/ssi.h"

#define	R_CR1             (0x00 / 4)
#define	R_CR1_DFF      (1 << 11)
#define	R_CR1_LSBFIRST (1 <<  7)
#define	R_CR1_SPE      (1 <<  6)
#define	R_CR2             (0x04 / 4)

#define	R_SR       (0x08 / 4)
#define	R_SR_RESET    0x0002
#define	R_SR_MASK     0x01FF
#define R_SR_OVR     (1 << 6)
#define R_SR_TXE     (1 << 1)
#define R_SR_RXNE    (1 << 0)

#define	R_DR       (0x0C / 4)
#define	R_CRCPR    (0x10 / 4)
#define	R_CRCPR_RESET 0x0007
#define	R_RXCRCR   (0x14 / 4)
#define	R_TXCRCR   (0x18 / 4)
#define	R_I2SCFGR  (0x1C / 4)
#define	R_I2SPR    (0x20 / 4)
#define	R_I2SPR_RESET 0x0002
#define R_MAX      (0x24 / 4)

typedef struct stm32_spi_state {
    SysBusDevice busdev;
    MemoryRegion iomem;
    qemu_irq irq;

    SSIBus *spi;

    stm32_periph_t periph;

    int32_t rx;
    int rx_full; 
    uint16_t regs[R_MAX];
} stm32_spi_state;

static uint64_t
stm32_spi_read(void *arg, hwaddr offset, unsigned size)
{
    stm32_spi_state *s = arg;
    uint16_t r = UINT16_MAX;

    if (!(size == 1 || size == 2 || size == 4 || (offset & 0x3) != 0)) {
        STM32_BAD_REG(offset, size);
    }
    offset >>= 2;
    if (offset < R_MAX) {
        r = s->regs[offset];
    } else {
        stm32_hw_warn("Out of range SPI write, offset 0x%x", (unsigned)offset<<2);
    }
    switch (offset) {
    case R_DR:
        s->regs[R_SR] &= ~R_SR_RXNE;
    }
    return r;
}

static uint8_t
bitswap(uint8_t val)
{
    return ((val * 0x0802LU & 0x22110LU) | (val * 0x8020LU & 0x88440LU)) * 0x10101LU >> 16;
}

static void
stm32_spi_write(void *arg, hwaddr addr, uint64_t data, unsigned size)
{
    struct stm32_spi_state *s = (struct stm32_spi_state *)arg;
    int offset = addr & 0x3;

    /* SPI registers are all at most 16 bits wide */
    data &= 0xFFFFF;
    addr >>= 2;

    switch (size) {
        case 1:
            data = (s->regs[addr] & ~(0xff << (offset * 8))) | data << (offset * 8);
            break;
        case 2:
            data = (s->regs[addr] & ~(0xffff << (offset * 8))) | data << (offset * 8);
            break;
        case 4:
            break;
        default:
            abort();
    }

    switch (addr) {
    case R_CR1:
        if ((data & R_CR1_DFF) != s->regs[R_CR1] && (s->regs[R_CR1] & R_CR1_SPE) != 0)
            qemu_log_mask(LOG_GUEST_ERROR, "cannot change DFF with SPE set\n");
        if (data & R_CR1_DFF)
            qemu_log_mask(LOG_UNIMP, "f2xx DFF 16-bit mode not implemented\n");
        s->regs[R_CR1] = data;
        break;
    case R_DR:
        s->regs[R_SR] &= ~R_SR_TXE;
        if (s->regs[R_SR] & R_SR_RXNE) {
            s->regs[R_SR] |= R_SR_OVR;
        }
        if (s->regs[R_CR1] & R_CR1_LSBFIRST) {
            s->regs[R_DR] = bitswap(ssi_transfer(s->spi, bitswap(data)));
        } else {
            s->regs[R_DR] = ssi_transfer(s->spi, data);
        }
        
        s->regs[R_SR] |= R_SR_RXNE;
        s->regs[R_SR] |= R_SR_TXE;
        break;
    default:
        if (addr < ARRAY_SIZE(s->regs)) {
            s->regs[addr] = data;
        } else {
            STM32_BAD_REG(addr, size);
        }
    }
}

static const MemoryRegionOps stm32_spi_ops = {
    .read = stm32_spi_read,
    .write = stm32_spi_write,
    .endianness = DEVICE_NATIVE_ENDIAN
};

static void
stm32_spi_reset(DeviceState *dev)
{
    struct stm32_spi_state *s = STM32_SPI(dev);

    s->regs[R_SR] = R_SR_RESET;
    switch (s->periph) {
    case 0:
        break;
    case 1:
        break;
    default:
        break;
    }
}

static void
stm32_spi_init(Object *obj)
{
    struct stm32_spi_state *s = STM32_SPI(obj);

    memory_region_init_io(&s->iomem, obj, &stm32_spi_ops, s, "spi", 0x3ff);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->spi = ssi_create_bus(DEVICE(obj), "ssi");

    return;
}


static Property stm32_spi_properties[] = {
    DEFINE_PROP_INT32("periph", struct stm32_spi_state, periph, -1),
    DEFINE_PROP_END_OF_LIST()
};

static void
stm32_spi_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    
    dc->reset = stm32_spi_reset;
    device_class_set_props(dc, stm32_spi_properties);
}

static const TypeInfo stm32_spi_info = {
    .name = TYPE_STM32_SPI,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(struct stm32_spi_state),
    .instance_init = stm32_spi_init,
    .class_init = stm32_spi_class_init
};

static void
stm32_spi_register_types(void)
{
    type_register_static(&stm32_spi_info);
}

type_init(stm32_spi_register_types)

/*
 *
 */
/*
 * Serial peripheral interface (SPI) RM0033 section 25
 */

