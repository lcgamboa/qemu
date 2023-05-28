/*
 * ESP32 SPI controller
 *
 * Copyright (c) 2019 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "sysemu/sysemu.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/registerfields.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/ssi/ssi.h"
#include "hw/ssi/esp32c3_spi.h"

#define SPI1_DEBUG      0
#define SPI1_WARNING    0

#define ESP32C3_SPI_FLASH_ID 0xc84016

static uint64_t esp32c3_spi_read(void *opaque, hwaddr addr, unsigned int size)
{
    ESP32C3SpiState *s = ESP32C3_SPI(opaque);

    uint64_t r = 0;
    switch (addr) {
        case A_SPI_MEM_CMD:
            r = 0;
            break;
        case A_SPI_MEM_ADDR:
            r = s->mem_addr;
            break;
        case A_SPI_MEM_CTRL:
            r = s->mem_ctrl;
            break;
        case A_SPI_MEM_USER:
            r = s->mem_user;
            break;
        case A_SPI_MEM_W0...A_SPI_MEM_W15:
            r = s->data_reg[(addr - A_SPI_MEM_W0) / sizeof(uint32_t)];
            break;
        default:
#if SPI1_WARNING
            warn_report("[SPI1] Unsupported read to 0x%lx\n", addr);
#endif
            break;
    }

#if SPI1_DEBUG
    info_report("[SPI1] Reading 0x%lx (0x%lx)\n", addr, r);
#endif

    return r;
}


static void esp32c3_spi_txrx_buffer(ESP32C3SpiState *s,
                                    const void *tx, int tx_bytes,
                                    void *rx, int rx_bytes)
{
    int bytes = MAX(tx_bytes, rx_bytes);
    for (int i = 0; i < bytes; ++i) {
        uint8_t byte = 0;
        if (byte < tx_bytes) {
            memcpy(&byte, tx + i, 1);
        }
        uint32_t res = ssi_transfer(s->spi, byte);
        if (byte < rx_bytes) {
            memcpy(rx + i, &res, 1);
        }
    }
}


static void esp32c3_spi_begin_transaction(ESP32C3SpiState *s)
{
    /* Check if we are in the command state */
    if (s->mem_user & R_SPI_MEM_USER_USR_COMMAND_MASK) {
        /* Get the number of bytes to read from the device */
        uint32_t miso_len = FIELD_EX32(s->mem_miso_len, SPI_MEM_MISO_DLEN, USR_MISO_DBITLEN);
        miso_len = (miso_len + 1) / 8;
        /* and the number of bytes to write to the device */
        uint32_t mosi_len = FIELD_EX32(s->mem_mosi_len, SPI_MEM_MOSI_DLEN, USR_MOSI_DBITLEN);
        mosi_len = (mosi_len + 1) / 8;

        /* Get the command and its length, in bytes */
        uint32_t command = FIELD_EX32(s->mem_user2, SPI_MEM_USER2, USR_COMMAND_VALUE);
        uint32_t commands_len = FIELD_EX32(s->mem_user2, SPI_MEM_USER2, USR_COMMAND_BITLEN);
        commands_len = (commands_len + 1) / 8;

        /* Get the address and its length, in bytes */
        uint32_t address = FIELD_EX32(s->mem_addr, SPI_MEM_ADDR, USR_ADDR_VALUE);
        uint32_t address_len = FIELD_EX32(s->mem_user1, SPI_MEM_USER1, USR_ADDR_BITLEN);
        address_len = (address_len + 1) / 8;

        qemu_set_irq(s->cs_gpio[0], 0);
        esp32c3_spi_txrx_buffer(s, &command, commands_len, NULL, 0);
        esp32c3_spi_txrx_buffer(s, &address, address_len, NULL, 0);
        esp32c3_spi_txrx_buffer(s, s->data_reg, mosi_len, s->data_reg, miso_len);
        qemu_set_irq(s->cs_gpio[0], 1);
    }
}


static void esp32c3_spi_write(void *opaque, hwaddr addr,
                       uint64_t value, unsigned int size)
{
    ESP32C3SpiState *s = ESP32C3_SPI(opaque);
    uint32_t wvalue = (uint32_t) value;

    switch (addr) {
        case A_SPI_MEM_CMD:
            if (wvalue & R_SPI_MEM_CMD_FLASH_RDID_MASK) {
                s->data_reg[0] = ESP32C3_SPI_FLASH_ID;
            } else if(wvalue & R_SPI_MEM_CMD_USR_MASK) {
                esp32c3_spi_begin_transaction(s);
            }
            break;
        case A_SPI_MEM_ADDR:
            s->mem_addr = wvalue;
            break;
        case A_SPI_MEM_CTRL:
            s->mem_ctrl = wvalue;
            break;
        case A_SPI_MEM_USER:
            s->mem_user = wvalue;
            break;
        case A_SPI_MEM_USER1:
            s->mem_user1 = wvalue;
            break;
        case A_SPI_MEM_USER2:
            s->mem_user2 = wvalue;
            break;
        case A_SPI_MEM_MISO_DLEN:
            s->mem_miso_len = wvalue;
            break;
        case A_SPI_MEM_MOSI_DLEN:
            s->mem_mosi_len = wvalue;
            break;
        case A_SPI_MEM_W0...A_SPI_MEM_W15:
            s->data_reg[(addr - A_SPI_MEM_W0) / sizeof(uint32_t)] = wvalue;
            break;
        default:
#if SPI1_WARNING
            warn_report("[SPI1] Unsupported write to 0x%lx (%08lx)\n", addr, value);
#endif
            break;
    }

#if SPI1_DEBUG
    info_report("[SPI1] Writing 0x%lx = %08lx\n", addr, value);
#endif
}

/* Convert one of the hardware "bitlen" registers to a byte count */
static inline int bitlen_to_bytes(uint32_t val)
{
    return (val + 1 + 7) / 8; /* bitlen registers hold number of bits, minus one */
}

static const MemoryRegionOps esp32c3_spi_ops = {
    .read =  esp32c3_spi_read,
    .write = esp32c3_spi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32c3_spi_reset(DeviceState *dev)
{
    ESP32C3SpiState *s = ESP32C3_SPI(dev);
    memset(s->data_reg, 0, ESP32C3_SPI_BUF_WORDS * sizeof(uint32_t));
}

static void esp32c3_spi_realize(DeviceState *dev, Error **errp)
{
}

static void esp32c3_spi_init(Object *obj)
{
    ESP32C3SpiState *s = ESP32C3_SPI(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32c3_spi_ops, s,
                          TYPE_ESP32C3_SPI, ESP32C3_SPI_IO_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    // sysbus_init_irq(sbd, &s->irq);

    s->spi = ssi_create_bus(DEVICE(s), "spi");
    qdev_init_gpio_out_named(DEVICE(s), &s->cs_gpio[0], SSI_GPIO_CS, ESP32C3_SPI_CS_COUNT);
}

static Property esp32c3_spi_properties[] = {
    DEFINE_PROP_END_OF_LIST(),
};

static void esp32c3_spi_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = esp32c3_spi_reset;
    dc->realize = esp32c3_spi_realize;
    device_class_set_props(dc, esp32c3_spi_properties);
}

static const TypeInfo esp32c3_spi_info = {
    .name = TYPE_ESP32C3_SPI,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32C3SpiState),
    .instance_init = esp32c3_spi_init,
    .class_init = esp32c3_spi_class_init
};

static void esp32c3_spi_register_types(void)
{
    type_register_static(&esp32c3_spi_info);
}

type_init(esp32c3_spi_register_types)
