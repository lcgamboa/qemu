/*
 * PICSimLab SPI  Emulation
 * 
 * 
 * Luis Claudio Gamboa Lopes 2022 lcgamboa@yahoo.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/qdev-properties.h"
#include "hw/ssi/ssi.h"
#include "hw/irq.h"

extern uint8_t (*picsimlab_spi_event)(const uint8_t id, const uint16_t event);

struct  picsimlab_spi_State {
    SSIPeripheral ssidev;
    uint8_t id;
};

static int spi_id = 0;

#define TYPE_PICSIMLAB_SPI "picsimlab_spi"
OBJECT_DECLARE_SIMPLE_TYPE(picsimlab_spi_State, PICSIMLAB_SPI)

static int PICSIMLAB_SPI_cs(SSIPeripheral *dev, bool select)
{
  picsimlab_spi_State *s = (picsimlab_spi_State *)(dev);
  return picsimlab_spi_event(s->id, (select<<8)|0x01);
}

static uint32_t PICSIMLAB_SPI_transfer(SSIPeripheral *dev, uint32_t data)
{
    picsimlab_spi_State *s = (picsimlab_spi_State *)(dev);
    return picsimlab_spi_event(s->id, data<<8);
}

static void PICSIMLAB_SPI_realize(SSIPeripheral *d, Error **errp) {
    picsimlab_spi_State *s = PICSIMLAB_SPI(d);
    //DeviceState *dev = DEVICE(s);
    s->id = spi_id++;
}


static void PICSIMLAB_SPI_class_init(ObjectClass *klass, void *data) {
    //DeviceClass *dc = DEVICE_CLASS(klass);
    SSIPeripheralClass *k = SSI_PERIPHERAL_CLASS(klass);
    k->realize = PICSIMLAB_SPI_realize;
    k->transfer = PICSIMLAB_SPI_transfer;
    k->set_cs = PICSIMLAB_SPI_cs;
    k->cs_polarity = SSI_CS_LOW;
}

static const TypeInfo PICSIMLAB_SPI_info = {
    .name = TYPE_PICSIMLAB_SPI,
    .parent = TYPE_SSI_PERIPHERAL,
    .instance_size = sizeof(picsimlab_spi_State),
    .class_init = PICSIMLAB_SPI_class_init
};

static void PICSIMLAB_SPI_register_types(void) {
    type_register_static(&PICSIMLAB_SPI_info);
}

type_init(PICSIMLAB_SPI_register_types)
