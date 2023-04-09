/*
 * PICSimLab I2C  Emulation
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
#include "hw/hw.h"
#include "hw/i2c/i2c.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "migration/vmstate.h"

extern int (*picsimlab_i2c_event)(const uint8_t id, const uint8_t addr, const uint16_t event);

static int i2c_id = 0;

typedef struct PICSIMLAB_I2CState {
  /*< private >*/
  I2CSlave i2c;
  /*< public >*/
  uint8_t device_addr;
  uint8_t id;
} PICSIMLAB_I2CState;

#define TYPE_PICSIMLAB_I2C "picsimlab_i2c"
#define PICSIMLAB_I2C(obj)                                                     \
  OBJECT_CHECK(PICSIMLAB_I2CState, (obj), TYPE_PICSIMLAB_I2C)

static void picsimlab_i2c_reset(DeviceState *dev) {
  // PICSIMLAB_I2CState *s = PICSIMLAB_I2C(dev);
}

static uint8_t picsimlab_i2c_rx(I2CSlave *i2c) {
  PICSIMLAB_I2CState *s = PICSIMLAB_I2C(i2c);
  //  read i2c
  return picsimlab_i2c_event(s->id, s->device_addr, I2C_NACK+2);
}

static int picsimlab_i2c_tx(I2CSlave *i2c, uint8_t data) {
  PICSIMLAB_I2CState *s = PICSIMLAB_I2C(i2c);
  //  write i2c
  return picsimlab_i2c_event(s->id, s->device_addr, (data<<8)|(I2C_NACK+1));
}

static int picsimlab_i2c_ev(I2CSlave *i2c, enum i2c_event event) {
  PICSIMLAB_I2CState *s = PICSIMLAB_I2C(i2c);
  // i2c event
  return picsimlab_i2c_event(s->id, s->device_addr, event);
}

/*
 * For each channel, if it's enabled, recursively call match on those children.
 */
static bool picsimlab_i2c_match(I2CSlave *candidate, uint8_t address,
                                bool broadcast, I2CNodeList *current_devs) {
  PICSIMLAB_I2CState *s = PICSIMLAB_I2C(candidate);
  // Always return true
  s->device_addr = address;
  I2CNode *node = g_malloc(sizeof(struct I2CNode));
  node->elt = candidate;
  QLIST_INSERT_HEAD(current_devs, node, next);
  return true;
}

static void picsimlab_i2c_realize(DeviceState *dev, Error **errp) {
    I2CSlave *i2c = I2C_SLAVE(dev);
    PICSIMLAB_I2CState *s = PICSIMLAB_I2C(i2c);
    s->id = i2c_id++;
}

static const VMStateDescription vmstate_picsimlab_i2c = {
    .name = "PICSIMLAB_I2C",
    .version_id = 0,
    .minimum_version_id = 0,
    .fields = (VMStateField[]){VMSTATE_UINT8(device_addr, PICSIMLAB_I2CState),
                               VMSTATE_I2C_SLAVE(i2c, PICSIMLAB_I2CState),
                               VMSTATE_END_OF_LIST()}};

static void picsimlab_i2c_class_init(ObjectClass *klass, void *data) {
  DeviceClass *dc = DEVICE_CLASS(klass);
  I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);
    k->event = picsimlab_i2c_ev;
    k->recv = picsimlab_i2c_rx;
    k->send = picsimlab_i2c_tx;
    k->match_and_add = picsimlab_i2c_match;
    dc->reset = picsimlab_i2c_reset;
    dc->vmsd = &vmstate_picsimlab_i2c;
    dc->realize = picsimlab_i2c_realize;
  }

  static const TypeInfo picsimlab_i2c_info = {
      .name = TYPE_PICSIMLAB_I2C,
      .parent = TYPE_I2C_SLAVE,
      .instance_size = sizeof(PICSIMLAB_I2CState),
      .class_init = picsimlab_i2c_class_init,
  };

  static void picsimlab_i2c_register_types(void) {
    type_register_static(&picsimlab_i2c_info);
  }


type_init(picsimlab_i2c_register_types)
