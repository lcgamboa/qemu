/*
 * ESP32 SHA accelerator
 *
 * Copyright (c) 2019 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/registerfields.h"
#include "hw/misc/esp32c3_sha.h"


#define SHA_WARNING 0


static ESP32C3HashAlg esp32c3_algs[] = {
    [ESP32C3_SHA_1_MODE]    = {
        .init     = (hash_init) sha1_init,
        .compress = (hash_compress) sha1_compress,
        .len      = sizeof(struct sha1_state)
    },
    [ESP32C3_SHA_224_MODE]  = {
        .init     = (hash_init) sha224_init,
        .compress = (hash_compress) sha224_compress,
        .len      = SHA224_HASH_SIZE
    },
    [ESP32C3_SHA_256_MODE]  = {
        .init     = (hash_init) sha256_init,
        .compress = (hash_compress) sha256_compress,
        .len      = sizeof(struct sha256_state)
    },
};


static void esp32c3_resume_hash(ESP32C3ShaState *s)
{
    /* TODO: we can imagine a scenario where the application wants to calculate the hash of a
     * precalculated context, in that case, the application would fill the `hash` registers and
     * call `continue` instead of using `start`. We don't support that for the moment. */
    assert(s->mode <= ESP32C3_SHA_256_MODE);
    ESP32C3HashAlg alg = esp32c3_algs[s->mode];

    assert(alg.compress);

    alg.compress(&s->context, (uint8_t*) s->message);
    memcpy(s->hash, &s->context, alg.len);
}


static void esp32c3_start_hash(ESP32C3ShaState *s)
{
    ESP32C3HashAlg alg = esp32c3_algs[s->mode];

    if (alg.init && alg.compress) {
        alg.init(&s->context);
        esp32c3_resume_hash(s);
    }
}


static uint64_t esp32c3_sha_read(void *opaque, hwaddr addr, unsigned int size)
{
    ESP32C3ShaState *s = ESP32C3_SHA(opaque);
    hwaddr index = 0;

    uint64_t r = 0;
    switch (addr) {
    case A_SHA_MODE:
        r = s->mode;
        break;
    case A_SHA_BUSY:
        /* SHA driver is never busy as calculation happens synchronously */
        r = 0;
        break;
    case A_SHA_DATE:
        /* Hardcode the version control register for now */
        r = 0x20190402;
        break;
    case A_SHA_H_MEM ... A_SHA_M_MEM - 1:
        index = (addr - A_SHA_H_MEM) / sizeof(uint32_t);
        r = __builtin_bswap32(s->hash[index]);
        break;
    case A_SHA_M_MEM ... ESP32C3_SHA_REGS_SIZE - 1:
        index = (addr - A_SHA_M_MEM) / sizeof(uint32_t);
        r = s->message[index];
        break;
    case A_SHA_DMA_BLOCK_NUM:
    case A_SHA_IRQ_ENA:
    default:
#if SHA_WARNING
        warn_report("[ESP32-C3] SHA DMA and IRQ unsupported for now, ignoring...\n");
#endif
        break;
    }

    return r;
}


static void esp32c3_sha_write(void *opaque, hwaddr addr,
                       uint64_t value, unsigned int size)
{
    ESP32C3ShaState *s = ESP32C3_SHA(opaque);
    hwaddr index = 0;

    switch (addr) {
    case A_SHA_MODE:
        /* Make sure the value is always between 0 and 2 as the real hardware doesn't
         * accept a value of 3. Choose SHA-1 by default in that case. */
        s->mode = (value & 0b11) % 3;
        break;
    case A_SHA_START:
        esp32c3_start_hash(s);
        break;
    case A_SHA_CONTINUE:
        esp32c3_resume_hash(s);
        break;
    case A_SHA_H_MEM ... A_SHA_M_MEM - 1:
        index = (addr - A_SHA_H_MEM) / sizeof(uint32_t);
        s->hash[index] = (uint32_t) value;
        break;
    case A_SHA_M_MEM ... ESP32C3_SHA_REGS_SIZE - 1:
        index = (addr - A_SHA_M_MEM) / sizeof(uint32_t);
        s->message[index] = (uint32_t) value;
        break;
    case A_SHA_DMA_BLOCK_NUM:
    case A_SHA_DMA_START:
    case A_SHA_DMA_CONTINUE:
    case A_SHA_CLEAR_IRQ:
    case A_SHA_IRQ_ENA:
    default:
#if SHA_WARNING
        /* Unsupported for now, do nothing */
        warn_report("[SHA] DMA and IRQ unsupported for now, ignoring\n");
#endif
        break;
    }
}

static const MemoryRegionOps esp32c3_sha_ops = {
    .read =  esp32c3_sha_read,
    .write = esp32c3_sha_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32c3_sha_init(Object *obj)
{
    ESP32C3ShaState *s = ESP32C3_SHA(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32c3_sha_ops, s,
                          TYPE_ESP32C3_SHA, ESP32C3_SHA_REGS_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
}

static const TypeInfo esp32c3_sha_info = {
    .name = TYPE_ESP32C3_SHA,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32C3ShaState),
    .instance_init = esp32c3_sha_init,
};

static void esp32c3_sha_register_types(void)
{
    type_register_static(&esp32c3_sha_info);
}

type_init(esp32c3_sha_register_types)
