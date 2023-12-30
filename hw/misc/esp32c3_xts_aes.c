/*
 * ESP32-C3 XTS-AES emulation
 *
 * Copyright (c) 2023 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "qemu/error-report.h"
#include "crypto/aes.h"
#include "crypto/xts.h"
#include "hw/riscv/esp32c3_clk.h"
#include "hw/nvram/esp32c3_efuse.h"
#include "hw/misc/esp32c3_xts_aes.h"

#define XTS_AES_WARNING 0
#define XTS_AES_DEBUG   0

#define EFUSE_KEY_PURPOSE_XTS_AES_128_KEY 4
#define XTS_AES_KEY_SIZE 32
#define ESP32C3_XTS_AES_DATA_UNIT_SIZE 128

struct xts_aes_keys_ctx {
    AES_KEY enc;
    AES_KEY dec;
};

static void xts_aes_encrypt(const void *ctx,
                            size_t length,
                            uint8_t *dst,
                            const uint8_t *src)
{
    const struct xts_aes_keys_ctx *aesctx = ctx;

    AES_encrypt(src, dst, &aesctx->enc);
}

static void xts_aes_decrypt(const void *ctx,
                            size_t length,
                            uint8_t *dst,
                            const uint8_t *src)
{
    const struct xts_aes_keys_ctx *aesctx = ctx;

    AES_decrypt(src, dst, &aesctx->dec);
}

static bool esp32c3_xts_aes_is_ciphertext_spi_visible(ESP32C3XtsAesState *s)
{
    return (s->state == XTS_AES_RELEASE);
}

static bool esp32c3_xts_aes_is_manual_enc_enabled(ESP32C3XtsAesState *s)
{
    ESP32C3ClockClass *clock_class = ESP32C3_CLOCK_GET_CLASS(s->clock);
    ESP32C3EfuseClass *efuse_class = ESP32C3_EFUSE_GET_CLASS(s->efuse);
    uint32_t ext_dev_enc_dec_ctrl_reg = clock_class->get_ext_dev_enc_dec_ctrl(s->clock);
    return ((ext_dev_enc_dec_ctrl_reg & 1) || ((ext_dev_enc_dec_ctrl_reg & 8) && (efuse_class->get_dis_downlaod_man_encrypt == 0)));
}

static bool esp32c3_xts_aes_is_flash_enc_enabled(ESP32C3XtsAesState *s)
{
    ESP32C3EfuseClass *efuse_class = ESP32C3_EFUSE_GET_CLASS(s->efuse);
    uint32_t spi_boot_crypt_cnt = efuse_class->get_spi_boot_crypt_cnt(s->efuse);
    return (ctpop32(spi_boot_crypt_cnt) & 1);
}

static void esp32c3_xts_aes_get_key(ESP32C3XtsAesState *s, uint8_t *key)
{
    for (int i = EFUSE_BLOCK_KEY0; i < EFUSE_BLOCK_KEY6; i++) {
        if (esp32c3_efuse_get_key_purpose(s->efuse, i) == EFUSE_KEY_PURPOSE_XTS_AES_128_KEY) {
            esp32c3_efuse_get_key(s->efuse, i, key);
            return;
        }
    }
    memset(key, 0, XTS_AES_KEY_SIZE);
}

static void esp32c3_xts_aes_read_ciphertext(ESP32C3XtsAesState *s, uint32_t* spi_data_regs, uint32_t* spi_data_size, uint32_t* spi_addr, uint32_t* spi_addr_size)
{
    *spi_data_size = s->linesize == 0 ? 16 : 32;
    memcpy(spi_data_regs, s->ciphertext, *spi_data_size);
    /* Right shift address by 8 as the target memory space is a 24-bit address */
    *spi_addr = (cpu_to_be32(s->physical_addr)) >> 8;
    *spi_addr_size = 24 / 8;
}

static void esp32c3_xts_aes_encrypt(ESP32C3XtsAesState *s)
{
    uint8_t efuse_key[XTS_AES_KEY_SIZE];
    uint8_t tweak[16];
    uint32_t linesize = s->linesize == 0 ? 16 : 32;

    uint8_t input_plaintext[ESP32C3_XTS_AES_DATA_UNIT_SIZE];
    uint8_t input_plaintext_reversed[ESP32C3_XTS_AES_DATA_UNIT_SIZE];
    uint8_t output_ciphertext[ESP32C3_XTS_AES_DATA_UNIT_SIZE];
    uint8_t output_ciphertext_reversed[ESP32C3_XTS_AES_DATA_UNIT_SIZE];

    *((uint32_t *) tweak) = cpu_to_le32(s->physical_addr & 0xFFFF80);
    memset(tweak + 4, 0, 12);

    esp32c3_xts_aes_get_key(s, efuse_key);

    struct xts_aes_keys_ctx aesdata = {};
    struct xts_aes_keys_ctx aestweak = {};

    AES_set_encrypt_key(efuse_key, XTS_AES_KEY_SIZE / 2 * 8, &aesdata.enc);
    AES_set_decrypt_key(efuse_key, XTS_AES_KEY_SIZE / 2 * 8, &aesdata.dec);
    AES_set_encrypt_key(efuse_key + XTS_AES_KEY_SIZE / 2, XTS_AES_KEY_SIZE / 2 * 8, &aestweak.enc);
    AES_set_decrypt_key(efuse_key + XTS_AES_KEY_SIZE / 2, XTS_AES_KEY_SIZE / 2 * 8, &aestweak.dec);

    memset(input_plaintext, 0, ESP32C3_XTS_AES_DATA_UNIT_SIZE);

    uint32_t plaintext_offs = (s->physical_addr % (ESP32C3_XTS_AES_PLAIN_REG_CNT * 4));
    uint32_t pad_left = s->physical_addr % ESP32C3_XTS_AES_DATA_UNIT_SIZE;
    memcpy(input_plaintext + pad_left, ((uint8_t*)s->plaintext) + plaintext_offs, linesize);

    for (int i = 0; i < ESP32C3_XTS_AES_DATA_UNIT_SIZE; i++) {
        input_plaintext_reversed[i] = input_plaintext[ESP32C3_XTS_AES_DATA_UNIT_SIZE-i-1];
    }

    xts_encrypt(&aesdata, &aestweak,
                xts_aes_encrypt,
                xts_aes_decrypt,
                tweak, ESP32C3_XTS_AES_DATA_UNIT_SIZE, output_ciphertext_reversed, input_plaintext_reversed);

    for (int i = 0; i < ESP32C3_XTS_AES_DATA_UNIT_SIZE; i++) {
        output_ciphertext[i] = output_ciphertext_reversed[ESP32C3_XTS_AES_DATA_UNIT_SIZE-i-1];
    }

    memset(s->ciphertext, 0, ESP32C3_XTS_AES_PLAIN_REG_CNT * sizeof(uint32_t));
    memcpy(s->ciphertext, output_ciphertext + pad_left, linesize);
}

static void esp32c3_xts_aes_decrypt(ESP32C3XtsAesState *s, uint32_t physical_address, uint8_t *data, uint32_t size)
{
    struct xts_aes_keys_ctx aesdata = {};
    struct xts_aes_keys_ctx aestweak = {};

    uint8_t efuse_key[XTS_AES_KEY_SIZE];
    uint8_t tweak[16];

    esp32c3_xts_aes_get_key(s, efuse_key);

    AES_set_encrypt_key(efuse_key, XTS_AES_KEY_SIZE / 2 * 8, &aesdata.enc);
    AES_set_decrypt_key(efuse_key, XTS_AES_KEY_SIZE / 2 * 8, &aesdata.dec);
    AES_set_encrypt_key(efuse_key + XTS_AES_KEY_SIZE / 2, XTS_AES_KEY_SIZE / 2 * 8, &aestweak.enc);
    AES_set_decrypt_key(efuse_key + XTS_AES_KEY_SIZE / 2, XTS_AES_KEY_SIZE / 2 * 8, &aestweak.dec);

    uint8_t input_ciphertext[ESP32C3_XTS_AES_DATA_UNIT_SIZE];
    uint8_t input_ciphertext_reversed[ESP32C3_XTS_AES_DATA_UNIT_SIZE];
    uint8_t output_plaintext[ESP32C3_XTS_AES_DATA_UNIT_SIZE];
    uint8_t output_plaintext_reversed[ESP32C3_XTS_AES_DATA_UNIT_SIZE];

    for (int i = 0; i < size; i += ESP32C3_XTS_AES_DATA_UNIT_SIZE) {
        *((uint32_t *) tweak) = cpu_to_le32((physical_address + i) & 0xFFFF80);
        memset(tweak + 4, 0, 12);

        memcpy(input_ciphertext, data + i, ESP32C3_XTS_AES_DATA_UNIT_SIZE);
        for (int i = 0; i < ESP32C3_XTS_AES_DATA_UNIT_SIZE; i++) {
            input_ciphertext_reversed[i] = input_ciphertext[ESP32C3_XTS_AES_DATA_UNIT_SIZE-i-1];
        }

        xts_decrypt(&aesdata, &aestweak,
                    xts_aes_encrypt,
                    xts_aes_decrypt,
                    tweak, ESP32C3_XTS_AES_DATA_UNIT_SIZE, output_plaintext_reversed, input_ciphertext_reversed);

        for (int i = 0; i < ESP32C3_XTS_AES_DATA_UNIT_SIZE; i++) {
            output_plaintext[i] = output_plaintext_reversed[ESP32C3_XTS_AES_DATA_UNIT_SIZE-i-1];
        }

        memcpy(data + i, output_plaintext, ESP32C3_XTS_AES_DATA_UNIT_SIZE);
    }
}

static uint64_t esp32c3_xts_aes_read(void *opaque, hwaddr addr, unsigned int size)
{
    ESP32C3XtsAesState *s = ESP32C3_XTS_AES(opaque);

    uint64_t r = 0;
    switch (addr) {
        case A_XTS_AES_DATE_REG:
            r = 0x20200111;
            break;

        case A_XTS_AES_PLAIN_0_REG ... A_XTS_AES_PLAIN_7_REG:
            r = s->plaintext[(addr - A_XTS_AES_PLAIN_0_REG) / sizeof(uint32_t)];
            break;

        case A_XTS_AES_LINESIZE_REG:
            r = s->linesize;
            break;

        case A_XTS_AES_DESTINATION_REG:
            r = s->destination;
            break;

        case A_XTS_AES_PHYSICAL_ADDRESS_REG:
            r = s->physical_addr;
            break;

        case A_XTS_AES_STATE_REG:
            r = s->state;
            break;

        default:
#if XTS_AES_WARNING
            /* Other registers are not supported yet */
            warn_report("[XTS_AES] Unsupported read to %08lx\n", addr);
#endif
            break;
    }

#if XTS_AES_DEBUG
    info_report("[XTS_AES] Reading from %08lx (%08lx)\n", addr, r);
#endif

    return r;
}


static void esp32c3_xts_aes_write(void *opaque, hwaddr addr,
                       uint64_t value, unsigned int size)
{
    ESP32C3XtsAesState *s = ESP32C3_XTS_AES(opaque);

    switch (addr) {
        case A_XTS_AES_LINESIZE_REG:
            s->linesize = FIELD_EX32(value, XTS_AES_LINESIZE_REG, XTS_AES_LINESIZE);
            break;

        case A_XTS_AES_DESTINATION_REG:
            s->destination = FIELD_EX32(value, XTS_AES_DESTINATION_REG, XTS_AES_DESTINATION);
            break;

        case A_XTS_AES_PHYSICAL_ADDRESS_REG:
            s->physical_addr = FIELD_EX32(value, XTS_AES_PHYSICAL_ADDRESS_REG, XTS_AES_PHYSICAL_ADDRESS);
            if (s->physical_addr > 0x00FFFFFF) {
                error_report("[XTS-AES] Physical Adress greater than 0x00FFFFFF");
            }
            break;

        case A_XTS_AES_PLAIN_0_REG ... A_XTS_AES_PLAIN_7_REG:
            s->plaintext[(addr - A_XTS_AES_PLAIN_0_REG) / sizeof(uint32_t)] = value;
            break;

        case A_XTS_AES_TRIGGER_REG:
            if (FIELD_EX32(value, XTS_AES_TRIGGER_REG, XTS_AES_TRIGGER) == 1) {
                s->state = XTS_AES_BUSY;
                esp32c3_xts_aes_encrypt(s);
                s->state= XTS_AES_DONE;
            }
            break;

        case A_XTS_AES_RELEASE_REG:
            if (FIELD_EX32(value, XTS_AES_RELEASE_REG, XTS_AES_RELEASE) == 1) {
                // "Grant SPI1 access for the ciphertext"
                s->state = XTS_AES_RELEASE;
            }
            break;

        case A_XTS_AES_DESTROY_REG:
            FIELD_EX32(value, XTS_AES_DESTROY_REG, XTS_AES_DESTROY);
            s->state = XTS_AES_IDLE;
            memset(s->ciphertext, 0, ESP32C3_XTS_AES_PLAIN_REG_CNT * sizeof(uint32_t));
            break;

        default:
#if XTS_AES_WARNING
            /* Other registers are not supported yet */
            warn_report("[XTS_AES] Unsupported write to %08lx (%08lx)\n", addr, value);
#endif
            break;
    }

#if XTS_AES_DEBUG
    info_report("[XTS_AES] Writing to %08lx (%08lx)\n", addr, value);
#endif

}

static const MemoryRegionOps esp32c3_xts_aes_ops = {
    .read =  esp32c3_xts_aes_read,
    .write = esp32c3_xts_aes_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32c3_xts_aes_reset(DeviceState *dev)
{
    ESP32C3XtsAesState *s = ESP32C3_XTS_AES(dev);
    memset(s->plaintext, 0, ESP32C3_XTS_AES_PLAIN_REG_CNT * sizeof(uint32_t));
    memset(s->ciphertext, 0, ESP32C3_XTS_AES_PLAIN_REG_CNT * sizeof(uint32_t));
    s->state = XTS_AES_IDLE;
    s->linesize = 0;
    s->destination = 0;
    s->physical_addr = 0;
}

static void esp32c3_xts_aes_realize(DeviceState *dev, Error **errp)
{
    ESP32C3XtsAesState *s = ESP32C3_XTS_AES(dev);

    /* Make sure Efuse was set of issue an error */
    if (s->efuse == NULL) {
        error_report("[XTS_AES] Efuse controller must be set!");
    }

    /* Make sure Clock was set or issue an error */
    if (s->clock == NULL) {
        error_report("[XTS_AES] Clock controller must be set!");
    }
}

static void esp32c3_xts_aes_init(Object *obj)
{
    ESP32C3XtsAesState *s = ESP32C3_XTS_AES(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32c3_xts_aes_ops, s,
                          TYPE_ESP32C3_XTS_AES, ESP32C3_XTS_AES_REGS_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
}

static void esp32c3_xts_aes_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ESP32C3XtsAesClass* esp32c3_xts_aes = ESP32C3_XTS_AES_CLASS(klass);

    dc->realize = esp32c3_xts_aes_realize;
    dc->reset = esp32c3_xts_aes_reset;

    esp32c3_xts_aes->is_ciphertext_spi_visible = esp32c3_xts_aes_is_ciphertext_spi_visible;
    esp32c3_xts_aes->is_flash_enc_enabled = esp32c3_xts_aes_is_flash_enc_enabled;
    esp32c3_xts_aes->is_manual_enc_enabled = esp32c3_xts_aes_is_manual_enc_enabled;
    esp32c3_xts_aes->decrypt = esp32c3_xts_aes_decrypt;
    esp32c3_xts_aes->read_ciphertext = esp32c3_xts_aes_read_ciphertext;
}

static const TypeInfo esp32c3_xts_aes_info = {
    .name = TYPE_ESP32C3_XTS_AES,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32C3XtsAesState),
    .instance_init = esp32c3_xts_aes_init,
    .class_init = esp32c3_xts_aes_class_init,
    .class_size = sizeof(ESP32C3XtsAesClass)
};

static void esp32c3_xts_aes_register_types(void)
{
    type_register_static(&esp32c3_xts_aes_info);
}

type_init(esp32c3_xts_aes_register_types)

