#pragma once

#define DR_REG_FRAMEBUF_BASE                    0x21000000

#define DR_REG_SYSTEM_BASE                      0x600C0000
#define DR_REG_SENSITIVE_BASE                   0x600C1000
#define DR_REG_INTERRUPT_BASE                   0x600C2000
#define DR_REG_EXTMEM_BASE                      0x600C4000
#define DR_REG_MMU_TABLE                        0x600C5000
#define DR_REG_AES_BASE                         0x6003A000
#define DR_REG_SHA_BASE                         0x6003B000
#define DR_REG_RSA_BASE                         0x6003C000
#define DR_REG_HMAC_BASE                        0x6003E000
#define DR_REG_DIGITAL_SIGNATURE_BASE           0x6003D000
#define DR_REG_GDMA_BASE                        0x6003F000
#define DR_REG_ASSIST_DEBUG_BASE                0x600CE000
#define DR_REG_DEDICATED_GPIO_BASE              0x600CF000
#define DR_REG_WORLD_CNTL_BASE                  0x600D0000
#define DR_REG_DPORT_END                        0x600d3FFC
#define DR_REG_UART_BASE                        0x60000000
#define DR_REG_SPI1_BASE                        0x60002000
#define DR_REG_SPI0_BASE                        0x60003000
#define DR_REG_GPIO_BASE                        0x60004000
#define DR_REG_FE2_BASE                         0x60005000
#define DR_REG_FE_BASE                          0x60006000
#define DR_REG_RTCCNTL_BASE                     0x60008000
#define DR_REG_IO_MUX_BASE                      0x60009000
#define DR_REG_UHCI1_BASE                       0x6000C000
#define DR_REG_RTC_I2C_BASE                     0x6000E000
#define DR_REG_UART1_BASE                       0x60010000
#define DR_REG_I2C_EXT_BASE                     0x60013000
#define DR_REG_UHCI0_BASE                       0x60014000
#define DR_REG_RMT_BASE                         0x60016000
#define DR_REG_LEDC_BASE                        0x60019000
#define DR_REG_EFUSE_BASE                       0x60008800
#define DR_REG_NRX_BASE                         0x6001CC00
#define DR_REG_BB_BASE                          0x6001D000
#define DR_REG_TIMERGROUP0_BASE                 0x6001F000
#define DR_REG_TIMERGROUP1_BASE                 0x60020000
#define DR_REG_SYSTIMER_BASE                    0x60023000
#define DR_REG_SPI2_BASE                        0x60024000
#define DR_REG_SYSCON_BASE                      0x60026000
#define DR_REG_APB_CTRL_BASE                    0x60026000    /* Old name for SYSCON, to be removed */
#define DR_REG_TWAI_BASE                        0x6002B000
#define DR_REG_I2S0_BASE                        0x6002D000
#define DR_REG_WIFI_BASE                        0x60033000
#define DR_REG_PHYA_BASE                        0x60034000
#define DR_REG_PWR_MANAGER_BASE                 0x60035000
#define DR_REG_APB_SARADC_BASE                  0x60040000
#define DR_REG_USB_SERIAL_JTAG_BASE             0x60043000
#define DR_REG_AES_XTS_BASE                     0x600CC000
#define DR_REG_EMAC_BASE                        0x600CD000

#define ESP32C3_IO_START_ADDR                   (DR_REG_UART_BASE)
#define ESP32C3_UART_COUNT                      2