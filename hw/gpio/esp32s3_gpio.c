/*
 * ESP32-S3 GPIO emulation
 *
 * Copyright (c) 2023 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "ui/console.h"
#include "ui/console-priv.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/registerfields.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/gpio/esp32s3_gpio.h"
#include "sysemu/runstate.h"
#include "hw/misc/esp32s3_reg.h"
#include "hw/misc/esp32s3_rtc_cntl.h"
#include "exec/address-spaces.h"


struct gpio_matrix_t {
    int         num;
    const char* in;
    const char* out;
    bool        iomux;
};

#define N_GPIOS 49
#define DIRECT_GPIO 1
#define CONSOLE_WIDTH 320
#define CONSOLE_HEIGHT (16*(N_GPIOS+1))
#define N_RTC_GPIOS 22

static const struct gpio_matrix_t gpio_matrix[] = {
    { 0,   "SPIQ_in",                "SPIQ_out",                true  },
    { 1,   "SPID_in",                "SPID_out",                true  },
    { 2,   "SPIHD_in",               "SPIHD_out",               true  },
    { 3,   "SPIWP_in",               "SPIWP_out",               true  },
    { 4,   "",                     "SPICLK_out_mux",          true  },
    { 5,   "",                     "SPICS0_out",              true  },
    { 6,   "",                     "SPICS1_out",              true  },
    { 7,   "SPID4_in",               "SPID4_out",               true  },
    { 8,   "SPID5_in",               "SPID5_out",               true  },
    { 9,   "SPID6_in",               "SPID6_out",               true  },
    { 10,  "SPID7_in",               "SPID7_out",               true  },
    { 11,  "SPIDQS_in",              "SPIDQS_out",              true  },

    { 12,  "U0RXD_in",               "U0TXD_out",               true  },
    { 13,  "U0CTS_in",               "U0RTS_out",               true  },
    { 14,  "U0DSR_in",               "U0DTR_out",               false },
    { 15,  "U1RXD_in",               "U1TXD_out",               true  },
    { 16,  "U1CTS_in",               "U1RTS_out",               true  },
    { 17,  "U1DSR_in",               "U1DTR_out",               false },
    { 18,  "U2RXD_in",               "U2TXD_out",               false },
    { 19,  "U2CTS_in",               "U2RTS_out",               false },
    { 20,  "U2DSR_in",               "U2DTR_out",               false },

    { 21,  "I2S1_MCLK_in",            "I2S1_MCLK_out",           false },
    { 22,  "I2S0O_BCK_in",            "I2S0O_BCK_out",           false },
    { 23,  "I2S0_MCLK_in",            "I2S0_MCLK_out",           false },
    { 24,  "I2S0O_WS_in",             "I2S0O_WS_out",            false },
    { 25,  "I2S0I_SD_in",             "I2S0O_SD_out",            false },
    { 26,  "I2S0I_BCK_in",            "I2S0I_BCK_out",           false },
    { 27,  "I2S0I_WS_in",             "I2S0I_WS_out",            false },
    { 28,  "I2S1O_BCK_in",            "I2S1O_BCK_out",           false },
    { 29,  "I2S1O_WS_in",             "I2S1O_WS_out",            false },
    { 30,  "I2S1I_SD_in",             "I2S1O_SD_out",            false },
    { 31,  "I2S1I_BCK_in",            "I2S1I_BCK_out",           false },
    { 32,  "I2S1I_WS_in",             "I2S1I_WS_out",            false },

    { 33,  "pcnt_sig_ch0_in0",        "",                     false },
    { 34,  "pcnt_sig_ch1_in0",        "",                     false },
    { 35,  "pcnt_ctrl_ch0_in0",       "",                     false },
    { 36,  "pcnt_ctrl_ch1_in0",       "",                     false },
    { 37,  "pcnt_sig_ch0_in1",        "",                     false },
    { 38,  "pcnt_sig_ch1_in1",        "",                     false },
    { 39,  "pcnt_ctrl_ch0_in1",       "",                     false },
    { 40,  "pcnt_ctrl_ch1_in1",       "",                     false },
    { 41,  "pcnt_sig_ch0_in2",        "",                     false },
    { 42,  "pcnt_sig_ch1_in2",        "",                     false },
    { 43,  "pcnt_ctrl_ch0_in2",       "",                     false },
    { 44,  "pcnt_ctrl_ch1_in2",       "",                     false },
    { 45,  "pcnt_sig_ch0_in3",        "",                     false },
    { 46,  "pcnt_sig_ch1_in3",        "",                     false },
    { 47,  "pcnt_ctrl_ch0_in3",       "",                     false },
    { 48,  "pcnt_ctrl_ch1_in3",       "",                     false },

    { 49,  "",                     "",                     false },
    { 50,  "",                     "",                     false },
    { 51,  "I2S0I_SD1_in",            "",                     false },
    { 52,  "I2S0I_SD2_in",            "",                     false },
    { 53,  "I2S0I_SD3_in",            "",                     false },
    { 54,  "Core1_gpio_in7",          "Core1_gpio_out7",        false },
    { 55,  "",                     "",                     false },
    { 56,  "",                     "",                     false },
    { 57,  "",                     "",                     false },
    { 58,  "usb_otg_iddig_in",        "",                     false },
    { 59,  "usb_otg_avalid_in",       "",                     false },
    { 60,  "usb_srp_bvalid_in",       "usb_otg_idpullup",       false },
    { 61,  "usb_otg_vbusvalid_in",    "usb_otg_dppulldown",     false },
    { 62,  "usb_srp_sessend_in",      "usb_otg_dmpulldown",     false },
    { 63,  "",                     "usb_otg_drvvbus",        false },
    { 64,  "",                     "usb_srp_chrgvbus",       false },
    { 65,  "",                     "usb_srp_dischrgvbus",    false },

    { 66,  "SPI3_CLK_in",             "SPI3_CLK_out_mux",       false },
    { 67,  "SPI3_Q_in",               "SPI3_Q_out",             false },
    { 68,  "SPI3_D_in",               "SPI3_D_out",             false },
    { 69,  "SPI3_HD_in",              "SPI3_HD_out",            false },
    { 70,  "SPI3_WP_in",              "SPI3_WP_out",            false },
    { 71,  "SPI3_CS0_in",             "SPI3_CS0_out",           false },
    { 72,  "",                     "SPI3_CS1_out",           false },

    { 73,  "ext_adc_start",           "ledc_ls_sig_out0",       false },
    { 74,  "",                     "ledc_ls_sig_out1",       false },
    { 75,  "",                     "ledc_ls_sig_out2",       false },
    { 76,  "",                     "ledc_ls_sig_out3",       false },
    { 77,  "",                     "ledc_ls_sig_out4",       false },
    { 78,  "",                     "ledc_ls_sig_out5",       false },
    { 79,  "",                     "ledc_ls_sig_out6",       false },
    { 80,  "",                     "ledc_ls_sig_out7",       false },

    { 81,  "rmt_sig_in0",             "rmt_sig_out0",           false },
    { 82,  "rmt_sig_in1",             "rmt_sig_out1",           false },
    { 83,  "rmt_sig_in2",             "rmt_sig_out2",           false },
    { 84,  "rmt_sig_in3",             "rmt_sig_out3",           false },

    /* 85–88 unused */
    { 85,  "", "", false }, 
    { 86, "", "", false },
    { 87,  "", "", false }, 
    { 88, "", "", false },

    { 89,  "I2CEXT0_SCL_in",           "I2CEXT0_SCL_out",        false },
    { 90,  "I2CEXT0_SDA_in",           "I2CEXT0_SDA_out",        false },
    { 91,  "I2CEXT1_SCL_in",           "I2CEXT1_SCL_out",        false },
    { 92,  "I2CEXT1_SDA_in",           "I2CEXT1_SDA_out",        false },

    { 93,  "", "gpio_sd0_out", false },
    { 94,  "", "gpio_sd1_out", false },
    { 95,  "", "gpio_sd2_out", false },
    { 96,  "", "gpio_sd3_out", false },
    { 97,  "", "gpio_sd4_out", false },
    { 98,  "", "gpio_sd5_out", false },
    { 99,  "", "gpio_sd6_out", false },
    {100,  "", "gpio_sd7_out", false },

    {101, "FSPICLK_in",  "FSPICLK_out_mux", true },
    {102, "FSPIQ_in",    "FSPIQ_out",       true },
    {103, "FSPID_in",    "FSPID_out",       true },
    {104, "FSPIHD_in",   "FSPIHD_out",      true },
    {105, "FSPIWP_in",   "FSPIWP_out",      true },
    {106, "FSPIIO4_in",  "FSPIIO4_out",     true },
    {107, "FSPIIO5_in",  "FSPIIO5_out",     true },
    {108, "FSPIIO6_in",  "FSPIIO6_out",     true },
    {109, "FSPIIO7_in",  "FSPIIO7_out",     true },
    {110, "FSPICS0_in",  "FSPICS0_out",     true },

    {111, "", "FSPICS1_out", false },
    {112, "", "FSPICS2_out", false },
    {113, "", "FSPICS3_out", false },
    {114, "", "FSPICS4_out", false },
    {115, "", "FSPICS5_out", false },

    {116, "twai_rx", "twai_tx", false },
    {117, "", "twai_bus_off_on", false },
    {118, "", "twai_clkout", false },

    {119, "", "SUBSPICLK_out_mux", false },
    {120, "SUBSPIQ_in",  "SUBSPIQ_out",  true },
    {121, "SUBSPID_in",  "SUBSPID_out",  true },
    {122, "SUBSPIHD_in", "SUBSPIHD_out", true },
    {123, "SUBSPIWP_in", "SUBSPIWP_out", true },
    {124, "", "SUBSPICS0_out", true },
    {125, "", "SUBSPICS1_out", true },
    {126, "", "FSPIDQS_out",  true },
    {127, "", "SPI3_CS2_out", false },
    {128, "",                   "I2S0O_SD1_out",           false },

    {129, "Core1_gpio_in0",        "Core1_gpio_out0",         false },
    {130, "Core1_gpio_in1",        "Core1_gpio_out1",         false },
    {131, "Core1_gpio_in2",        "Core1_gpio_out2",         false },
    {132, "",                   "LCD_CS",                  false },

    {133, "CAM_DATA_in0",          "LCD_DATA_out0",           false },
    {134, "CAM_DATA_in1",          "LCD_DATA_out1",           false },
    {135, "CAM_DATA_in2",          "LCD_DATA_out2",           false },
    {136, "CAM_DATA_in3",          "LCD_DATA_out3",           false },
    {137, "CAM_DATA_in4",          "LCD_DATA_out4",           false },
    {138, "CAM_DATA_in5",          "LCD_DATA_out5",           false },
    {139, "CAM_DATA_in6",          "LCD_DATA_out6",           false },
    {140, "CAM_DATA_in7",          "LCD_DATA_out7",           false },
    {141, "CAM_DATA_in8",          "LCD_DATA_out8",           false },
    {142, "CAM_DATA_in9",          "LCD_DATA_out9",           false },
    {143, "CAM_DATA_in10",         "LCD_DATA_out10",          false },
    {144, "CAM_DATA_in11",         "LCD_DATA_out11",          false },
    {145, "CAM_DATA_in12",         "LCD_DATA_out12",          false },
    {146, "CAM_DATA_in13",         "LCD_DATA_out13",          false },
    {147, "CAM_DATA_in14",         "LCD_DATA_out14",          false },
    {148, "CAM_DATA_in15",         "LCD_DATA_out15",          false },

    {149, "CAM_PCLK",              "CAM_CLK",                 false },
    {150, "CAM_H_ENABLE",          "LCD_H_ENABLE",            false },
    {151, "CAM_H_SYNC",            "LCD_H_SYNC",              false },
    {152, "CAM_V_SYNC",            "LCD_V_SYNC",              false },

    {153, "",                   "LCD_DC",                  false },
    {154, "",                   "LCD_PCLK",                false },

    {155, "SUBSPID4_in",           "SUBSPID4_out",            true  },
    {156, "SUBSPID5_in",           "SUBSPID5_out",            true  },
    {157, "SUBSPID6_in",           "SUBSPID6_out",            true  },
    {158, "SUBSPID7_in",           "SUBSPID7_out",            true  },
    {159, "SUBSPIDQS_in",          "SUBSPIDQS_out",           true  },

    {160, "pwm0_sync0_in",         "pwm0_out0a",              false },
    {161, "pwm0_sync1_in",         "pwm0_out0b",              false },
    {162, "pwm0_sync2_in",         "pwm0_out1a",              false },
    {163, "pwm0_f0_in",            "pwm0_out1b",              false },
    {164, "pwm0_f1_in",            "pwm0_out2a",              false },
    {165, "pwm0_f2_in",            "pwm0_out2b",              false },

    {166, "pwm0_cap0_in",          "pwm1_out0a",              false },
    {167, "pwm0_cap1_in",          "pwm1_out0b",              false },
    {168, "pwm0_cap2_in",          "pwm1_out1a",              false },

    {169, "pwm1_sync0_in",         "pwm1_out1b",              false },
    {170, "pwm1_sync1_in",         "pwm1_out2a",              false },
    {171, "pwm1_sync2_in",         "pwm1_out2b",              false },
    {172, "pwm1_f0_in",            "sdhost_cclk_out_1",       false },
    {173, "pwm1_f1_in",            "sdhost_cclk_out_2",       false },
    {174, "pwm1_f2_in",            "sdhost_rst_n_1",          false },
    {175, "pwm1_cap0_in",          "sdhost_rst_n_2",          false },

    {176, "pwm1_cap1_in",          "sdhost_ccmd_od_pullup_en_n", false },
    {177, "pwm1_cap2_in",          "sdio_tohost_int_out",     false },

    {178, "sdhost_ccmd_in_1",      "sdhost_ccmd_out_1",       false },
    {179, "sdhost_ccmd_in_2",      "sdhost_ccmd_out_2",       false },
    {180, "sdhost_cdata_in_10",    "sdhost_cdata_out_10",     false },
    {181, "sdhost_cdata_in_11",    "sdhost_cdata_out_11",     false },
    {182, "sdhost_cdata_in_12",    "sdhost_cdata_out_12",     false },
    {183, "sdhost_cdata_in_13",    "sdhost_cdata_out_13",     false },
    {184, "sdhost_cdata_in_14",    "sdhost_cdata_out_14",     false },
    {185, "sdhost_cdata_in_15",    "sdhost_cdata_out_15",     false },

    {186, "sdhost_cdata_in_16",    "sdhost_cdata_out_16",     false },
    {187, "sdhost_cdata_in_17",    "sdhost_cdata_out_17",     false },

    {188, "",                   "",                      false },
    {189, "",                   "",                      false },
    {190, "",                   "",                      false },
    {191, "",                   "",                      false },

    {192, "sdhost_data_strobe_1",  "",                      false },
    {193, "sdhost_data_strobe_2",  "",                      false },
    {194, "sdhost_card_detect_n_1","",                      false },
    {195, "sdhost_card_detect_n_2","",                      false },
    {196, "sdhost_card_write_prt_1","",                     false },
    {197, "sdhost_card_write_prt_2","",                     false },
    {198, "sdhost_card_int_n_1",   "",                      false },
    {199, "sdhost_card_int_n_2",   "",                      false },

    {200, "",                   "",                      false },
    {201, "",                   "",                      false },
    {202, "",                   "",                      false },
    {203, "",                   "",                      false },
    {204, "",                   "",                      false },
    {205, "",                   "",                      false },
    {206, "",                   "",                      false },
    {207, "",                   "",                      false },

    {208, "sig_in_func_208",       "sig_in_func208",          false },
    {209, "sig_in_func_209",       "sig_in_func209",          false },
    {210, "sig_in_func_210",       "sig_in_func210",          false },
    {211, "sig_in_func_211",       "sig_in_func211",          false },
    {212, "sig_in_func_212",       "sig_in_func212",          false },

    {213, "sdhost_cdata_in_20",    "sdhost_cdata_out_20",     false },
    {214, "sdhost_cdata_in_21",    "sdhost_cdata_out_21",     false },
    {215, "sdhost_cdata_in_22",    "sdhost_cdata_out_22",     false },
    {216, "sdhost_cdata_in_23",    "sdhost_cdata_out_23",     false },
    {217, "sdhost_cdata_in_24",    "sdhost_cdata_out_24",     false },
    {218, "sdhost_cdata_in_25",    "sdhost_cdata_out_25",     false },
    {219, "sdhost_cdata_in_26",    "sdhost_cdata_out_26",     false },
    {220, "sdhost_cdata_in_27",    "sdhost_cdata_out_27",     false },

    {221, "pro_alonegpio_in0",     "pro_alonegpio_out0",      false },
    {222, "pro_alonegpio_in1",     "pro_alonegpio_out1",      false },
    {223, "pro_alonegpio_in2",     "pro_alonegpio_out2",      false },
    {224, "pro_alonegpio_in3",     "pro_alonegpio_out3",      false },
    {225, "pro_alonegpio_in4",     "pro_alonegpio_out4",      false },
    {226, "pro_alonegpio_in5",     "pro_alonegpio_out5",      false },
    {227, "pro_alonegpio_in6",     "pro_alonegpio_out6",      false },
    {228, "pro_alonegpio_in7",     "pro_alonegpio_out7",      false },

    {229, "",                   "",                      false },
    {230, "",                   "",                      false },
    {231, "",                   "",                      false },
    {232, "",                   "",                      false },
    {233, "",                   "",                      false },
    {234, "",                   "",                      false },
    {235, "",                   "",                      false },
    {236, "",                   "",                      false },
    {237, "",                   "",                      false },
    {238, "",                   "",                      false },
    {239, "",                   "",                      false },

    {240, "",                   "",                      false },
    {241, "",                   "",                      false },
    {242, "",                   "",                      false },
    {243, "",                   "",                      false },
    {244, "",                   "",                      false },
    {245, "",                   "",                      false },
    {246, "",                   "",                      false },
    {247, "",                   "",                      false },
    {248, "",                   "",                      false },
    {249, "",                   "",                      false },
    {250, "",                   "",                      false },

    {251, "usb_jtag_tdo_bridge",   "usb_jtag_trst",           false },

    {252, "Core1_gpio_in3",        "Core1_gpio_out3",         false },
    {253, "Core1_gpio_in4",        "Core1_gpio_out4",         false },
    {254, "Core1_gpio_in5",        "Core1_gpio_out5",         false },
    {255, "Core1_gpio_in6",        "Core1_gpio_out6",         false },
    { -1, "", "", false }
};

struct pin_mux {
    int         pinnum;
    int         reset;
    const char* pinname;
    const char* functions[5];
    
};

static const struct pin_mux io_mux_pins[] = {
    { 0, 3, "GPIO0",  { "GPIO0",  "GPIO0",  "",        "",        "" } },
    { 1, 1, "GPIO1",  { "GPIO1",  "GPIO1",  "",        "",        "" } },
    { 2, 1, "GPIO2",  { "GPIO2",  "GPIO2",  "",        "",        "" } },
    { 3, 1, "GPIO3",  { "GPIO3",  "GPIO3",  "",        "",        "" } },
    { 4, 0, "GPIO4",  { "GPIO4",  "GPIO4",  "",        "",        "" } },
    { 5, 0, "GPIO5",  { "GPIO5",  "GPIO5",  "",        "",        "" } },
    { 6, 0, "GPIO6",  { "GPIO6",  "GPIO6",  "",        "",        "" } },
    { 7, 0, "GPIO7",  { "GPIO7",  "GPIO7",  "",        "",        "" } },

    { 8, 0, "GPIO8",  { "GPIO8",  "GPIO8",  "",        "SUBSPICS1", "" } },
    { 9, 1, "GPIO9",  { "GPIO9",  "GPIO9",  "",        "SUBSPIHD",  "FSPIHD" } },
    { 10, 1,"GPIO10", { "GPIO10", "GPIO10", "FSPIIO4",   "SUBSPICS0", "FSPICS0" } },
    { 11, 1,"GPIO11", { "GPIO11", "GPIO11", "FSPIIO5",   "SUBSPID",   "FSPID" } },
    { 12, 1,"GPIO12", { "GPIO12", "GPIO12", "FSPIIO6",   "SUBSPICLK", "FSPICLK" } },
    { 13, 1,"GPIO13", { "GPIO13", "GPIO13", "FSPIIO7",   "SUBSPIQ",   "FSPIQ" } },
    { 14, 1,"GPIO14", { "GPIO14", "GPIO14", "FSPIDQS",   "SUBSPIWP",  "FSPIWP" } },

    { 15, 0,"XTAL_32K_P", { "GPIO15", "GPIO15", "U0RTS", "", "" } },
    { 16, 0,"XTAL_32K_N", { "GPIO16", "GPIO16", "U0CTS", "", "" } },

    { 17, 1,"GPIO17", { "GPIO17", "GPIO17", "U1TXD",  "", "" } },
    { 18, 1,"GPIO18", { "GPIO18", "GPIO18", "U1RXD",  "CLK_OUT3", "" } },
    { 19, 0,"GPIO19", { "GPIO19", "GPIO19", "U1RTS",  "CLK_OUT2", "" } },
    { 20, 0,"GPIO20", { "GPIO20", "GPIO20", "U1CTS",  "CLK_OUT1", "" } },
    { 21, 0,"GPIO21", { "GPIO21", "GPIO21", "",     "", "" } },

    { 22, 0,"", { "", "" , "", "", "" } },
    { 23, 0,"", { "", "" , "", "", "" } },
    { 24, 0,"", { "", "" , "", "", "" } },
    { 25, 0,"", { "", "" , "", "", "" } },

    { 26, 3,"SPICS1", { "SPICS1", "GPIO26", "", "", "" } },
    { 27, 3,"SPIHD",  { "SPIHD",  "GPIO27", "", "", "" } },
    { 28, 3,"SPIWP",  { "SPIWP",  "GPIO28", "", "", "" } },
    { 29, 3,"SPICS0", { "SPICS0", "GPIO29", "", "", "" } },
    { 30, 3,"SPICLK", { "SPICLK", "GPIO30", "", "", "" } },
    { 31, 3,"SPIQ",   { "SPIQ",   "GPIO31", "", "", "" } },
    { 32, 3,"SPID",   { "SPID",   "GPIO32", "", "", "" } },

    { 33, 1,"GPIO33", { "GPIO33", "GPIO33", "FSPIHD",  "SUBSPIHD",  "SPIIO4" } },
    { 34, 1,"GPIO34", { "GPIO34", "GPIO34", "FSPICS0", "SUBSPICS0", "SPIIO5" } },
    { 35, 1,"GPIO35", { "GPIO35", "GPIO35", "FSPID",   "SUBSPID",   "SPIIO6" } },
    { 36, 1,"GPIO36", { "GPIO36", "GPIO36", "FSPICLK", "SUBSPICLK", "SPIIO7" } },
    { 37, 1,"GPIO37", { "GPIO37", "GPIO37", "FSPIQ",   "SUBSPIQ",   "SPIDQS" } },
    { 38, 1,"GPIO38", { "GPIO38", "GPIO38", "FSPIWP",  "SUBSPIWP",  "" } },

    { 39, 1,"MTCK", { "MTCK", "GPIO39", "CLK_OUT3", "SUBSPICS1", "" } },
    { 40, 1,"MTDO", { "MTDO", "GPIO40", "CLK_OUT2", "", "" } },
    { 41, 1,"MTDI", { "MTDI", "GPIO41", "CLK_OUT1", "", "" } },
    { 42, 1,"MTMS", { "MTMS", "GPIO42", "", "", "" } },

    { 43, 4,"U0TXD", { "U0TXD", "GPIO43", "CLK_OUT1", "", "" } },
    { 44, 3,"U0RXD", { "U0RXD", "GPIO44", "CLK_OUT2", "", "" } },

    { 45, 2,"GPIO45", { "GPIO45", "GPIO45", "", "", "" } },
    { 46, 2,"GPIO46", { "GPIO46", "GPIO46", "", "", "" } },

    { 47, 1,"SPICLK_P", { "SPICLK_P_DIFF", "GPIO47", "SUBSPICLK_P_DIFF", "", "" } },
    { 48, 1,"SPICLK_N", { "SPICLK_N_DIFF", "GPIO48", "SUBSPICLK_N_DIFF", "", "" } },
    { -1, -1,"", { "" } },
};

static const char *int_types[8]={"No IRQ","IRQ Rising","IRQ Falling","IRQ Any","IRQ Low","IRQ High","",""} ;

static void draw_char_x_y(DisplaySurface *surface, int x, int y, unsigned char c, pixman_color_t fgcol) {
    if(x>320/8) return;
    static pixman_image_t *glyphs[256];
    static pixman_color_t bgcol=QEMU_PIXMAN_COLOR_BLACK;
    
    if (!glyphs[c])
            glyphs[c] = qemu_pixman_glyph_from_vgafont(FONT_HEIGHT, vgafont16, c);
    qemu_pixman_glyph_render(glyphs[c], surface->image,
                             &fgcol, &bgcol, x, y, FONT_WIDTH, FONT_HEIGHT);
}
static void draw_string_x_y(DisplaySurface *surface, int x, int y, char *s,  pixman_color_t fgcol) {
    while(*s) {
        draw_char_x_y(surface,x++,y,*s++,fgcol);
    }
}

static void addconnection(char connection[32],const char str[]) {
    if(strlen(connection)>1 && (strlen(str)+strlen(connection))<32) {
        strncat(connection,",",31);
    }
    strncat(connection,str,31);
}
static const pixman_color_t WHITE=QEMU_PIXMAN_COLOR(0xff, 0xff, 0xff);
static const pixman_color_t GREEN=QEMU_PIXMAN_COLOR(0x00, 0xff, 0x00);
static const pixman_color_t GREY=QEMU_PIXMAN_COLOR(0x80, 0x80, 0x80);
static const pixman_color_t YELLOW=QEMU_PIXMAN_COLOR(0xff, 0xff, 0x00);

static void text_console_update(void *obj) {
    ESP32S3GPIOState *s = ESP32S3_GPIO(obj);
    if(!s->redraw || !qemu_console_is_visible(QEMU_CONSOLE(s->con))) return;
    s->redraw=0;

        char connections[N_GPIOS][2][32];
    
    DisplaySurface *surface = qemu_console_surface(QEMU_CONSOLE(s->con));
    /* clear screen */
    int bpp = (surface_bits_per_pixel(surface) + 7) >> 3;
    uint8_t *d1 = surface_data(surface);
    for (int y = 0; y < surface_height(surface); y++) {
        memset(d1, 0x00, surface_width(surface) * bpp);
        d1 += surface_stride(surface);
    }

    char str[128];
    draw_string_x_y(surface, 0,0,(char *)"No I O Connections",YELLOW);
    
    for(int i=0;i<N_GPIOS;i++) {
        connections[i][0][0]=0;
        connections[i][1][0]=0;
        snprintf(str,32,"%2d:",i);
        draw_string_x_y(surface, 0,i+1,str,WHITE);
        int op_en=((i<32)?s->gpio_enable>>i:s->gpio_enable1>>(i-32))&1;
        int in=((i<32)?s->gpio_in>>i:s->gpio_in1>>(i-32))&1;
        int out=((i<32)?s->gpio_out>>i:s->gpio_out1>>(i-32))&1;

        int io_mux=i+1;
        uint32_t out_sel=FIELD_EX32(s->gpio_out_sel[i],GPIO_FUNC_OUT,SEL);
        uint32_t oen_sel=FIELD_EX32(s->gpio_out_sel[i],GPIO_FUNC_OUT,OEN_SEL);
        uint32_t mux_ie=FIELD_EX32(s->iomux_regs[io_mux],IO_MUX,FUN_IE);
        uint32_t mux_func=FIELD_EX32(s->iomux_regs[io_mux],IO_MUX,MCU_SEL);
        uint32_t pullup=FIELD_EX32(s->iomux_regs[io_mux],IO_MUX,FUN_WPU);
        uint32_t pulldown=FIELD_EX32(s->iomux_regs[io_mux],IO_MUX,FUN_WPD);
        uint32_t int_type=FIELD_EX32(s->gpio_pin[i],GPIO_PIN,INT_TYPE);
        uint32_t int_enable=FIELD_EX32(s->gpio_pin[i],GPIO_PIN,INT_ENABLE);

        draw_char_x_y(surface, 3,i+1,in?'1':'0',((oen_sel || mux_ie)&&!op_en)?GREEN:GREY);
        draw_char_x_y(surface, 5,i+1,out?'1':'0',((out_sel==0x100)&&op_en)?GREEN:GREY);

        if(mux_func==DIRECT_GPIO) {
            if(out_sel==0x100 && op_en) {
                snprintf(str,32,"GPIO%2d",i);
                addconnection(connections[i][0],str);
            } else {
                if(out_sel<229 && op_en)
                    addconnection(connections[i][0],gpio_matrix[out_sel].out);
            }
        } else {
            if(mux_func<6 && op_en)
                addconnection(connections[i][0],(char *)io_mux_pins[i].functions[mux_func]);
        }
      //  printf("%d: out_sel=%x iomux=%x pin=%x op_en=%d oen_sel=%d mux_func=%d mix_ie=%d\n",i,s->gpio_out_sel[i],s->iomux_regs[io_mux],s->gpio_pin[i],op_en,oen_sel,mux_func,mux_ie);

        if((oen_sel || mux_ie) && !op_en && mux_func!=DIRECT_GPIO) {
            addconnection(connections[i][1],(char *)io_mux_pins[i].functions[mux_func]);
        }
        if(pullup) addconnection(connections[i][1],"PU");
        if(pulldown) addconnection(connections[i][1],"PD");
        if(int_enable) addconnection(connections[i][1],int_types[int_type]);
    }
    
    int i=0;
    while(gpio_matrix[i].num>=0) {
        int n=gpio_matrix[i].num;
        int in_sel=FIELD_EX32(s->gpio_in_sel[n],GPIO_FUNC_IN,SEL);
        int sig_sel=FIELD_EX32(s->gpio_in_sel[n],GPIO_FUNC_IN,SIG_SEL);
      //  printf("%d %d %x\n",n,in_sel,sig_sel);
        if(in_sel<N_GPIOS /*&& mux_func==2*/) {
            if(sig_sel) {
                snprintf(str,32,"%s",gpio_matrix[i].in);
                addconnection(connections[in_sel][1],str);
            } 
        }
        i++;
    }
    for(i=0;i<N_GPIOS;i++) {
        str[0]=0;
        if(strlen(connections[i][0])!=0) {
            strcat(str," Out:");
            strcat((char *)str,(char *)(connections[i][0]));
        }
        if(strlen(connections[i][1])!=0) {
            strcat(str," In:");
            strcat(str,(char *)(connections[i][1]));
        }
        draw_string_x_y(surface, 6, i+1, str,WHITE);
    }
    dpy_gfx_update_full(QEMU_CONSOLE(s->con));
}
static void text_console_invalidate(void *obj) {
    ESP32S3GPIOState *s = ESP32S3_GPIO(obj);
    s->redraw = 1;
}

static const GraphicHwOps text_console_ops = {
    .invalidate  = text_console_invalidate,
    .gfx_update = text_console_update,
};

static uint64_t esp32_iomux_read(void *opaque, hwaddr addr, unsigned int size) {
    ESP32S3GPIOState *s = ESP32S3_GPIO(opaque);
    int n=addr/4;
    if(n<N_GPIOS+1) {
        return s->iomux_regs[n];
    }
    return 0;
}

static void esp32s3_apply_pull_state(ESP32S3GPIOState *s, int gpio)
{
    if (gpio < 0 || gpio >= N_GPIOS) {
        return;
    }

    bool output = false;
    if (gpio < 32) {
        output = (s->gpio_enable >> gpio) & 1;
    } else {
        output = (s->gpio_enable1 >> (gpio - 32)) & 1;
    }
    if (output) {
        return;
    }

    uint32_t io_mux = s->iomux_regs[gpio + 1];
    bool pullup = FIELD_EX32(io_mux, IO_MUX, FUN_WPU);
    bool pulldown = FIELD_EX32(io_mux, IO_MUX, FUN_WPD);
    if (pullup == pulldown) {
        return;
    }

    int val = pullup ? 1 : 0;
    if (gpio < 32) {
        s->gpio_in = (s->gpio_in & ~(1U << gpio)) | (val << gpio);
    } else {
        int n1 = gpio - 32;
        s->gpio_in1 = (s->gpio_in1 & ~(1U << n1)) | (val << n1);
    }
}

static void esp32_iomux_write(void *opaque, hwaddr addr, uint64_t value,
                             unsigned int size) {
    ESP32S3GPIOState *s = ESP32S3_GPIO(opaque);
    int n=addr/4;
    if(n<N_GPIOS+1) {
        s->iomux_regs[n]=value;
        if (n > 0) {
            esp32s3_apply_pull_state(s, n - 1);
        }
    }
}

static uint64_t ESP32S3_GPIO_read(void *opaque, hwaddr addr, unsigned int size) {
    ESP32S3GPIOState *s = ESP32S3_GPIO(opaque);
    uint64_t r = 0;
    switch (addr) {
        case A_GPIO_OUT:
            r = s->gpio_out;
            break;
        case A_GPIO_ENABLE:
            r = s->gpio_enable;
            break;
        case A_GPIO_OUT1:
            r = s->gpio_out1;
            break;
        case A_GPIO_ENABLE1:
            r = s->gpio_enable1;
            break;
        case A_GPIO_STRAP:
            r = s->strap_mode;
            break;
        case A_GPIO_IN:
            r = s->gpio_in;
            break;
        case A_GPIO_IN1:
            r = s->gpio_in1;
            break;
        case A_GPIO_STATUS:
            r = s->gpio_status;
            break;
        case A_GPIO_STATUS1:
            r = s->gpio_status1;
            break;
        case A_GPIO_CPU_INT:
            r = s->gpio_cpu_int;
            break;
        case A_GPIO_CPU_INT1:
            r = s->gpio_cpu_int1;
            break;
        case A_GPIO_PIN_BASE ... A_GPIO_FUNC_IN_SEL_CFG_BASE-4:
            r = s->gpio_pin[(addr - A_GPIO_PIN_BASE) / 4];
            break;
        case A_GPIO_FUNC_IN_SEL_CFG_BASE ... A_GPIO_FUNC_OUT_SEL_CFG_BASE-4:
            r = s->gpio_in_sel[(addr - A_GPIO_FUNC_IN_SEL_CFG_BASE) / 4];
            break;
        case A_GPIO_FUNC_OUT_SEL_CFG_BASE ... A_GPIO_FUNC_OUT_SEL_CFG_BASE+N_GPIOS*4-4:
            r = s->gpio_out_sel[(addr - A_GPIO_FUNC_OUT_SEL_CFG_BASE) / 4];
            break;
        case A_GPIO_DATE:
            r = 0x1907040;
            break;
        default:
            break;
    }
//    printf("ESP32S3_GPIO_read %lx %lx\n",addr,r);
    return r;
}
static int get_triggering(int int_type, int oldval, int val) {
    switch (int_type) {
        case 1:
            return (val > oldval);
        case 2:
            return (val < oldval);
        case 3:
            return (val != oldval);
        case 4:
            return (val == 0);
        case 5:
            return (val == 1);
    }
    return 0;
}

static void set_gpio(void *opaque, int n, int val) {
    ESP32S3GPIOState *s = ESP32S3_GPIO(opaque);
    if(runstate_get()==RUN_STATE_SUSPENDED) {
        uint32_t wakeup_state,wakeup_conf,ext1_wakeup;
        uint32_t addr=DR_REG_RTCCNTL_BASE+A_RTC_CNTL_WAKEUP_STATE;
        address_space_read(&address_space_memory, addr, MEMTXATTRS_UNSPECIFIED, &wakeup_state, 4);
        addr=DR_REG_RTCCNTL_BASE+A_RTC_CNTL_EXT_WAKEUP_CONF;
        address_space_read(&address_space_memory, addr, MEMTXATTRS_UNSPECIFIED, &wakeup_conf, 4);
        addr=DR_REG_RTCCNTL_BASE+A_RTC_CNTL_EXT_WAKEUP1;
        address_space_read(&address_space_memory, addr, MEMTXATTRS_UNSPECIFIED, &ext1_wakeup, 4);
    //    printf("wakeup_state %d %x %x %x %x\n",n , wakeup_state,s->rtc_ext_wakeup0, wakeup_conf, s->gpio_pin[n]);

        if(FIELD_EX32(wakeup_state,RTC_CNTL_WAKEUP_STATE,WAKEUP_ENA_EXT0)) {
            if(FIELD_EX32(s->rtc_ext_wakeup0,RTC_EXT_WAKEUP0,SEL)==n && val==0) {
                qemu_system_wakeup_request(QEMU_WAKEUP_REASON_OTHER, NULL);
                qemu_set_irq(s->rtc_wakeup,RTC_EXT0_TRIG_EN);
                s->rtc_ext_wakeup0=0;
            }
        }

        if(FIELD_EX32(wakeup_state,RTC_CNTL_WAKEUP_STATE,WAKEUP_ENA_EXT1)) {
            if(((ext1_wakeup>>n) & 1) && val==0) {
                qemu_set_irq(s->rtc_wakeup,RTC_EXT1_TRIG_EN);
            }
        }


        if(FIELD_EX32(wakeup_state,RTC_CNTL_WAKEUP_STATE,WAKEUP_ENA_GPIO) && FIELD_EX32(s->gpio_pin[n],GPIO_PIN,WAKEUP_ENABLE)) {
            int oldval = (n<32?(s->gpio_in >> n):(s->gpio_in1 >> (n-32)))&1;
            int irq=get_triggering(FIELD_EX32(s->gpio_pin[n],GPIO_PIN,INT_TYPE),oldval,val);
    //        printf("wake %x %d\n",irq, oldval);
            if(irq)
                qemu_set_irq(s->rtc_wakeup,RTC_EXT0_TRIG_EN);
        }
    }

    //printf("set_gpio %d %d\n",n,val);
    if (n < 32) {
        int oldval = (s->gpio_in >> n) & 1;
        int int_type = (s->gpio_pin[n] >> 7) & 7;
        s->gpio_in &= ~(1 << n);
        s->gpio_in |= (val << n);
        int irq = get_triggering(int_type, oldval, val);
        if (irq && (s->gpio_pin[n] & (1 << 13))) {  // cpu int enable
            qemu_set_irq(s->irq, 1);
            s->gpio_cpu_int |= (1 << n);
        }
    } else {
        int n1 = n - 32;
        int oldval = (s->gpio_in1 >> n1) & 1;
        int int_type = (s->gpio_pin[n] >> 7) & 7;
        s->gpio_in1 &= ~(1 << n1);
        s->gpio_in1 |= (val << n1);
        int irq = get_triggering(int_type, oldval, val);
        if (irq && (s->gpio_pin[n] & (1 << 13))) {  // pro cpu int enable
            qemu_set_irq(s->irq, 1);
            s->gpio_cpu_int1 |= (1 << n1);
        }
    }
}

static uint64_t esp32_rtc_read(void *opaque, hwaddr addr, unsigned int size) {
    ESP32S3GPIOState *s = ESP32S3_GPIO(opaque);
    uint64_t r=0;
    
    int rtc_in=s->gpio_in&((1<<N_RTC_GPIOS)-1);
    int rtc_out=s->gpio_out&((1<<N_RTC_GPIOS)-1);

    switch (addr) {
        case A_RTC_GPIO_OUT:
            r=rtc_out<<10;
            break;
        case A_RTC_GPIO_IN:
            r=rtc_in<<10;
            break;
        case A_RTC_GPIO_PIN ... A_RTC_GPIO_PIN+17*4:
            r=s->rtc_gpio_pin[(addr-A_RTC_GPIO_PIN)/4];
            break;
        case A_RTC_PAD_CFG ... A_RTC_PAD_CFG+15*4:
            r=s->rtc_pad_cfg[(addr-A_RTC_PAD_CFG)/4];
            break;
        case A_RTC_EXT_WAKEUP0:
            r=s->rtc_ext_wakeup0;
            break;
    }
//    printf("RTC read %lx=%lx\n",addr,r);
    return r;
}

static void esp32_rtc_write(void *opaque, hwaddr addr, uint64_t value,
                             unsigned int size) {
    ESP32S3GPIOState *s = ESP32S3_GPIO(opaque);
//    printf("RTC write %lx %lx\n",addr,value);
    uint32_t oldvalue = s->gpio_out;
    uint32_t gpio_mask=(1<<N_RTC_GPIOS)-1;
    uint32_t rtc_out=s->gpio_out&gpio_mask;
 
    rtc_out<<=10;
    switch (addr) {
        case A_RTC_GPIO_OUT:
            rtc_out = value;
            break;
        case A_RTC_GPIO_OUT_W1TS:
            rtc_out |= value;
            break;
        case A_RTC_GPIO_OUT_W1TC:
            rtc_out &= ~value;
            break;
        case A_RTC_GPIO_ENABLE:
        	s->rtc_gpio_enable = value;
        	break;
        case A_RTC_GPIO_ENABLE_W1TS:
        	s->rtc_gpio_enable |= value;
        	break;
        case A_RTC_GPIO_ENABLE_W1TC:
        	s->rtc_gpio_enable &= ~value;
        	break;
        case A_RTC_GPIO_PIN ... A_RTC_GPIO_PIN+17*4:
            s->rtc_gpio_pin[(addr-A_RTC_GPIO_PIN)/4] = value;
            break;
        case A_RTC_PAD_CFG ... A_RTC_PAD_CFG+15*4:
            s->rtc_pad_cfg[(addr-A_RTC_PAD_CFG)/4] = value;
            break;
        case A_RTC_EXT_WAKEUP0:
            s->rtc_ext_wakeup0 = value;
            break;
            
    }
    

    s->gpio_out &= ~gpio_mask;
    s->gpio_out |= rtc_out>>10;
    if (s->gpio_out != oldvalue) {
        uint32_t diff = (s->gpio_out ^ oldvalue);
        for (int i = 0; i < N_RTC_GPIOS; i++) {
            if ((1 << i) & diff) {
                if(i!=16) {
                    s->redraw=1;
                }
                qemu_set_irq(s->gpios[i], (s->gpio_out & (1 << i)) ? 1 : 0);
            }
        }
    }
}
static void ESP32S3_GPIO_write(void *opaque, hwaddr addr, uint64_t value,
                             unsigned int size) {
    ESP32S3GPIOState *s = ESP32S3_GPIO(opaque);
    int clearirq;
    uint32_t oldvalue,oldvalue1;
    oldvalue = s->gpio_out;
    oldvalue1 = s->gpio_out1;
//    printf("ESP32S3_GPIO_write %lx %lx\n",addr,value);
    switch (addr) {
        case A_GPIO_OUT:
            s->gpio_out = value;
            s->gpio_in = (s->gpio_in & ~s->gpio_enable) | (value & s->gpio_enable);
            break;
        case A_GPIO_OUT_W1TS:
            s->gpio_out |= value;
            s->gpio_in |= value;
            break;
        case A_GPIO_OUT_W1TC:
            s->gpio_out &= ~value;
            s->gpio_in &= ~value;
            break;
        case A_GPIO_OUT1:
            s->gpio_out1 = value;
            s->gpio_in1 = (s->gpio_in1 & ~s->gpio_enable1) | (value & s->gpio_enable1);
            break;
        case A_GPIO_OUT1_W1TS:
            s->gpio_out1 |= value;
            s->gpio_in1 |= value;
            break;
        case A_GPIO_OUT1_W1TC:
            s->gpio_out1 &= ~value;
            s->gpio_in1 &= ~value;
            break;
        case A_GPIO_ENABLE:
        	s->gpio_enable = value;
        	break;
        case A_GPIO_ENABLE1:
        	s->gpio_enable1 = value;
        	break;
        case A_GPIO_ENABLE_W1TS:
        	s->gpio_enable |= value;
        	break;
        case A_GPIO_ENABLE_W1TC:
        	s->gpio_enable &= ~value;
        	break;
        case A_GPIO_ENABLE1_W1TS:
        	s->gpio_enable1 |= value;
        	break;
        case A_GPIO_ENABLE1_W1TC:
        	s->gpio_enable1 &= ~value;
        	break;
        case A_GPIO_STRAP:
            s->strap_mode = value;
            break;
        case A_GPIO_STATUS:
            s->gpio_status = value;
            break;
        case A_GPIO_STATUS_W1TS:
            s->gpio_status |= value;
            break;
        case A_GPIO_STATUS_W1TC:
            clearirq = 1;
            for (int i = 0; i < 32; i++) {
                if ((1 << i) & value) {
                    int int_type = (s->gpio_pin[i] >> 7) & 7;
                    if ((int_type == 4 && !(s->gpio_in & (1 << i))) ||
                        (int_type == 5 && (s->gpio_in & (1 << i))))
                        clearirq = 0;
                }
            }
            if (clearirq) {
                s->gpio_status &= ~value;
                s->gpio_cpu_int &= ~value;
                qemu_set_irq(s->irq, 0);
            }
            break;
        case A_GPIO_STATUS1:
            s->gpio_status1 = value;
            break;
        case A_GPIO_STATUS1_W1TS:
            s->gpio_status1 |= value;
            break;
        case A_GPIO_STATUS1_W1TC:
            clearirq = 1;
            for (int i = 0; i < 17; i++) {
                if ((1 << i) & value) {
                    int int_type = (s->gpio_pin[i + 32] >> 7) & 7;
                    if ((int_type == 4 && !(s->gpio_in1 & (1 << i))) ||
                        (int_type == 5 && (s->gpio_in1 & (1 << i))))
                        clearirq = 0;
                }
            }
            if (clearirq) {
                s->gpio_status1 &= ~value;
                s->gpio_cpu_int1 &= ~value;
                qemu_set_irq(s->irq, 0);
            }
            break;
        case A_GPIO_CPU_INT:
            s->gpio_cpu_int=value;
            break;
        case A_GPIO_CPU_INT1:
            s->gpio_cpu_int1=value;
            break;
        case A_GPIO_PIN_BASE ... A_GPIO_FUNC_IN_SEL_CFG_BASE-4:
            s->gpio_pin[(addr - A_GPIO_PIN_BASE) / 4] = value;
            break;
        case A_GPIO_FUNC_IN_SEL_CFG_BASE ... A_GPIO_FUNC_OUT_SEL_CFG_BASE-4:
            s->gpio_in_sel[(addr - A_GPIO_FUNC_IN_SEL_CFG_BASE)/4] = value;
//            printf("gpio_in_sel %lx %lx\n",(addr - A_GPIO_FUNC_IN_SEL_CFG_BASE)/4,value);
            break;
        case A_GPIO_FUNC_OUT_SEL_CFG_BASE ... A_GPIO_FUNC_OUT_SEL_CFG_BASE+N_GPIOS*4-4:
//            printf("gpio_out_sel %lx %lx\n",(addr - A_GPIO_FUNC_OUT_SEL_CFG_BASE)/4,value);
            s->gpio_out_sel[(addr - A_GPIO_FUNC_OUT_SEL_CFG_BASE)/4] = value;
            break;
    }

    for (int i = 0; i < N_GPIOS; i++) {
        esp32s3_apply_pull_state(s, i);
    }

    if (s->gpio_out != oldvalue) {
        uint32_t diff = (s->gpio_out ^ oldvalue);
        for (int i = 0; i < 32; i++) {
            if ((1 << i) & diff) {
                qemu_set_irq(s->gpios[i], (s->gpio_out & (1 << i)) ? 1 : 0);
            }
        }
    }
    if (s->gpio_out1 != oldvalue1) {
        uint32_t diff = (s->gpio_out1 ^ oldvalue1);
        for (int i = 0; i < 16; i++) {
            if ((1 << i) & diff) {
                qemu_set_irq(s->gpios[i+32], (s->gpio_out1 & (1 << i)) ? 1 : 0);
            }
        }
    }
}

static void func_gpio(void *opaque, int n, int val) {
        ESP32S3GPIOState *s = ESP32S3_GPIO(opaque);
        int func=(val>>1)&0x1ff;
        int v=val & 1;
        int param=((unsigned)val)>>10;
        for(int i=0;i<N_GPIOS;i++) {
                if((s->gpio_out_sel[i] & 0x1ff ) == func) {
                        qemu_set_irq(s->gpios[i], v+(param<<1));
                }
        }
}

static const MemoryRegionOps gpio_ops = {
    .read = ESP32S3_GPIO_read,
    .write = ESP32S3_GPIO_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static const MemoryRegionOps iomux_ops = {
    .read = esp32_iomux_read,
    .write = esp32_iomux_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static const MemoryRegionOps rtc_ops = {
    .read = esp32_rtc_read,
    .write = esp32_rtc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void ESP32S3_GPIO_reset(Object *dev, ResetType type) {
    ESP32S3GPIOState *s = ESP32S3_GPIO(dev);
    for(int i=0;i<256;i++) {
        s->gpio_in_sel[i]=0x3c;
    }
    for(int i=0;i<N_GPIOS;i++) {
        s->gpio_out_sel[i]=0;
    }
    for(int i=0;i<N_GPIOS;i++) {
        uint32_t v=0;
        int reset_type=io_mux_pins[i].reset;
        if(reset_type==1 || reset_type==2 || reset_type==3) {
            v=FIELD_DP32(v,IO_MUX,FUN_IE,1);
        }
        if(reset_type==2) v=FIELD_DP32(v,IO_MUX,FUN_WPD,1);
        if(reset_type==3 || reset_type==4) v=FIELD_DP32(v,IO_MUX,FUN_WPU,1);
        if(reset_type==4) {
            if(i<32) s->gpio_enable=1<<i; else s->gpio_enable1=1<<(i-32);
        }
        // Emulate the external pullup resistors on the GPIO0/GPIO14 buttons of the T-Display-S3
        if (i == 0 || i == 14) v = FIELD_DP32(v, IO_MUX, FUN_WPU, 1);
        s->iomux_regs[i+1]=v;
    }
    for (int i = 0; i < N_GPIOS; i++) {
        esp32s3_apply_pull_state(s, i);
    }
}

static void esp32_gpio_realize(DeviceState *dev, Error **errp) {   
    ESP32S3GPIOState *s = ESP32S3_GPIO(dev);
    s->con=QEMU_CONSOLE(object_new(TYPE_QEMU_FIXED_TEXT_CONSOLE));
    s->con->hw_ops = &text_console_ops;
    s->con->hw = s;
    dpy_gfx_replace_surface(QEMU_CONSOLE(s->con), qemu_create_displaysurface(CONSOLE_WIDTH,CONSOLE_HEIGHT));
}

static void esp32s3_gpio_init(Object *obj)
{
    ESP32S3GPIOState *s = ESP32S3_GPIO(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    /* Set the default value for the property */
    object_property_set_int(obj, "strap_mode", ESP32S3_STRAP_MODE_FLASH_BOOT, &error_fatal);
    memory_region_init_io(&s->iomem, obj, &gpio_ops, s,
                          TYPE_ESP32S3_GPIO, 0x1000);
    memory_region_init_io(&s->iomuxmem, obj, &iomux_ops, s,
                          TYPE_ESP32S3_GPIO, 0x1000);
    memory_region_init_io(&s->iortcmem, obj, &rtc_ops, s,
                          TYPE_ESP32S3_GPIO, 0x100);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_mmio(sbd, &s->iomuxmem);
    sysbus_init_mmio(sbd, &s->iortcmem);
    sysbus_init_irq(sbd, &s->irq);
    qdev_init_gpio_out_named(DEVICE(s), &s->irq, SYSBUS_DEVICE_GPIO_IRQ, 1);
    qdev_init_gpio_out_named(DEVICE(s), s->gpios, ESP32_GPIOS, 49);
    qdev_init_gpio_out_named(DEVICE(s), &s->rtc_wakeup, ESP32_RTCIO_WAKEUP_GPIO, 1);
    qdev_init_gpio_in_named(DEVICE(s), set_gpio, ESP32_GPIOS_IN, 49);
    qdev_init_gpio_in_named(DEVICE(s), func_gpio, ESP32_GPIOS_FUNC,1);

}

static Property ESP32S3_GPIO_properties[] = {
    /* The strap_mode needs to be explicitly set in the instance init, thus, set
     * the default value to 0. */
    DEFINE_PROP_UINT32("strap_mode", ESP32S3GPIOState, strap_mode, 0),
    DEFINE_PROP_END_OF_LIST(),
};

/* If we need to override any function from the parent (reset, realize, ...), it shall be done
 * in this class_init function */
static void esp32s3_gpio_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.enter = ESP32S3_GPIO_reset;
    dc->realize = esp32_gpio_realize;;
    device_class_set_props(dc, ESP32S3_GPIO_properties);
}

static const TypeInfo esp32s3_gpio_info = {
    .name = TYPE_ESP32S3_GPIO,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32S3GPIOState),
    .instance_init = esp32s3_gpio_init,
    .class_init = esp32s3_gpio_class_init,
    .class_size = sizeof(ESP32S3GPIOClass),
};

static void esp32s3_gpio_register_types(void)
{
    type_register_static(&esp32s3_gpio_info);
}

type_init(esp32s3_gpio_register_types)
