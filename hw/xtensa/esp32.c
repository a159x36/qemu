/*
 * ESP32 SoC and machine
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
#include "qemu-common.h"
#include "hw/hw.h"
#include "hw/boards.h"
#include "hw/loader.h"
#include "hw/sysbus.h"
#include "hw/i2c/esp32_i2c.h"
#include "hw/xtensa/xtensa_memory.h"
#include "hw/misc/unimp.h"
#include "hw/irq.h"
#include "hw/i2c/i2c.h"
#include "hw/qdev-properties.h"
#include "hw/xtensa/esp32.h"
#include "sysemu/sysemu.h"
#include "sysemu/reset.h"
#include "sysemu/cpus.h"
#include "sysemu/runstate.h"
#include "sysemu/blockdev.h"
#include "sysemu/block-backend.h"
#include "exec/exec-all.h"
#include "net/net.h"
#include "elf.h"

#define TYPE_ESP32_SOC "xtensa.esp32"
#define ESP32_SOC(obj) OBJECT_CHECK(Esp32SocState, (obj), TYPE_ESP32_SOC)

#define TYPE_ESP32_CPU XTENSA_CPU_TYPE_NAME("esp32")

typedef struct XtensaCPU XtensaCPU;

enum {
    ESP32_MEMREGION_IROM,
    ESP32_MEMREGION_DROM,
    ESP32_MEMREGION_DRAM,
    ESP32_MEMREGION_IRAM,
    ESP32_MEMREGION_ICACHE0,
    ESP32_MEMREGION_ICACHE1,
    ESP32_MEMREGION_RTCSLOW,
    ESP32_MEMREGION_RTCFAST_D,
    ESP32_MEMREGION_RTCFAST_I,
};

static const struct MemmapEntry {
    hwaddr base;
    hwaddr size;
} esp32_memmap[] = {
    [ESP32_MEMREGION_DROM] = { 0x3ff90000, 0x10000 },
    [ESP32_MEMREGION_IROM] = { 0x40000000, 0x70000 },
    [ESP32_MEMREGION_DRAM] = { 0x3ffae000, 0x52000 },
    [ESP32_MEMREGION_IRAM] = { 0x40080000, 0x40000 },
    [ESP32_MEMREGION_ICACHE0] = { 0x40070000, 0x8000 },
    [ESP32_MEMREGION_ICACHE1] = { 0x40078000, 0x8000 },
    [ESP32_MEMREGION_RTCSLOW] = { 0x50000000, 0x2000 },
    [ESP32_MEMREGION_RTCFAST_I] = { 0x400C0000, 0x2000 },
    [ESP32_MEMREGION_RTCFAST_D] = { 0x3ff80000, 0x2000 },
};


#define ESP32_SOC_RESET_PROCPU    0x1
#define ESP32_SOC_RESET_APPCPU    0x2
#define ESP32_SOC_RESET_PERIPH    0x4
#define ESP32_SOC_RESET_DIG       (ESP32_SOC_RESET_PROCPU | ESP32_SOC_RESET_APPCPU | ESP32_SOC_RESET_PERIPH)
#define ESP32_SOC_RESET_RTC       0x8
#define ESP32_SOC_RESET_ALL       (ESP32_SOC_RESET_RTC | ESP32_SOC_RESET_DIG)




static void remove_cpu_watchpoints(XtensaCPU* xcs)
{
    for (int i = 0; i < MAX_NDBREAK; ++i) {
        if (xcs->env.cpu_watchpoint[i]) {
            cpu_watchpoint_remove_by_ref(CPU(xcs), xcs->env.cpu_watchpoint[i]);
            xcs->env.cpu_watchpoint[i] = NULL;
        }
    }
}

static void esp32_dig_reset(void *opaque, int n, int level)
{
    Esp32SocState *s = ESP32_SOC(opaque);
    if (level) {
        esp32_dport_clear_ill_trap_state(&s->dport);
        s->requested_reset = ESP32_SOC_RESET_DIG;
        qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
    }
}

static void esp32_cpu_reset(void* opaque, int n, int level)
{
    Esp32SocState *s = ESP32_SOC(opaque);
    if (level) {
        s->requested_reset = (n == 0) ? ESP32_SOC_RESET_PROCPU : ESP32_SOC_RESET_APPCPU;
        /* Use different cause for APP CPU so that its reset doesn't cause QEMU to exit,
         * when -no-reboot option is given.
         */
        ShutdownCause cause = (n == 0) ? SHUTDOWN_CAUSE_GUEST_RESET : SHUTDOWN_CAUSE_SUBSYSTEM_RESET;
        s->rtc_cntl.reset_cause[n] = ESP32_SW_CPU_RESET;
        qemu_system_reset_request(cause);
    }
}

static void esp32_timg_cpu_reset(void* opaque, int n, int level)
{
    Esp32SocState *s = ESP32_SOC(opaque);
    if (level) {
        s->requested_reset = (n == 0) ? ESP32_SOC_RESET_PROCPU : ESP32_SOC_RESET_APPCPU;
        /* Use different cause for APP CPU so that its reset doesn't cause QEMU to exit,
         * when -no-reboot option is given.
         */
        ShutdownCause cause = (n == 0) ? SHUTDOWN_CAUSE_GUEST_RESET : SHUTDOWN_CAUSE_SUBSYSTEM_RESET;
        s->rtc_cntl.reset_cause[n] = ESP32_TGWDT_CPU_RESET;
        qemu_system_reset_request(cause);
    }
}

static void esp32_timg_sys_reset(void* opaque, int n, int level)
{
    Esp32SocState *s = ESP32_SOC(opaque);
    if (level) {
        esp32_dport_clear_ill_trap_state(&s->dport);
        s->requested_reset = ESP32_SOC_RESET_DIG;
        for (int i = 0; i < ESP32_CPU_COUNT; ++i) {
            s->rtc_cntl.reset_cause[i] = ESP32_TG0WDT_SYS_RESET + n;
        }
        qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
    }
}

static void esp32_soc_reset(DeviceState *dev)
{
    Esp32SocState *s = ESP32_SOC(dev);

    uint32_t strap_mode = s->gpio.strap_mode;

    bool flash_boot_mode = ((strap_mode & 0x10) || (strap_mode & 0x1f) == 0x0c);
    qemu_set_irq(qdev_get_gpio_in_named(DEVICE(&s->flash_enc), ESP32_FLASH_ENCRYPTION_DL_MODE_GPIO, 0), !flash_boot_mode);

    if (s->requested_reset == 0) {
        s->requested_reset = ESP32_SOC_RESET_ALL;
    }
    if (s->requested_reset & ESP32_SOC_RESET_RTC) {
        device_cold_reset(DEVICE(&s->rtc_cntl));
    }
    if (s->requested_reset & ESP32_SOC_RESET_PERIPH) {
        device_cold_reset(DEVICE(&s->dport));
        device_cold_reset(DEVICE(&s->intmatrix));
        device_cold_reset(DEVICE(&s->sha));
        device_cold_reset(DEVICE(&s->rsa));
        device_cold_reset(DEVICE(&s->gpio));
        for (int i = 0; i < ESP32_UART_COUNT; ++i) {
            device_cold_reset(DEVICE(&s->uart));
        }
        for (int i = 0; i < ESP32_FRC_COUNT; ++i) {
            device_cold_reset(DEVICE(&s->frc_timer[i]));
        }
        for (int i = 0; i < ESP32_TIMG_COUNT; ++i) {
            device_cold_reset(DEVICE(&s->timg[i]));
        }
        s->timg[0].flash_boot_mode = flash_boot_mode;
        for (int i = 0; i < ESP32_SPI_COUNT; ++i) {
            device_cold_reset(DEVICE(&s->spi[i]));
        }
        for (int i = 0; i < ESP32_I2C_COUNT; i++) {
            device_cold_reset(DEVICE(&s->i2c[i]));
        }
        device_cold_reset(DEVICE(&s->efuse));
        if (s->eth) {
            device_cold_reset(s->eth);
        }
    }
    if (s->requested_reset & ESP32_SOC_RESET_PROCPU) {
        xtensa_select_static_vectors(&s->cpu[0].env, s->rtc_cntl.stat_vector_sel[0]);
        remove_cpu_watchpoints(&s->cpu[0]);
        cpu_reset(CPU(&s->cpu[0]));
    }
    if (s->requested_reset & ESP32_SOC_RESET_APPCPU) {
        xtensa_select_static_vectors(&s->cpu[1].env, s->rtc_cntl.stat_vector_sel[1]);
        remove_cpu_watchpoints(&s->cpu[1]);
        cpu_reset(CPU(&s->cpu[1]));
    }
    s->requested_reset = 0;
}

static void esp32_cpu_stall(void* opaque, int n, int level)
{
    Esp32SocState *s = ESP32_SOC(opaque);

    bool stall;
    if (n == 0) {
        stall = s->rtc_cntl.cpu_stall_state[0];
    } else {
        stall = s->rtc_cntl.cpu_stall_state[1] || s->dport.appcpu_stall_state;
    }

    if (stall != s->cpu[n].env.runstall) {
        xtensa_runstall(&s->cpu[n].env, stall);
    }
}

static void esp32_clk_update(void* opaque, int n, int level)
{
    Esp32SocState *s = ESP32_SOC(opaque);
    if (!level) {
        return;
    }

    /* APB clock */
    uint32_t apb_clk_freq, cpu_clk_freq;
    if (s->rtc_cntl.soc_clk == ESP32_SOC_CLK_PLL) {
        const uint32_t cpu_clk_mul[] = {1, 2, 3};
        apb_clk_freq = s->rtc_cntl.pll_apb_freq;
        cpu_clk_freq = cpu_clk_mul[s->dport.cpuperiod_sel] * apb_clk_freq;
    } else {
        apb_clk_freq = s->rtc_cntl.xtal_apb_freq;
        cpu_clk_freq = apb_clk_freq;
    }
    qdev_prop_set_int32(DEVICE(&s->frc_timer), "apb_freq", apb_clk_freq);
    qdev_prop_set_int32(DEVICE(&s->timg[0]), "apb_freq", apb_clk_freq);
    qdev_prop_set_int32(DEVICE(&s->timg[1]), "apb_freq", apb_clk_freq);
    *(uint32_t*)(&s->cpu[0].env.config->clock_freq_khz) = cpu_clk_freq / 1000;
}

static void esp32_soc_add_periph_device(MemoryRegion *dest, void* dev, hwaddr dport_base_addr)
{
    MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(dev), 0);
    memory_region_add_subregion_overlap(dest, dport_base_addr, mr, 0);
    MemoryRegion *mr_apb = g_new(MemoryRegion, 1);
    char *name = g_strdup_printf("mr-apb-0x%08x", (uint32_t) dport_base_addr);
    memory_region_init_alias(mr_apb, OBJECT(dev), name, mr, 0, memory_region_size(mr));
    memory_region_add_subregion_overlap(dest, dport_base_addr - DR_REG_DPORT_APB_BASE + APB_REG_BASE, mr_apb, 0);
    g_free(name);
}

static void esp32_soc_add_unimp_device(MemoryRegion *dest, const char* name, hwaddr dport_base_addr, size_t size, uint32_t default_value)
{
    
    create_unimplemented_device(name, dport_base_addr, size, default_value);
    char * name_apb = g_strdup_printf("%s-apb", name);
    create_unimplemented_device(name_apb, dport_base_addr - DR_REG_DPORT_APB_BASE + APB_REG_BASE, size, default_value);
    g_free(name_apb);
    /*
    MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(dev), 0);
    memory_region_add_subregion_overlap(dest, dport_base_addr, mr, 0);
    MemoryRegion *mr_apb = g_new(MemoryRegion, 1);
    char *name = g_strdup_printf("mr-apb-0x%08x", (uint32_t) dport_base_addr);
    memory_region_init_alias(mr_apb, OBJECT(dev), name, mr, 0, memory_region_size(mr));
    memory_region_add_subregion_overlap(dest, dport_base_addr - DR_REG_DPORT_APB_BASE + APB_REG_BASE, mr_apb, 0);
    g_free(name);
    */
}

static void esp32_soc_realize(DeviceState *dev, Error **errp)
{
    Esp32SocState *s = ESP32_SOC(dev);
    MachineState *ms = MACHINE(qdev_get_machine());

    const struct MemmapEntry *memmap = esp32_memmap;
    MemoryRegion *sys_mem = get_system_memory();

    MemoryRegion *dram = g_new(MemoryRegion, 1);
    MemoryRegion *iram = g_new(MemoryRegion, 1);
    MemoryRegion *drom = g_new(MemoryRegion, 1);
    MemoryRegion *irom = g_new(MemoryRegion, 1);
    MemoryRegion *icache0 = g_new(MemoryRegion, 1);
    MemoryRegion *icache1 = g_new(MemoryRegion, 1);
    MemoryRegion *rtcslow = g_new(MemoryRegion, 1);
    MemoryRegion *rtcfast_i = g_new(MemoryRegion, 1);
    MemoryRegion *rtcfast_d = g_new(MemoryRegion, 1);

    memory_region_init_rom(irom, NULL, "esp32.irom",
                           memmap[ESP32_MEMREGION_IROM].size, &error_fatal);
    memory_region_add_subregion(sys_mem, memmap[ESP32_MEMREGION_IROM].base, irom);

    memory_region_init_alias(drom, NULL, "esp32.drom", irom, 0x60000, memmap[ESP32_MEMREGION_DROM].size);
    memory_region_add_subregion(sys_mem, memmap[ESP32_MEMREGION_DROM].base, drom);

    memory_region_init_ram(dram, NULL, "esp32.dram",
                           memmap[ESP32_MEMREGION_DRAM].size, &error_fatal);
    memory_region_add_subregion(sys_mem, memmap[ESP32_MEMREGION_DRAM].base, dram);

    memory_region_init_ram(iram, NULL, "esp32.iram",
                           memmap[ESP32_MEMREGION_IRAM].size, &error_fatal);
    memory_region_add_subregion(sys_mem, memmap[ESP32_MEMREGION_IRAM].base, iram);

    memory_region_init_ram(icache0, NULL, "esp32.icache0",
                           memmap[ESP32_MEMREGION_ICACHE0].size, &error_fatal);
    memory_region_add_subregion(sys_mem, memmap[ESP32_MEMREGION_ICACHE0].base, icache0);

    memory_region_init_ram(icache1, NULL, "esp32.icache1",
                           memmap[ESP32_MEMREGION_ICACHE1].size, &error_fatal);
    memory_region_add_subregion(sys_mem, memmap[ESP32_MEMREGION_ICACHE1].base, icache1);

    memory_region_init_ram(rtcslow, NULL, "esp32.rtcslow",
                           memmap[ESP32_MEMREGION_RTCSLOW].size, &error_fatal);
    memory_region_add_subregion(sys_mem, memmap[ESP32_MEMREGION_RTCSLOW].base, rtcslow);

    /* RTC Fast memory is only accessible by the PRO CPU */

    memory_region_init_ram(rtcfast_i, NULL, "esp32.rtcfast_i",
                           memmap[ESP32_MEMREGION_RTCSLOW].size, &error_fatal);
    memory_region_add_subregion(&s->cpu_specific_mem[0], memmap[ESP32_MEMREGION_RTCFAST_I].base, rtcfast_i);

    memory_region_init_alias(rtcfast_d, NULL, "esp32.rtcfast_d", rtcfast_i, 0, memmap[ESP32_MEMREGION_RTCFAST_D].size);
    memory_region_add_subregion(&s->cpu_specific_mem[0], memmap[ESP32_MEMREGION_RTCFAST_D].base, rtcfast_d);

    for (int i = 0; i < ms->smp.cpus; ++i) {
        qdev_realize(DEVICE(&s->cpu[i]), NULL, &error_fatal);
    }

    qdev_realize(DEVICE(&s->dport), &s->periph_bus, &error_fatal);
    MemoryRegion* dport_mem = sysbus_mmio_get_region(SYS_BUS_DEVICE(&s->dport), 0);

    memory_region_add_subregion(sys_mem, DR_REG_DPORT_BASE, dport_mem);
    qdev_connect_gpio_out_named(DEVICE(&s->dport), ESP32_DPORT_APPCPU_RESET_GPIO, 0,
                                qdev_get_gpio_in_named(dev, ESP32_RTC_CPU_RESET_GPIO, 1));
    qdev_connect_gpio_out_named(DEVICE(&s->dport), ESP32_DPORT_APPCPU_STALL_GPIO, 0,
                                qdev_get_gpio_in_named(dev, ESP32_RTC_CPU_STALL_GPIO, 1));
    qdev_connect_gpio_out_named(DEVICE(&s->rtc_cntl), ESP32_DPORT_CLK_UPDATE_GPIO, 0,
                                qdev_get_gpio_in_named(dev, ESP32_RTC_CLK_UPDATE_GPIO, 0));

    for (int i = 0; i < ESP32_CPU_COUNT; ++i) {
        char name[16];
        snprintf(name, sizeof(name), "cpu%d", i);
        object_property_set_link(OBJECT(&s->intmatrix), name, OBJECT(qemu_get_cpu(i)), &error_abort);
    }
    qdev_realize(DEVICE(&s->intmatrix), &s->periph_bus, &error_fatal);
    DeviceState* intmatrix_dev = DEVICE(&s->intmatrix);
    memory_region_add_subregion_overlap(dport_mem, ESP32_DPORT_PRO_INTMATRIX_BASE, sysbus_mmio_get_region(SYS_BUS_DEVICE(&s->intmatrix), 0), -1);

    if (s->dport.flash_blk) {
        for (int i = 0; i < ESP32_CPU_COUNT; ++i) {
            Esp32CacheRegionState *drom0 = &s->dport.cache_state[i].drom0;
            memory_region_add_subregion_overlap(&s->cpu_specific_mem[i], drom0->base, &drom0->illegal_access_trap_mem, -2);
            memory_region_add_subregion_overlap(&s->cpu_specific_mem[i], drom0->base, &drom0->mem, -1);
            Esp32CacheRegionState *iram0 = &s->dport.cache_state[i].iram0;
            memory_region_add_subregion_overlap(&s->cpu_specific_mem[i], iram0->base, &iram0->illegal_access_trap_mem, -2);
            memory_region_add_subregion_overlap(&s->cpu_specific_mem[i], iram0->base, &iram0->mem, -1);
        }
        qdev_connect_gpio_out_named(DEVICE(&s->dport), ESP32_DPORT_CACHE_ILL_IRQ_GPIO, 0,
                                    qdev_get_gpio_in(DEVICE(&s->intmatrix), ETS_CACHE_IA_INTR_SOURCE));
    }

    int n_crosscore_irqs = ESP32_DPORT_CROSSCORE_INT_COUNT;
    object_property_set_int(OBJECT(&s->crosscore_int), "n_irqs", n_crosscore_irqs, &error_abort);
    qdev_realize(DEVICE(&s->crosscore_int), &s->periph_bus, &error_fatal);
    memory_region_add_subregion_overlap(dport_mem, ESP32_DPORT_CROSSCORE_INT_BASE, &s->crosscore_int.iomem, -1);

    for (int index = 0; index < ESP32_DPORT_CROSSCORE_INT_COUNT; ++index) {
        qemu_irq target = qdev_get_gpio_in(DEVICE(&s->intmatrix), ETS_FROM_CPU_INTR0_SOURCE + index);
        assert(target);
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->crosscore_int), index, target);
    }

    qdev_realize(DEVICE(&s->rsa), &s->periph_bus, &error_fatal);
    esp32_soc_add_periph_device(sys_mem, &s->rsa, DR_REG_RSA_BASE);

    qdev_realize(DEVICE(&s->sha), &s->periph_bus, &error_fatal);
    esp32_soc_add_periph_device(sys_mem, &s->sha, DR_REG_SHA_BASE);

    qdev_realize(DEVICE(&s->rtc_cntl), &s->rtc_bus, &error_fatal);
    esp32_soc_add_periph_device(sys_mem, &s->rtc_cntl, DR_REG_RTCCNTL_BASE);

    qdev_connect_gpio_out_named(DEVICE(&s->rtc_cntl), ESP32_RTC_DIG_RESET_GPIO, 0,
                                qdev_get_gpio_in_named(dev, ESP32_RTC_DIG_RESET_GPIO, 0));
    qdev_connect_gpio_out_named(DEVICE(&s->rtc_cntl), ESP32_RTC_CLK_UPDATE_GPIO, 0,
                                qdev_get_gpio_in_named(dev, ESP32_RTC_CLK_UPDATE_GPIO, 0));
    for (int i = 0; i < ms->smp.cpus; ++i) {
        qdev_connect_gpio_out_named(DEVICE(&s->rtc_cntl), ESP32_RTC_CPU_RESET_GPIO, i,
                                    qdev_get_gpio_in_named(dev, ESP32_RTC_CPU_RESET_GPIO, i));
        qdev_connect_gpio_out_named(DEVICE(&s->rtc_cntl), ESP32_RTC_CPU_STALL_GPIO, i,
                                    qdev_get_gpio_in_named(dev, ESP32_RTC_CPU_STALL_GPIO, i));
    }

    qdev_realize(DEVICE(&s->gpio), &s->periph_bus, &error_fatal);
    esp32_soc_add_periph_device(sys_mem, &s->gpio, DR_REG_GPIO_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->gpio),0,qdev_get_gpio_in(intmatrix_dev, ETS_GPIO_INTR_SOURCE));

    for (int i = 0; i < ESP32_UART_COUNT; ++i) {
        const hwaddr uart_base[] = {DR_REG_UART_BASE, DR_REG_UART1_BASE, DR_REG_UART2_BASE};
        qdev_realize(DEVICE(&s->uart[i]), &s->periph_bus, &error_fatal);
        esp32_soc_add_periph_device(sys_mem, &s->uart[i], uart_base[i]);
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->uart[i]), 0,
                           qdev_get_gpio_in(intmatrix_dev, ETS_UART0_INTR_SOURCE + i));
    }

    for (int i = 0; i < ESP32_FRC_COUNT; ++i) {
        qdev_realize(DEVICE(&s->frc_timer[i]), &s->periph_bus, &error_fatal);

        esp32_soc_add_periph_device(sys_mem, &s->frc_timer[i], DR_REG_FRC_TIMER_BASE + i * ESP32_FRC_TIMER_STRIDE);

        sysbus_connect_irq(SYS_BUS_DEVICE(&s->frc_timer[i]), 0,
                           qdev_get_gpio_in(intmatrix_dev, ETS_TIMER1_INTR_SOURCE + i));
    }

    for (int i = 0; i < ESP32_TIMG_COUNT; ++i) {
        s->timg[i].id = i;

        const hwaddr timg_base[] = {DR_REG_TIMERGROUP0_BASE, DR_REG_TIMERGROUP1_BASE};
        qdev_realize(DEVICE(&s->timg[i]), &s->periph_bus, &error_fatal);

        esp32_soc_add_periph_device(sys_mem, &s->timg[i], timg_base[i]);

        int timg_level_int[] = { ETS_TG0_T0_LEVEL_INTR_SOURCE, ETS_TG1_T0_LEVEL_INTR_SOURCE };
        int timg_edge_int[] = { ETS_TG0_T0_EDGE_INTR_SOURCE, ETS_TG1_T0_EDGE_INTR_SOURCE };
        for (Esp32TimgInterruptType it = TIMG_T0_INT; it < TIMG_INT_MAX; ++it) {
            sysbus_connect_irq(SYS_BUS_DEVICE(&s->timg[i]), it, qdev_get_gpio_in(intmatrix_dev, timg_level_int[i] + it));
            sysbus_connect_irq(SYS_BUS_DEVICE(&s->timg[i]), TIMG_INT_MAX + it, qdev_get_gpio_in(intmatrix_dev, timg_edge_int[i] + it));
        }

        qdev_connect_gpio_out_named(DEVICE(&s->timg[i]), ESP32_TIMG_WDT_CPU_RESET_GPIO, 0,
                                    qdev_get_gpio_in_named(dev, ESP32_TIMG_WDT_CPU_RESET_GPIO, i));
        qdev_connect_gpio_out_named(DEVICE(&s->timg[i]), ESP32_TIMG_WDT_SYS_RESET_GPIO, 0,
                                    qdev_get_gpio_in_named(dev, ESP32_TIMG_WDT_SYS_RESET_GPIO, i));
    }
    s->timg[0].wdt_en_at_reset = true;
    const hwaddr spi_base[] = {
            DR_REG_SPI0_BASE, DR_REG_SPI1_BASE, DR_REG_SPI2_BASE, DR_REG_SPI3_BASE
    };
    // speed up vspi and hspi by allowng the controller to send 32 bits at a time.
    // this is only suppoerted by the st7789v 
    object_property_set_bool(OBJECT(&s->spi[2]),"xfer_32_bits",true, &error_abort);
    object_property_set_bool(OBJECT(&s->spi[3]),"xfer_32_bits",true, &error_abort);
    for (int i = 0; i < ESP32_SPI_COUNT; ++i) {        
        qdev_realize(DEVICE(&s->spi[i]), &s->periph_bus, &error_fatal);

        esp32_soc_add_periph_device(sys_mem, &s->spi[i], spi_base[i]);

        sysbus_connect_irq(SYS_BUS_DEVICE(&s->spi[i]), 0,
                           qdev_get_gpio_in(intmatrix_dev, ETS_SPI0_INTR_SOURCE + i));
    }

    for (int i = 0; i < ESP32_I2C_COUNT; i++) {
        const hwaddr i2c_base[] = {
            DR_REG_I2C_EXT_BASE, DR_REG_I2C1_EXT_BASE
        };
        qdev_realize(DEVICE(&s->i2c[i]), &s->periph_bus, &error_fatal);

        esp32_soc_add_periph_device(sys_mem, &s->i2c[i], i2c_base[i]);

        sysbus_connect_irq(SYS_BUS_DEVICE(&s->i2c[i]), 0,
                           qdev_get_gpio_in(intmatrix_dev, ETS_I2C_EXT0_INTR_SOURCE + i));
    }

    qdev_realize(DEVICE(&s->rng), &s->periph_bus, &error_fatal);
    esp32_soc_add_periph_device(sys_mem, &s->rng, ESP32_RNG_BASE);

    qdev_realize(DEVICE(&s->efuse), &s->periph_bus, &error_fatal);
    esp32_soc_add_periph_device(sys_mem, &s->efuse, DR_REG_EFUSE_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->efuse), 0,
                       qdev_get_gpio_in(intmatrix_dev, ETS_EFUSE_INTR_SOURCE));

    qdev_realize(DEVICE(&s->sens), &s->periph_bus, &error_fatal);
    esp32_soc_add_periph_device(sys_mem, &s->sens, DR_REG_SENS_BASE);

    qdev_realize(DEVICE(&s->ana), &s->periph_bus, &error_fatal);
    esp32_soc_add_periph_device(sys_mem, &s->ana, DR_REG_ANA_BASE);

    qdev_realize(DEVICE(&s->wifi), &s->periph_bus, &error_fatal);
    esp32_soc_add_periph_device(sys_mem, &s->wifi, DR_REG_WIFI_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->wifi), 0,
                           qdev_get_gpio_in(intmatrix_dev, ETS_WIFI_MAC_INTR_SOURCE));

    qdev_realize(DEVICE(&s->fe), &s->periph_bus, &error_fatal);
    esp32_soc_add_periph_device(sys_mem, &s->fe, DR_REG_FE_BASE);

    qdev_realize(DEVICE(&s->fe2), &s->periph_bus, &error_fatal);
    esp32_soc_add_periph_device(sys_mem, &s->fe2, DR_REG_FE2_BASE);

    qdev_realize(DEVICE(&s->flash_enc), &s->periph_bus, &error_abort);
    esp32_soc_add_periph_device(sys_mem, &s->flash_enc, DR_REG_SPI_ENCRYPT_BASE);

    qdev_connect_gpio_out_named(DEVICE(&s->efuse), ESP32_EFUSE_UPDATE_GPIO, 0,
                                qdev_get_gpio_in_named(DEVICE(&s->flash_enc), ESP32_FLASH_ENCRYPTION_EFUSE_UPDATE_GPIO, 0));
    qdev_connect_gpio_out_named(DEVICE(&s->dport), ESP32_DPORT_FLASH_ENC_EN_GPIO, 0,
                                qdev_get_gpio_in_named(DEVICE(&s->flash_enc), ESP32_FLASH_ENCRYPTION_ENC_EN_GPIO, 0));
    qdev_connect_gpio_out_named(DEVICE(&s->dport), ESP32_DPORT_FLASH_DEC_EN_GPIO, 0,
                                qdev_get_gpio_in_named(DEVICE(&s->flash_enc), ESP32_FLASH_ENCRYPTION_DEC_EN_GPIO, 0));

//    esp32_soc_add_unimp_device(sys_mem, "esp32.analog", DR_REG_ANA_BASE, 0x1000);
    esp32_soc_add_unimp_device(sys_mem, "esp32.rtcio", DR_REG_RTCIO_BASE, 0x400,0);
    esp32_soc_add_unimp_device(sys_mem, "esp32.iomux", DR_REG_IO_MUX_BASE, 0x2000,0);
    esp32_soc_add_unimp_device(sys_mem, "esp32.hinf", DR_REG_HINF_BASE, 0x1000,0);
    esp32_soc_add_unimp_device(sys_mem, "esp32.slc", DR_REG_SLC_BASE, 0x1000,0);
    esp32_soc_add_unimp_device(sys_mem, "esp32.slchost", DR_REG_SLCHOST_BASE, 0x1000,0);
    esp32_soc_add_unimp_device(sys_mem, "esp32.apbctrl", DR_REG_APB_CTRL_BASE, 0x1000,0);
    esp32_soc_add_unimp_device(sys_mem, "esp32.i2s0", DR_REG_I2S_BASE, 0x1000,0);
    esp32_soc_add_unimp_device(sys_mem, "esp32.i2s1", DR_REG_I2S1_BASE, 0x1000,0);
  //  esp32_soc_add_unimp_device(sys_mem, "esp32.fe", DR_REG_FE_BASE, 0x1000);
 //   esp32_soc_add_unimp_device(sys_mem, "esp32.fe2", DR_REG_FE2_BASE, 0x1000);
    esp32_soc_add_unimp_device(sys_mem, "esp32.chipv7_phy", 0x3ff71000, 0x2000,-1);
    esp32_soc_add_unimp_device(sys_mem, "esp32.chipv7_phya", 0x3ff74000, 0x2000,-1);
//   esp32_soc_add_unimp_device(sys_mem, "esp32.chipv7_rf", 0x3FF45000, 0x3000);
    esp32_soc_add_unimp_device(sys_mem, "esp32.unknown_wifi", 0x3FF5c000  , 0x1000,-1);

    esp32_soc_add_unimp_device(sys_mem, "esp32.unknown_wifi1", 0x3FF5d000 , 0x1000,-1);
    qemu_register_reset((QEMUResetHandler*) esp32_soc_reset, dev);

    /* st7789v is attached to SPI2 and SPI2 so the both HSPI and VSPI will work, 
    they share a single console*/
    DeviceState *disp=ssi_create_slave(s->spi[2].spi, "st7789v");
    DeviceState *disp1=ssi_create_slave(s->spi[3].spi, "st7789v");
    qemu_irq cmd_irq=qemu_irq_split(
                qdev_get_gpio_in_named(disp, "cmd", 0),
                qdev_get_gpio_in_named(disp1, "cmd", 0));
    qemu_irq bl_irq=qemu_irq_split(
                qdev_get_gpio_in_named(disp, "backlight", 0),
                qdev_get_gpio_in_named(disp1, "backlight", 0));
    qdev_connect_gpio_out_named(DEVICE(&s->gpio), ESP32_GPIOS, 16, cmd_irq);
    qdev_connect_gpio_out_named(DEVICE(&s->gpio), ESP32_GPIOS, 4,bl_irq);

    qemu_irq in0=qdev_get_gpio_in_named(DEVICE(&s->gpio), ESP32_GPIOS_IN, 0);
    qemu_irq in35=qdev_get_gpio_in_named(DEVICE(&s->gpio), ESP32_GPIOS_IN, 35);
    qdev_connect_gpio_out_named(disp, "buttons", 0, in0);
    qdev_connect_gpio_out_named(disp, "buttons", 1, in35);
    qdev_connect_gpio_out_named(disp1, "buttons", 0, in0);
    qdev_connect_gpio_out_named(disp1, "buttons", 1, in35);
}

static void esp32_soc_init(Object *obj)
{
    Esp32SocState *s = ESP32_SOC(obj);
    MachineState *ms = MACHINE(qdev_get_machine());
    char name[16];

    MemoryRegion *system_memory = get_system_memory();

    qbus_create_inplace(&s->periph_bus, sizeof(s->periph_bus),
                        TYPE_SYSTEM_BUS, DEVICE(s), "esp32-periph-bus");
    qbus_create_inplace(&s->rtc_bus, sizeof(s->rtc_bus),
                        TYPE_SYSTEM_BUS, DEVICE(s), "esp32-rtc-bus");

    for (int i = 0; i < ms->smp.cpus; ++i) {
        snprintf(name, sizeof(name), "cpu%d", i);
        object_initialize_child(obj, name, &s->cpu[i], TYPE_ESP32_CPU);

        const uint32_t cpuid[ESP32_CPU_COUNT] = { 0xcdcd, 0xabab };
        s->cpu[i].env.sregs[PRID] = cpuid[i];

        snprintf(name, sizeof(name), "cpu%d-mem", i);
        memory_region_init(&s->cpu_specific_mem[i], NULL, name, UINT32_MAX);

        CPUState* cs = CPU(&s->cpu[i]);
        cs->num_ases = 1;
        cpu_address_space_init(cs, 0, "cpu-memory", &s->cpu_specific_mem[i]);

        MemoryRegion *cpu_view_sysmem = g_new(MemoryRegion, 1);
        snprintf(name, sizeof(name), "cpu%d-sysmem", i);
        memory_region_init_alias(cpu_view_sysmem, NULL, name, system_memory, 0, UINT32_MAX);
        memory_region_add_subregion_overlap(&s->cpu_specific_mem[i], 0, cpu_view_sysmem, 0);
        cs->memory = &s->cpu_specific_mem[i];
    }

    for (int i = 0; i < ESP32_UART_COUNT; ++i) {
        snprintf(name, sizeof(name), "uart%d", i);
        object_initialize_child(obj, name, &s->uart[i], TYPE_ESP32_UART);
    }

    object_property_add_alias(obj, "serial0", OBJECT(&s->uart[0]), "chardev");
    object_property_add_alias(obj, "serial1", OBJECT(&s->uart[1]), "chardev");
    object_property_add_alias(obj, "serial2", OBJECT(&s->uart[2]), "chardev");

    object_initialize_child(obj, "gpio", &s->gpio, TYPE_ESP32_GPIO);

    object_initialize_child(obj, "dport", &s->dport, TYPE_ESP32_DPORT);

    object_initialize_child(obj, "intmatrix", &s->intmatrix, TYPE_ESP32_INTMATRIX);

    object_initialize_child(obj, "crosscore_int", &s->crosscore_int, TYPE_ESP32_CROSSCORE_INT);

    object_initialize_child(obj, "rtc_cntl", &s->rtc_cntl, TYPE_ESP32_RTC_CNTL);

    for (int i = 0; i < ESP32_FRC_COUNT; ++i) {
        snprintf(name, sizeof(name), "frc%d", i);
        object_initialize_child(obj, name, &s->frc_timer[i], TYPE_ESP32_FRC_TIMER);
    }

    for (int i = 0; i < ESP32_TIMG_COUNT; ++i) {
        snprintf(name, sizeof(name), "timg%d", i);
        object_initialize_child(obj, name, &s->timg[i], TYPE_ESP32_TIMG);
    }
    for (int i = 0; i < ESP32_SPI_COUNT; ++i) {
        snprintf(name, sizeof(name), "spi%d", i);
        object_initialize_child(obj, name, &s->spi[i], TYPE_ESP32_SPI);
    }

    for (int i = 0; i < ESP32_I2C_COUNT; ++i) {
        snprintf(name, sizeof(name), "i2c%d", i);
        object_initialize_child(obj, name, &s->i2c[i], TYPE_ESP32_I2C);
    }

    object_initialize_child(obj, "rng", &s->rng, TYPE_ESP32_RNG);

    object_initialize_child(obj, "sha", &s->sha, TYPE_ESP32_SHA);

    object_initialize_child(obj, "rsa", &s->rsa, TYPE_ESP32_RSA);

    object_initialize_child(obj, "sens", &s->sens, TYPE_ESP32_SENS);

    object_initialize_child(obj, "ana", &s->ana, TYPE_ESP32_ANA);

    object_initialize_child(obj, "wifi", &s->wifi, TYPE_ESP32_WIFI);

    object_initialize_child(obj, "fe", &s->fe, TYPE_ESP32_FE);

    object_initialize_child(obj, "fe2", &s->fe2, TYPE_ESP32_RAMDEV);

    object_initialize_child(obj, "efuse", &s->efuse, TYPE_ESP32_EFUSE);

    object_initialize_child(obj, "flash_enc", &s->flash_enc, TYPE_ESP32_FLASH_ENCRYPTION);

    qdev_init_gpio_in_named(DEVICE(s), esp32_dig_reset, ESP32_RTC_DIG_RESET_GPIO, 1);
    qdev_init_gpio_in_named(DEVICE(s), esp32_cpu_reset, ESP32_RTC_CPU_RESET_GPIO, ESP32_CPU_COUNT);
    qdev_init_gpio_in_named(DEVICE(s), esp32_cpu_stall, ESP32_RTC_CPU_STALL_GPIO, ESP32_CPU_COUNT);
    qdev_init_gpio_in_named(DEVICE(s), esp32_clk_update, ESP32_RTC_CLK_UPDATE_GPIO, 1);
    qdev_init_gpio_in_named(DEVICE(s), esp32_timg_cpu_reset, ESP32_TIMG_WDT_CPU_RESET_GPIO, 2);
    qdev_init_gpio_in_named(DEVICE(s), esp32_timg_sys_reset, ESP32_TIMG_WDT_SYS_RESET_GPIO, 2);
}

static Property esp32_soc_properties[] = {
    DEFINE_PROP_END_OF_LIST(),
};

static void esp32_soc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = esp32_soc_realize;
    device_class_set_props(dc, esp32_soc_properties);
}

static const TypeInfo esp32_soc_info = {
    .name = TYPE_ESP32_SOC,
    .parent = TYPE_DEVICE,
    .instance_size = sizeof(Esp32SocState),
    .instance_init = esp32_soc_init,
    .class_init = esp32_soc_class_init
};

static void esp32_soc_register_types(void)
{
    type_register_static(&esp32_soc_info);
}

type_init(esp32_soc_register_types)



static uint64_t translate_phys_addr(void *opaque, uint64_t addr)
{
    XtensaCPU *cpu = opaque;

    return cpu_get_phys_page_debug(CPU(cpu), addr);
}


struct Esp32MachineState {
    MachineState parent;

    Esp32SocState esp32;
    DeviceState *flash_dev;
};
#define TYPE_ESP32_MACHINE MACHINE_TYPE_NAME("esp32")

OBJECT_DECLARE_SIMPLE_TYPE(Esp32MachineState, ESP32_MACHINE)


static void esp32_machine_init_spi_flash(Esp32SocState *ss, BlockBackend* blk)
{
    /* "main" flash chip is attached to SPI1 */
    DeviceState *spi_master = DEVICE(&ss->spi[1]);
    BusState* spi_bus = qdev_get_child_bus(spi_master, "spi");
    DeviceState *flash_dev = qdev_new("gd25q32");
    qdev_prop_set_drive(flash_dev, "drive", blk);
    qdev_realize_and_unref(flash_dev, spi_bus, &error_fatal);
    qdev_connect_gpio_out_named(spi_master, SSI_GPIO_CS, 0,
                                qdev_get_gpio_in_named(flash_dev, SSI_GPIO_CS, 0));
}


static void esp32_machine_init_i2c(Esp32SocState *s)
{
    /* It should be possible to create an I2C device from the command line,
     * however for this to work the I2C bus must be reachable from sysbus-default.
     * At the moment the peripherals are added to an unrelated bus, to avoid being
     * reset on CPU reset.
     * If we find a way to decouple peripheral reset from sysbus reset,
     * we can move them to the sysbus and thus enable creation of i2c devices.
     */
    DeviceState *i2c_master = DEVICE(&s->i2c[0]);
    I2CBus* i2c_bus = I2C_BUS(qdev_get_child_bus(i2c_master, "i2c"));
    I2CSlave* tmp105 = i2c_slave_create_simple(i2c_bus, "tmp105", 0x48);
    object_property_set_int(OBJECT(tmp105), "temperature", 25 * 1000, &error_fatal);
}

static void esp32_machine_init_openeth(Esp32SocState *ss)
{
    SysBusDevice *sbd;
    NICInfo *nd = &nd_table[0];
    MemoryRegion* sys_mem = get_system_memory();
    hwaddr reg_base = DR_REG_EMAC_BASE;
    hwaddr desc_base = reg_base + 0x400;
    qemu_irq irq = qdev_get_gpio_in(DEVICE(&ss->intmatrix), ETS_ETH_MAC_INTR_SOURCE);

    const char* type_openeth = "open_eth";
    if (nd->used && nd->model && strcmp(nd->model, type_openeth) == 0) {
        DeviceState* open_eth_dev = qdev_new(type_openeth);
        ss->eth = open_eth_dev;
        qdev_set_nic_properties(open_eth_dev, nd);
        sbd = SYS_BUS_DEVICE(open_eth_dev);
        sysbus_realize_and_unref(sbd, &error_fatal);
        sysbus_connect_irq(sbd, 0, irq);
        memory_region_add_subregion(sys_mem, reg_base, sysbus_mmio_get_region(sbd, 0));
        memory_region_add_subregion(sys_mem, desc_base, sysbus_mmio_get_region(sbd, 1));
    }
}

static void esp32_machine_init(MachineState *machine)
{
    BlockBackend* blk = NULL;
    DriveInfo *dinfo = drive_get_next(IF_MTD);
    if (dinfo) {
        qemu_log("Adding SPI flash device\n");
        blk = blk_by_legacy_dinfo(dinfo);
    } else {
        qemu_log("Not initializing SPI Flash\n");
    }

    Esp32MachineState *ms = ESP32_MACHINE(machine);
    object_initialize_child(OBJECT(ms), "soc", &ms->esp32, TYPE_ESP32_SOC);
    Esp32SocState *ss = ESP32_SOC(&ms->esp32);

    if (blk) {
        ss->dport.flash_blk = blk;
    }
    qdev_prop_set_chr(DEVICE(ss), "serial0", serial_hd(0));
    qdev_prop_set_chr(DEVICE(ss), "serial1", serial_hd(1));
    qdev_prop_set_chr(DEVICE(ss), "serial2", serial_hd(2));

    qdev_realize(DEVICE(ss), NULL, &error_fatal);

    if (blk) {
        esp32_machine_init_spi_flash(ss, blk);
    }

    esp32_machine_init_i2c(ss);

    esp32_machine_init_openeth(ss);

    /* Need MMU initialized prior to ELF loading,
     * so that ELF gets loaded into virtual addresses
     */
    cpu_reset(CPU(&ss->cpu[0]));

    const char *load_elf_filename = NULL;
    if (machine->firmware) {
        load_elf_filename = machine->firmware;
    }
    if (machine->kernel_filename) {
        qemu_log("Warning: both -bios and -kernel arguments specified. Only loading the the -kernel file.\n");
        load_elf_filename = machine->kernel_filename;
    }

    if (load_elf_filename) {
        uint64_t elf_entry;
        uint64_t elf_lowaddr;
        int success = load_elf(load_elf_filename, NULL,
                               translate_phys_addr, &ss->cpu[0],
                               &elf_entry, &elf_lowaddr,
                               NULL, NULL, 0, EM_XTENSA, 0, 0);
        if (success > 0) {
            ss->cpu[0].env.pc = elf_entry;
        }
    } else {
        char *rom_binary = qemu_find_file(QEMU_FILE_TYPE_BIOS, "esp32-v3-rom.bin");
        if (rom_binary == NULL) {
            error_report("Error: -bios argument not set, and ROM code binary not found");
            exit(1);
        }

        int size = load_image_targphys(rom_binary, esp32_memmap[ESP32_MEMREGION_IROM].base, esp32_memmap[ESP32_MEMREGION_IROM].size);
        if (size < 0) {
            error_report("Error: could not load ROM binary '%s'", rom_binary);
            exit(1);
        }
        g_free(rom_binary);
    }
}

/* Initialize machine type */
static void esp32_machine_class_init(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);
    mc->desc = "Espressif ESP32 machine";
    mc->init = esp32_machine_init;
    mc->max_cpus = 2;
    mc->is_default = true;
    mc->default_cpus = 2;
}

static const TypeInfo esp32_info = {
    .name = TYPE_ESP32_MACHINE,
    .parent = TYPE_MACHINE,
    .instance_size = sizeof(Esp32MachineState),
    .class_init = esp32_machine_class_init,
};

static void esp32_machine_type_init(void)
{
    type_register_static(&esp32_info);
}

type_init(esp32_machine_type_init);

