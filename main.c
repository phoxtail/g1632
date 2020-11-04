/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-10-12     RT-Thread    first version
 */

#include <rtthread.h>
#include <stdlib.h>
#include <string.h>
#include "i2c_utils.h"
#include "g1632.h"

#define DBG_TAG "main"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#define MAX_CHIP 2
#define MAX_CHANNEL 28
#define MAX_CHANNEL_PER_CHIP 14
#define GAMMA_MIDDLE 512
#define GAMMA_STEP 19

int main(void)
{
    int count = 0;

    g1632_device_t dev = RT_NULL;
    dev = g1632_init();
    if (dev == RT_NULL) {
        LOG_E("Failed to init g1632\n");
        return RT_EOK;
    }

//    while (1) {
//        g1632_disable_output(&dev[0]);
//        g1632_disable_output(&dev[1]);
//        g1632_disable_output(&dev[2]);
//        g1632_disable_output(&dev[3]);
//
//        for (int i=0; i< MAX_CHIP; i++) {
//            for (int j=0; j< MAX_CHANNEL_PER_CHIP; j++) {
//                g1632_set_gamma(&dev[i], j, 0);
//                g1632_set_gamma(&dev[i+2], j, 0);
//            }
//        }
//
//        g1632_enable_output(&dev[0]);
//        g1632_enable_output(&dev[1]);
//        g1632_enable_output(&dev[2]);
//        g1632_enable_output(&dev[3]);
//        rt_kprintf("_");
//        rt_thread_mdelay(1000);
//
//        g1632_disable_output(&dev[0]);
//        g1632_disable_output(&dev[1]);
//        g1632_disable_output(&dev[2]);
//        g1632_disable_output(&dev[3]);
//
//        for (int i=0; i< MAX_CHIP; i++) {
//            for (int j=0; j< MAX_CHANNEL_PER_CHIP; j++) {
//                g1632_set_gamma(&dev[i], j, 1023);
//                g1632_set_gamma(&dev[i+2], j, 1023);
//            }
//        }
//
//        g1632_enable_output(&dev[0]);
//        g1632_enable_output(&dev[1]);
//        g1632_enable_output(&dev[2]);
//        g1632_enable_output(&dev[3]);
//        rt_kprintf("+");
//        rt_thread_mdelay(1000);
//    }
#if 1

    g1632_disable_output(&dev[0]);
    g1632_disable_output(&dev[1]);
    g1632_disable_output(&dev[2]);
    g1632_disable_output(&dev[3]);

    for (int i=0; i< MAX_CHIP; i++) {
        for (int j=0; j< MAX_CHANNEL_PER_CHIP; j++) {
            g1632_set_gamma(&dev[i], j, 0);
            g1632_set_gamma(&dev[i+2], j, 1023);
        }
    }

    g1632_enable_output(&dev[0]);
    g1632_enable_output(&dev[1]);
    g1632_enable_output(&dev[2]);
    g1632_enable_output(&dev[3]);
    rt_thread_mdelay(100);

    g1632_disable_output(&dev[0]);
    g1632_disable_output(&dev[1]);
    g1632_disable_output(&dev[2]);
    g1632_disable_output(&dev[3]);

    for (int i=0; i< MAX_CHIP; i++) {
        for (int j=0; j< MAX_CHANNEL_PER_CHIP; j++) {
            g1632_set_gamma(&dev[i], j, 1023);
            g1632_set_gamma(&dev[i+2], j, 0);
        }
    }

    g1632_enable_output(&dev[0]);
    g1632_enable_output(&dev[1]);
    g1632_enable_output(&dev[2]);
    g1632_enable_output(&dev[3]);
    rt_thread_mdelay(100);

    g1632_disable_output(&dev[0]);
    g1632_disable_output(&dev[1]);
    g1632_disable_output(&dev[2]);
    g1632_disable_output(&dev[3]);

    for (int i=0; i< MAX_CHIP; i++) {
        for (int j=0; j< MAX_CHANNEL_PER_CHIP; j++) {
            g1632_set_gamma(&dev[i], j, GAMMA_MIDDLE);
            g1632_set_gamma(&dev[i+2], j, GAMMA_MIDDLE);
        }
    }

    while (1) {
        rt_uint16_t value_high, value_low;
        rt_uint8_t channel;
        rt_int32_t chip;

        chip = count / MAX_CHANNEL_PER_CHIP;
        channel = count % MAX_CHANNEL_PER_CHIP;

        value_high = (count*(GAMMA_STEP) >= (GAMMA_MIDDLE))?1023:((GAMMA_MIDDLE)+count*(GAMMA_STEP));
        value_low = (count*(GAMMA_STEP) >= (GAMMA_MIDDLE))?0:((GAMMA_MIDDLE)-count*(GAMMA_STEP));

        g1632_set_gamma(&dev[chip], channel, value_high);
        g1632_set_gamma(&dev[chip+2], channel, value_low);

        g1632_enable_output(&dev[0]);
        g1632_enable_output(&dev[1]);
        g1632_enable_output(&dev[2]);
        g1632_enable_output(&dev[3]);
        rt_thread_mdelay(100);

        g1632_disable_output(&dev[0]);
        g1632_disable_output(&dev[1]);
        g1632_disable_output(&dev[2]);
        g1632_disable_output(&dev[3]);
        g1632_set_gamma(&dev[chip], channel, value_low);
        g1632_set_gamma(&dev[chip+2], channel, value_high);
        g1632_enable_output(&dev[0]);
        g1632_enable_output(&dev[1]);
        g1632_enable_output(&dev[2]);
        g1632_enable_output(&dev[3]);
        rt_thread_mdelay(100);

        g1632_disable_output(&dev[0]);
        g1632_disable_output(&dev[1]);
        g1632_disable_output(&dev[2]);
        g1632_disable_output(&dev[3]);

        g1632_set_gamma(&dev[chip], channel, GAMMA_MIDDLE);
        g1632_set_gamma(&dev[chip+2], channel, GAMMA_MIDDLE);

        rt_kprintf(".");

        count++;
        if (count >= MAX_CHANNEL) {
            count = 0;
            rt_thread_mdelay(1000);
            rt_kprintf("+");
        }
    }
#endif
    return RT_EOK;
}
