#include <rtthread.h>
#include <stdlib.h>
#include <string.h>
#include "g1632.h"

// #define DBG_ENABLE
#define DBG_SECTION_NAME     "g1632"
#define DBG_LEVEL            DBG_LOG
#define DBG_COLOR
#include <rtdbg.h>

#define I2C_BUS    "i2c1"

static rt_thread_t tid1 = RT_NULL;
static rt_int32_t param[2];

static void g1632_example_entry(void *parameter)
{
    g1632_device_t dev = RT_NULL;

    dev = g1632_init(I2C_BUS, RT_NULL);
    if (dev == RT_NULL)
        goto _exit;

    rt_uint16_t value;
    g1632_get_gamma(dev, (rt_uint8_t)(((rt_int32_t *)parameter)[0]), &value);
    LOG_D("g1632 gamma for channel %d set to %d from %d\n", (((rt_int32_t *)parameter)[0]), (((rt_int32_t *)parameter)[1]), value);

    g1632_set_gamma(dev, (rt_uint8_t)(((rt_int32_t *)parameter)[0]), (rt_uint16_t)(((rt_int32_t *)parameter)[1]));
    rt_thread_mdelay(10);

_exit:    
    g1632_deinit(dev);

}

int g1632_example(int argc, char** argv)
{
    if (argc == 3) {
        param[0] = atoi(argv[1]);
        param[1] = atoi(argv[2]);

        tid1 = rt_thread_create("g1632_example",
                                g1632_example_entry, param,
                                512, 25, 10);

        if (tid1 != RT_NULL)
            rt_thread_startup(tid1);        
    } else if (argc == 2) {
        if(!strcmp(argv[1], "reset")) {
            g1632_device_t dev = RT_NULL;
            dev = g1632_init(I2C_BUS, RT_NULL);
            if (dev == RT_NULL)
                return 0;
                        /* reset before use it */
            g1632_reset(dev);
            rt_thread_mdelay(10);
            LOG_D("reset done\n");
        }
    }

    return 0;
}
#ifdef FINSH_USING_MSH
MSH_CMD_EXPORT(g1632_example, a g1632 example);
#endif
