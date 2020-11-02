#include <rtthread.h>
#include <stdlib.h>
#include <string.h>
#include "g1632.h"

// #define DBG_ENABLE
#define DBG_SECTION_NAME     "g1632"
#define DBG_LEVEL            DBG_LOG
#define DBG_COLOR
#include <rtdbg.h>

#define I2C_BUS_1    "i2c1"
#define I2C_BUS_2    "i2c2"
#define I2C_BUS_3    "i2c3"
#define I2C_BUS_4    "i2c4"

static rt_thread_t tid1 = RT_NULL;
static rt_int32_t param[2];

static void g1632_example_entry(void *parameter)
{
    g1632_device_t dev = RT_NULL;

    dev = g1632_init();
    if (dev == RT_NULL)
        goto _exit;

    rt_uint16_t value;
    g1632_get_gamma(&dev[0], (rt_uint8_t)(((rt_int32_t *)parameter)[0]), &value);
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
            // g1632_reset(dev);
            rt_thread_mdelay(10);
            LOG_D("reset done\n");
        }
    }

    return 0;
}

int main(void)
{
    int count = 1;

    x = 0;
    y = 0;

    while (1)
    {

        for (int i=0; i< ;i++) {
            for (int j=0; j< ;j++) {
                if (){
                    g1632_set_gamma(&dev[i], j, &value);
                } else () {
                    g1632_set_gamma(&dev[i], j, &value);
                }
            }
        }

        g1632_enable_output(&dev[0]);
        g1632_enable_output(&dev[1]);
        g1632_enable_output(&dev[2]);
        g1632_enable_output(&dev[3]);

        rt_thread_mdelay(200);

        for (int i=0; i< ;i++) {
            for (int j=0; j< ;j++) {
                if (){
                    g1632_set_gamma(&dev[i], j, &value);
                } else () {
                    g1632_set_gamma(&dev[i], j, &value);
                } else () {
                    g1632_set_gamma(&dev[i], j, &value);
                }
            }
        }
        
        rt_thread_mdelay(200);

        g1632_disable_output(&dev[0]);
        g1632_disable_output(&dev[1]);
        g1632_disable_output(&dev[2]);
        g1632_disable_output(&dev[3]);

        g1632_get_gamma(&dev[0], (rt_uint8_t)(((rt_int32_t *)parameter)[0]), &value);
        LOG_D("g1632 gamma for channel %d set to %d from %d\n", (((rt_int32_t *)parameter)[0]), (((rt_int32_t *)parameter)[1]), value);

        g1632_set_gamma(dev, (rt_uint8_t)(((rt_int32_t *)parameter)[0]), (rt_uint16_t)(((rt_int32_t *)parameter)[1]));
        rt_thread_mdelay(10);
        rt_thread_mdelay(1000);

        x++;
        y++;
    }

    return RT_EOK;
}

#ifdef FINSH_USING_MSH
MSH_CMD_EXPORT(g1632_example, a g1632 example);
#endif
