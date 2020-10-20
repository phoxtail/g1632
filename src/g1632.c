#include <rtthread.h>

/**
 * This function writes the value of the register for pca9685
 *
 * @param dev the pointer of device driver structure
 * @param reg the register for pca9685
 * @param data value to write
 *
 * @return the writing status, RT_EOK reprensents  writing the value of the register successfully.
 */
static rt_err_t write_reg(g1632_device_t dev, rt_uint8_t reg, unsigned short length, const unsigned char *data)
{
    rt_int8_t res = 0;
#ifdef RT_USING_I2C
    struct rt_i2c_msg msgs;
    rt_uint8_t buf[length+1];
    buf[0] = reg;
    int i;
    for ( i = 1; i <= length; i++)
    {
        buf[i] = data[i-1];
    }

#endif
    if (dev->bus->type == RT_Device_Class_I2CBUS)
    {
#ifdef RT_USING_I2C
        msgs.addr  = dev->i2c_addr;    /* slave address */
        msgs.flags = RT_I2C_WR;        /* write flag */
        msgs.buf   = buf;              /* Send data pointer */
        msgs.len   = length+1;

        if (rt_i2c_transfer((struct rt_i2c_bus_device *)dev->bus, &msgs, 1) == 1)
        {
            res = RT_EOK;
        }
        else
        {
            res = -RT_ERROR;
        }
#endif
    }
    else
    {
        res = -RT_ERROR;
    }
    return res;
}

/**
 * This function reads the value of registers for pca9685
 *
 * @param dev the pointer of device driver structure
 * @param reg the register for pca9685
 * @param len number of register
 * @param buf read data pointer
 *
 * @return the reading status, RT_EOK reprensents  reading the value of registers successfully.
 */
static rt_err_t read_regs(pca9685_device_t dev, rt_uint8_t reg, rt_uint8_t len, rt_uint8_t *buf)
{
    rt_int8_t res = 0;
#ifdef RT_USING_I2C
    struct rt_i2c_msg msgs[2];
#endif

    if (dev->bus->type == RT_Device_Class_I2CBUS)
    {
#ifdef RT_USING_I2C
        msgs[0].addr  = dev->i2c_addr;    /* Slave address */
        msgs[0].flags = RT_I2C_WR;        /* Write flag */
        msgs[0].buf   = &reg;             /* Slave register address */
        msgs[0].len   = 1;                /* Number of bytes sent */

        msgs[1].addr  = dev->i2c_addr;    /* Slave address */
        msgs[1].flags = RT_I2C_RD;        /* Read flag */
        msgs[1].buf   = buf;              /* Read data pointer */
        msgs[1].len   = len;              /* Number of bytes read */

        if (rt_i2c_transfer((struct rt_i2c_bus_device *)dev->bus, msgs, 2) == 2)
        {
            res = RT_EOK;
        }
        else
        {
            res = -RT_ERROR;
        }
#endif
    }
    else
    {
        res = -RT_ERROR;
    }
    return res;
}

/**
 * @brief restart g1632 
 * 
 * @param dev the pointer of device structure
 */
void g1632_reset(g1632_device_t dev)
{
    rt_uint8_t reg;
    reg = G1632_RESET_MASK;
    write_reg(dev, G1632_CTRL_BYTE, 1, &reg);
}

g1632_device_t g1632_init(const char *dev_name, rt_uint8_t i2c_addr)
{
    g1632_device_t dev = RT_NULL;

    dev = rt_calloc(1, sizeof(struct g1632_device));
    if (dev == RT_NULL)
    {
        LOG_E("Can't allocate memory for g1632 device on '%s' ", dev_name);
        goto __exit;
    }

    dev->bus = rt_device_find(dev_name);
    if (dev->bus == RT_NULL)
    {
        LOG_E("i2c_bus %s for g1632 not found!", dev_name);
        goto __exit;
    }

    if (dev->bus->type == RT_Device_Class_I2CBUS)
    {
        if (i2c_addr != RT_NULL)
            dev->i2c_addr = i2c_addr;
        else
            dev->i2c_addr = G1632_ADDR_DEFAULT;
    }
    else
    {
        LOG_E("Unsupported device:'%s'!", dev_name);
        goto __exit;
    }

    /* reset before use it */
    g1632_reset(dev);
    rt_thread_mdelay(10);

    LOG_D("g1632 init done", dev_name);
    return dev;

__exit:
    if (dev != RT_NULL)
        rt_free(dev);

    return RT_NULL;
}

void g1632_deinit(g1632_device_t dev)
{
    RT_ASSERT(dev);
    rt_free(dev);
}
