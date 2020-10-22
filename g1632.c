#include "g1632.h"

// #define DBG_ENABLE
#define DBG_SECTION_NAME     "g1632"
#define DBG_LEVEL            DBG_LOG
#define DBG_COLOR
#include <rtdbg.h>

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
static rt_err_t read_reg(g1632_device_t dev, rt_uint8_t reg, rt_uint8_t len, rt_uint8_t *buf)
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
 * @brief set gamma
 * 
 * VOUT = [(STATIC_Hi) / 1024*Decimal Value].
 * 
 * reg(x)bit[5:0]   -  Gamma(Y) MSB
 * reg(x+1)bit[7:4] -  Gamma(Y) LSB
 * reg(x+1)bit[1:0] -  Gamma(Y+1) MSB
 * reg(x+2)bit[7:0] -  Gamma(Y+1) LSB
 * 
 */
rt_err_t g1632_set_gamma(g1632_device_t dev, rt_uint8_t channel, rt_uint16_t value)
{
    if (channel >= 14 || value >= 1024) {
        return -RT_ERROR; 
    }

    rt_int8_t res = 0;
    rt_uint8_t control_byte = 0;
    res = read_reg(dev, G1632_CTRL_BYTE, 1, &control_byte);
    if (res != RT_EOK || (control_byte & G1632_WR_TO_NVM_MASK) || (control_byte & G1632_RESET_MASK )) {
        return -RT_ERROR;
    }
    
    rt_uint8_t index_addr, temp;
    index_addr = 2 + (channel/2) * 3;
    res = read_reg(dev, index_addr + 1, 1, &temp);
    if (res == RT_EOK) {
        rt_uint8_t regs[2];
        if (channel % 2) {
            index_addr++;
            regs[0] = (rt_uint8_t)((value >> 8) & 0x3) + (temp & 0xf0);
            regs[1] = (rt_uint8_t)(value & 0xff);
        } else {
            regs[0] = (rt_uint8_t)(value >> 4);
            regs[1] = (rt_uint8_t)((value & 0xf) << 4) + (temp & 0x03);
        }

        res = write_reg(dev, index_addr, 2, regs);
        LOG_D("write_reg index %d, regs[0] 0x%02x, regs[1] 0x%02x\n", index_addr, regs[0], regs[1]);
    }

    return res;
}

rt_err_t g1632_get_gamma(g1632_device_t dev, rt_uint8_t channel, rt_uint16_t * value) {
    if (channel >= 14) {
        return -RT_ERROR; 
    }

    rt_int8_t res = 0;
    rt_uint8_t control_byte = 0;
    res = read_reg(dev, G1632_CTRL_BYTE, 1, &control_byte);
    if (res != RT_EOK || (control_byte & G1632_WR_TO_NVM_MASK) || (control_byte & G1632_RESET_MASK )) {
        return -RT_ERROR;
    }

    rt_uint8_t index_addr, regs[2];
    if (channel % 2) {
        index_addr = 2 + (channel/2) * 3 + 1;
        res = read_reg(dev, index_addr, 2, regs);
        *value = (((rt_uint16_t)(regs[0] & 0x3)) << 8) + regs[1];
    } else {
        index_addr = 2 + (channel/2) * 3;
        res = read_reg(dev, index_addr, 2, regs);
        *value = (((rt_uint16_t)(regs[0] & 0x3f)) << 4) + (regs[1] >> 4);
    }

    LOG_D("read_reg index %d, regs[0] 0x%02x, regs[1] 0x%02x\n", index_addr, regs[0], regs[1]);
    return res;    
}

/**
 * @brief set dvcom
 * 
 * @param dev the pointer of device structure
 * 
 * Calculated VCOM Output Voltages
 * 
 * VALUE    VOUT(V)
 * 0        5.485
 * 10       5.313
 * 20       5.141
 * 30       4.969
 * 40       4.797
 * 50       4.625
 * 60       4.453
 * 70       4.281
 * 80       4.109
 * 90       3.936
 * 100      3.764
 * 110      3.592
 * 120      3.420
 * 127      3.300
 */
rt_err_t g1632_set_dvcom(g1632_device_t dev, rt_uint8_t value)
{
    rt_uint8_t reg = value << 1;
    return write_reg(dev, G1632_DVCOM, 1, &reg);
}

rt_err_t g1632_get_dvcom(g1632_device_t dev, rt_uint8_t *value)
{
    rt_int8_t res = 0;
    rt_uint8_t reg;
    res = read_reg(dev, G1632_DVCOM, 1, &reg);
    if (res == RT_EOK) {
        *value = reg >> 1;
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
    rt_uint8_t reg = G1632_RESET_MASK;
    write_reg(dev, G1632_CTRL_BYTE, 1, &reg);
}

g1632_device_t g1632_init(const char *dev_name, rt_uint8_t i2c_addr)
{
    g1632_device_t dev = RT_NULL;

    /* 蜂鸣器引脚为输出模式 */
    rt_pin_mode(BEEP_PIN_NUM, PIN_MODE_OUTPUT);
    /* 设置低电平 */
    rt_pin_write(BEEP_PIN_NUM, PIN_LOW);    

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
