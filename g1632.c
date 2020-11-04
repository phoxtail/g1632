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
        res = read_reg(dev, index_addr, 1, regs);
        res = read_reg(dev, index_addr + 1, 1, regs + 1);
        *value = (((rt_uint16_t)(regs[0] & 0x3)) << 8) + regs[1];
    } else {
        index_addr = 2 + (channel/2) * 3;
        res = read_reg(dev, index_addr, 1, regs);
        res = read_reg(dev, index_addr + 1, 1, regs + 1);
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

rt_err_t g1632_enable_output(g1632_device_t dev)
{
    rt_int8_t res = 0;
    rt_uint8_t control_byte = 0;
    res = read_reg(dev, G1632_CTRL_BYTE, 1, &control_byte);
    if (res != RT_EOK || (control_byte & G1632_WR_TO_NVM_MASK) || (control_byte & G1632_RESET_MASK )) {
        return -RT_ERROR;
    }

    control_byte |= (G1632_OUT_EN_MASK);
    write_reg(dev, G1632_CTRL_BYTE, 1, &control_byte);
    return res;
}

rt_err_t g1632_disable_output(g1632_device_t dev)
{
    rt_int8_t res = 0;
    rt_uint8_t control_byte = 0;
    res = read_reg(dev, G1632_CTRL_BYTE, 1, &control_byte);
    if (res != RT_EOK || (control_byte & G1632_WR_TO_NVM_MASK) || (control_byte & G1632_RESET_MASK )) {
        return -RT_ERROR;
    }

    control_byte &= (~(G1632_OUT_EN_MASK));
    write_reg(dev, G1632_CTRL_BYTE, 1, &control_byte);
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

g1632_device_t g1632_init(void)
{
    g1632_device_t dev = RT_NULL;

    dev = rt_calloc(4, sizeof(struct g1632_device));
    if (dev == RT_NULL)
    {
        LOG_E("Can't allocate memory for g1632 device");
        goto __exit;
    }

    dev[0].bus = rt_device_find("i2c1");
    dev[0].bank_sel_pin = GET_PIN(C, 13);
    dev[0].nwr_mcu_pin = GET_PIN(C, 14);
    dev[0].i2c_addr = G1632_ADDR_DEFAULT;
    dev[1].bus = rt_device_find("i2c2");
    dev[1].bank_sel_pin = GET_PIN(B, 6);
    dev[1].nwr_mcu_pin = GET_PIN(B, 7);
    dev[1].i2c_addr = G1632_ADDR_DEFAULT;
    dev[2].bus = rt_device_find("i2c3");
    dev[2].bank_sel_pin = GET_PIN(B, 0);
    dev[2].nwr_mcu_pin = GET_PIN(B, 1);
    dev[2].i2c_addr = G1632_ADDR_DEFAULT;
    dev[3].bus = rt_device_find("i2c4");
    dev[3].bank_sel_pin = GET_PIN(A, 6);
    dev[3].nwr_mcu_pin = GET_PIN(A, 7);
    dev[3].i2c_addr = G1632_ADDR_DEFAULT;
    if (dev[0].bus == RT_NULL || dev[1].bus == RT_NULL || dev[2].bus == RT_NULL || dev[3].bus == RT_NULL)
    {
        LOG_E("i2c_bus for g1632 not found!");
        goto __exit;
    }

    for (int i = 0; i < 4; i++) {
        rt_pin_mode(dev[i].bank_sel_pin, PIN_MODE_OUTPUT);
        rt_pin_mode(dev[i].nwr_mcu_pin, PIN_MODE_OUTPUT);

        rt_pin_write(dev[i].bank_sel_pin, PIN_LOW);
        rt_pin_write(dev[i].nwr_mcu_pin, PIN_HIGH);
    }

    LOG_D("g1632 init done");
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
