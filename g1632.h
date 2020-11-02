#ifndef __G1632_H__
#define __G1632_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <rtthread.h>
#include <rtdevice.h>
#include <rtdef.h>
#include <drv_common.h>

#define G1632_ADDR_DEFAULT    0x74    /**< G1632 default address */

#define G1632_CTRL_BYTE 0x0
#define G1632_DVCOM 0x1

#define G1632_HOT_EN_MASK    0x40
#define G1632_RESET_MASK     0x10
#define G1632_WR_TO_NVM_MASK 0x8
#define G1632_OUT_EN_MASK    0x2

#define G1632_BANK_SEL_PIN    GET_PIN(C, 13)
#define G1632_NWR_MCU_PIN    GET_PIN(C, 14)

/* g1632 device structure */
struct g1632_device
{
    rt_device_t bus;
    rt_uint8_t i2c_addr;
    rt_base_t bank_sel_pin;
    rt_base_t nwr_mcu_pin;
};
typedef struct g1632_device *g1632_device_t;

void g1632_reset(g1632_device_t dev);
g1632_device_t g1632_init(void);
void g1632_deinit(g1632_device_t dev);

rt_err_t g1632_get_gamma(g1632_device_t dev, rt_uint8_t channel, rt_uint16_t * value);
rt_err_t g1632_set_gamma(g1632_device_t dev, rt_uint8_t channel, rt_uint16_t value);

#ifdef __cplusplus
}
#endif

#endif
