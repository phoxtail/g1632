#ifndef __G1632_H__
#define __G1632_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <rtthread.h>
#include <rtdevice.h>
#include <rtdef.h>

#define G1632_ADDR_DEFAULT    0x74    /**< G1632 default address */

#define G1632_CTRL_BYTE 0x0
#define G1632_DVCOM 0x1

#define G1632_HOT_EN_MASK    0x40
#define G1632_RESET_MASK     0x10
#define G1632_WR_TO_NVM_MASK 0x8
#define G1632_OUT_EN_MASK    0x2

/* g1632 device structure */
struct g1632_device
{
    rt_device_t bus;
    rt_uint8_t i2c_addr;
};
typedef struct g1632_device *g1632_device_t;

void g1632_reset(g1632_device_t dev);
g1632_device_t g1632_init(const char *dev_name, rt_uint8_t i2c_addr);
void g1632_deinit(g1632_device_t dev);

#ifdef __cplusplus
}
#endif

#endif
