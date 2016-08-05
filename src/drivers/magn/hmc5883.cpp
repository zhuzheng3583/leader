/*******************************Copyright (c)***************************
**
** Porject name:	leader
** Created by:		zhuzheng<happyzhull@163.com>
** Created date:	2016/07/31
** Modified by:
** Modified date:
** Descriptions:
**
***********************************************************************/
#include "hmc5883.h"
#include <math.h>

namespace driver {

hmc5883::hmc5883(PCSTR name, s32 id) :
    device(name, id)
{

}

hmc5883::~hmc5883(void)
{

}

s32 hmc5883::probe(i2c *pi2c, u8 slave_addr)
{
    ASSERT(pi2c != NULL);

	if (device::probe() < 0) {
		ERR("%s: failed to probe.\n", _name);
		goto fail0;
	}

    _i2c = pi2c;
    _slave_addr = slave_addr;

    hmc5883::init();

    //while(1) {
      //  hmc5883::read_raw();
    //}
    return 0;

fail0:
    return -1;
}

s32 hmc5883::init(void)
{
	u8 id_a = hmc5883::read_reg8(ADDR_ID_A);
	u8 id_b = hmc5883::read_reg8(ADDR_ID_B);
	u8 id_c = hmc5883::read_reg8(ADDR_ID_C);
    if ((id_a != ID_A_WHO_AM_I) || (id_b != ID_B_WHO_AM_I) || (id_c != ID_C_WHO_AM_I)) {
        return -1;
    }

    hmc5883::write_reg8(ADDR_CONFIG_A, HMC_DEFAULT_CONFIGA_VALUE);
    hmc5883::write_reg8(ADDR_CONFIG_B, HMC_DEFAULT_CONFIGB_VALUE);
    hmc5883::write_reg8(ADDR_MODE, HMC_DEFAULT_MODE_VALUE); //初始化HMC5883

    return 0;
}


s32 hmc5883::read_raw(void)
{
	f64 angle;
	u32 acr;
    //连续读出HMC5883内部角度数据，地址范围0x3~0x5
    u8 tmp[6];
    s16 data[3];
    if (read_reg(ADDR_DATA_X_MSB, tmp, 6))
        return -1;
    data[0] = (tmp[0] << 8) | tmp[1];//Combine MSB and LSB of X Data output register
    data[1] = (tmp[2] << 8) | tmp[3];//Combine MSB and LSB of Z Data output register
    data[2] = (tmp[4] << 8) | tmp[5];//Combine MSB and LSB of Y Data output register
    angle = atan2((f64)data[2],(f64)data[0])*(180/3.14159265)+180;//单位：角度 (0~360)
    angle *= 10;
    acr = (u32)angle;
    INF("%s: %f, %d.\n", _name, angle, acr);

    return 0;
}


s32 hmc5883::read_reg8(u8 reg)
{
    s32 ret = 0;
    u8 data = 0;
    hmc5883::read_reg(reg, &data, 1);
    if (ret < 0) {
        return -1;
    }

    return ((s32)data);
}


s32 hmc5883::write_reg8(u8 reg, u8 data)
{
    s32 ret = 0;
    ret = hmc5883::write_reg(reg, &data, 1);
    if (ret < 0) {
        return -1;
    }

    return 0;
}

s32 hmc5883::read_reg(u8 reg, u8 *buf, u8 len)
{
	struct i2c_msg msgs[] = {
		[0] = {
			.addr = _slave_addr,
			.flags = 0,
			.len = 1,
			.buf = &reg,
		},
		[1] = {
			.addr = _slave_addr + 1,
			.flags = I2C_M_RD,
			.len = len,
			.buf = buf,
		},
	};
	if (_i2c->transfer(msgs, 2) != 0) {
		ERR("%s: failed to i2c transfer!\n", _name);
        return -1;
	}

    return 0;
}

s32 hmc5883::write_reg(u8 reg, u8 *buf, u8 len)
{
	struct i2c_msg msgs[] = {
		[0] = {
			.addr = _slave_addr,
			.flags = 0,
			.len = 1,
			.buf = &reg,
		},
		[1] = {
			.addr = _slave_addr,
			.flags = 0,
			.len = len,
			.buf = buf,
		},
	};
	if (_i2c->transfer(msgs, 2) != 0) {
		ERR("%s: failed to i2c transfer!\n", _name);
        return -1;
	}

    return 0;
}

}
