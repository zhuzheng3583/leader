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

hmc5883::hmc5883(PCSTR devname, s32 devid) :
    device(devname, devid),
    _mag_reports(NULL),
    _range_scale(0), /* default range scale from counts to gauss */
    _range_ga(1.3f)

{
	// default scaling
	_scale.x_offset = 0;
	_scale.x_scale = 1.0f;
	_scale.y_offset = 0;
	_scale.y_scale = 1.0f;
	_scale.z_offset = 0;
	_scale.z_scale = 1.0f;

	_params.name = "hmc5883_thread";
	_params.priority = 0;
	_params.stacksize = 1024;
	_params.func = (void *)thread::func;
	_params.parg = (thread *)this;
}

hmc5883::~hmc5883(void)
{

}

s32 hmc5883::probe(i2c *pi2c, u8 slave_addr)
{
    ASSERT(pi2c != NULL);

    _i2c = pi2c;
    _slave_addr = slave_addr;

    s32 ret = init();
    if (ret < 0) {
		ERR("%s: failed to init.\n", _devname);
        goto fail0;
    }

    return 0;

fail0:
    return -1;
}



s32 hmc5883::open(s32 flags)
{
    return 0;
}

s32 hmc5883::read(u8 *buf, u32 size)
{
    u32 count = size / sizeof(struct mag_report);

	/* buffer must be large enough */
	if (count < 1)
		return -ENOSPC;

	/* if automatic measurement is not enabled, get a fresh measurement into the buffer */
	//if (_call_interval == 0) {
	//	_accel_reports->flush();
	//	measure();
	//}

	/* if no data, error (we could block here) */
	if (_mag_reports->empty())
		return -EAGAIN;

	//perf_count(_accel_reads);

	/* copy reports out of our buffer to the caller */
    struct mag_report *mag_buf = reinterpret_cast<struct mag_report *>(buf);
	int transferred = 0;
	while (count--) {
		if (!_mag_reports->get(mag_buf))
			break;
		transferred++;
		mag_buf++;
	}

	/* return the number of bytes transferred */
	return (transferred * sizeof(mag_report));

}

void hmc5883::run(void *parg)
{
    for (;;) {
        /* Send the command to begin a measurement. */
        write_reg8(ADDR_MODE, MODE_REG_SINGLE_MODE);

        /* wait for it to complete */
        msleep(HMC5883_CONVERSION_INTERVAL_MS);

        measure();
    }
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

	/* allocate basic report buffers */
	_mag_reports = new ringbuffer(2, sizeof(mag_report));
	if (_mag_reports == NULL)
		goto fail0;


    return 0;

fail0:
    return -1;
}

s32 hmc5883::reset(void)
{
    return 0;
}

void hmc5883::measure(void)
{
#pragma pack(push, 1)
    struct { /* status register and data as read back from the device */
        u8  x[2];
        u8  z[2];
        u8  y[2];
    } hmc_report;
#pragma pack(pop)
    struct {
        s16 x;
        s16 y;
        s16 z;
    } report;

    s32 ret;
    u8 check_counter;

    //perf_begin(_sample_perf);
    struct mag_report new_report;
    bool sensor_is_onboard = false;

    float xraw_f;
    float yraw_f;
    float zraw_f;

    /* this should be fairly close to the end of the measurement, so the best approximation of the time */
    new_report.timestamp = 0;//hrt_absolute_time();
    new_report.error_count = 0;//perf_event_count(_comms_errors);

    /*
     * @note  We could read the status register here, which could tell us that
     *        we were too early and that the output registers are still being
     *        written.  In the common case that would just slow us down, and
     *        we're better off just never being early.
     */

    /* get measurements from the device */
    ret = read_reg(ADDR_DATA_OUT_X_MSB, (u8 *)&hmc_report, sizeof(hmc_report));
    if (ret < 0) {
        //perf_count(_comms_errors);
        //debug("data/status read error");
        goto out;
    }

    /* swap the data we just received */
    report.x = (((s16)hmc_report.x[0]) << 8) + hmc_report.x[1];
    report.y = (((s16)hmc_report.y[0]) << 8) + hmc_report.y[1];
    report.z = (((s16)hmc_report.z[0]) << 8) + hmc_report.z[1];

    /*
     * If any of the values are -4096, there was an internal math error in the sensor.
     * Generalise this to a simple range check that will also catch some bit errors.
     */
    if ((abs(report.x) > 2048) ||
        (abs(report.y) > 2048) ||
        (abs(report.z) > 2048)) {
        //perf_count(_comms_errors);
        goto out;
    }

    /* get measurements from the device */
    new_report.temperature = 0;

    /*
     * RAW outputs
     *
     * to align the sensor axes with the board, x and y need to be flipped
     * and y needs to be negated
     */
    new_report.x_raw = report.y;
    new_report.y_raw = -report.x;
    /* z remains z */
    new_report.z_raw = report.z;

    /* scale values for output */


    /* the standard external mag by 3DR has x pointing to the
     * right, y pointing backwards, and z down, therefore switch x
     * and y and invert y
     */
    xraw_f = -report.y;
    yraw_f = report.x;
    zraw_f = report.z;

    // apply user specified rotation
    //rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);

    new_report.x = ((xraw_f * _range_scale) - _scale.x_offset) * _scale.x_scale;
    /* flip axes and negate value for y */
    new_report.y = ((yraw_f * _range_scale) - _scale.y_offset) * _scale.y_scale;
    /* z remains z */
    new_report.z = ((zraw_f * _range_scale) - _scale.z_offset) * _scale.z_scale;


    _last_report = new_report;

    /* post a report to the ring */
    if (_mag_reports->force(&new_report)) {
        //perf_count(_buffer_overflows);
    }

    ret = 0;

out:
    //perf_end(_sample_perf);
    return;
}


s32 hmc5883::self_test(void)
{
	f64 angle;
	u32 acr;
    //连续读出HMC5883内部角度数据，地址范围0x3~0x5
    u8 tmp[6];
    s16 data[3];
    if (read_reg(ADDR_DATA_OUT_X_MSB, tmp, 6))
        return -1;
    data[0] = (tmp[0] << 8) | tmp[1];//Combine MSB and LSB of X Data output register
    data[1] = (tmp[2] << 8) | tmp[3];//Combine MSB and LSB of Z Data output register
    data[2] = (tmp[4] << 8) | tmp[5];//Combine MSB and LSB of Y Data output register
    angle = atan2((f64)data[2],(f64)data[0])*(180/3.14159265)+180;//单位：角度 (0~360)
    angle *= 10;
    acr = (u32)angle;
    INF("%s: %f, %d.\n", _devname, angle, acr);

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
		ERR("%s: failed to i2c transfer!\n", _devname);
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
		ERR("%s: failed to i2c transfer!\n", _devname);
        return -1;
	}

    return 0;
}

}
/***********************************************************************
** End of file
***********************************************************************/

