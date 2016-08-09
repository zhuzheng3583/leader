/*******************************Copyright (c)***************************
**
** Porject name:	LeaderUAV-Plus
** Created by:		zhuzheng<happyzhull@163.com>
** Created date:	2015/08/28
** Modified by:
** Modified date:
** Descriptions:
**
***********************************************************************/
#include "mpu6000.h"

namespace driver {

mpu6000::mpu6000(PCSTR devname, s32 devid) :
    device(devname, devid),
    _accel_reports(NULL),
    _accel_range_scale(0.0f),
    _accel_range_m_s2(0.0f),
    _gyro_reports(NULL),
    _gyro_range_scale(0.0f),
    _gyro_range_rad_s(0.0f)

{
    // default accel scale factors
	_accel_scale.x_offset = 0;
	_accel_scale.x_scale  = 1.0f;
	_accel_scale.y_offset = 0;
	_accel_scale.y_scale  = 1.0f;
	_accel_scale.z_offset = 0;
	_accel_scale.z_scale  = 1.0f;

	// default gyro scale factors
	_gyro_scale.x_offset = 0;
	_gyro_scale.x_scale  = 1.0f;
	_gyro_scale.y_offset = 0;
	_gyro_scale.y_scale  = 1.0f;
	_gyro_scale.z_offset = 0;
	_gyro_scale.z_scale  = 1.0f;

	_params.name = "mpu6000_thread";
	_params.priority = 0;
	_params.stacksize = 2048;
	_params.func = (void *)thread::func;
	_params.parg = (thread *)this;

}

mpu6000::~mpu6000(void)
{

}


s32 mpu6000::probe(spi *pspi, gpio *gpio_cs)
{
    ASSERT((pspi != NULL) && (gpio_cs != NULL));

	if (device::probe() < 0) {
		ERR("%s: failed to probe.\n", _devname);
		goto fail0;
	}

    _gpio_cs = gpio_cs;
    _spi = pspi;

	_gpio_cs->set_direction_output();
	_gpio_cs->set_value(true);

	//等待上电稳定
    core::mdelay(2000);
    write_reg8(MPUREG_PWR_MGMT1, 0X80); 	// 复位MPU6050
    core::mdelay(100);
    write_reg8(MPUREG_PWR_MGMT1, 0X00);  	// 唤醒MPU6050

    set_gyro_fsr(2000);                         //陀螺仪传感器,±2000dps
    set_accel_fsr(2);                           //加速度传感器,±2g
    set_sample_rate(50);                        //设置采样率50Hz
    write_reg8(MPUREG_INT_ENABLE, 0X00);    //关闭所有中断
    write_reg8(MPUREG_USER_CTRL, 0X00);     //I2C主模式关闭
    write_reg8(MPUREG_FIFO_ENABLE, 0X00);   //关闭FIFO
    write_reg8(MPUREG_INT_PIN_CFG, 0X80);   //INT引脚低电平有效
    u8 who_am_i = read_reg8(MPUREG_DEVICE_ID);
    core::mdelay(1000);
    if(who_am_i != MPU_I2C_SLAVE_ADDR) {
        // 器件ID不正确
        ERR("Failed to read mpu6000 ID...\n");
        return -1;
    }
    write_reg8(MPUREG_PWR_MGMT1, 0X01);     //设置CLKSEL,PLL X轴为参考
    write_reg8(MPUREG_PWR_MGMT2, 0X00);     // 加速度与陀螺仪都工作
    set_sample_rate(200);                        //设置采样率

#if 0 //test
    s16 gyro[3] = { 0 };
    s16 accel[3] = { 0 };
    while (1) {
        mpu6000::get_gyro_raw(gyro);
        mpu6000::get_accel_raw(accel);
        INF("accel[0]=%d, accel[1]=%d, accel[2]=%d "
            "gyro[0]=%d, gyro[1]=%d, gyro[2]=%d.\n",
            accel[0], accel[1], accel[2],gyro[0], gyro[1], gyro[2]);
    }
#endif

        /* allocate basic report buffers */
	_accel_reports = new ringbuffer(2, sizeof(accel_report));
	if (_accel_reports == NULL)
		return -1;

	_gyro_reports = new ringbuffer(2, sizeof(gyro_report));
	if (_gyro_reports == NULL)
		return -1;

    return 0;

fail0:
    return -1;
}

s32 mpu6000::open(s32 flags)
{
	return 0;
}

s32 mpu6000::read_accel(u8 *buf, u32 size)
{
	u32 count = size / sizeof(accel_report);

	/* buffer must be large enough */
	if (count < 1)
		return -ENOSPC;

	/* if automatic measurement is not enabled, get a fresh measurement into the buffer */
	//if (_call_interval == 0) {
	//	_accel_reports->flush();
	//	measure();
	//}

	/* if no data, error (we could block here) */
	if (_accel_reports->empty())
		return -EAGAIN;

	//perf_count(_accel_reads);

	/* copy reports out of our buffer to the caller */
	accel_report *arp = reinterpret_cast<accel_report *>(buf);
	int transferred = 0;
	while (count--) {
		if (!_accel_reports->get(arp))
			break;
		transferred++;
		arp++;
	}

	/* return the number of bytes transferred */
	return (transferred * sizeof(accel_report));	
}

s32 mpu6000::read_gyro(u8 *buf, u32 size)
{
	unsigned count = size / sizeof(gyro_report);

	/* buffer must be large enough */
	if (count < 1)
		return -ENOSPC;

	/* if automatic measurement is not enabled, get a fresh measurement into the buffer */
	//if (_call_interval == 0) {
	//	_gyro_reports->flush();
	//	measure();
	//}

	/* if no data, error (we could block here) */
	if (_gyro_reports->empty())
		return -EAGAIN;

	//perf_count(_gyro_reads);

	/* copy reports out of our buffer to the caller */
	gyro_report *grp = reinterpret_cast<gyro_report *>(buf);
	int transferred = 0;
	while (count--) {
		if (!_gyro_reports->get(grp))
			break;
		transferred++;
		grp++;
	}

	/* return the number of bytes transferred */
	return (transferred * sizeof(gyro_report));
}

s32 mpu6000::close(void)
{

}

void mpu6000::run(void *parg)
{
    u32 cnt = 0;
	for (cnt = 0; ;cnt++)
	{
		//INF("%s: ...\n", _devname);
        mpu6000::measure();
        msleep(100);
	}
}


s16 mpu6000::s16_from_bytes(u8 bytes[])
{
	union {
		u8    b[2];
		s16    w;
	} u;

	u.b[1] = bytes[0];
	u.b[0] = bytes[1];

	return u.w;
}


s32 mpu6000::init(void)
{
#if 0
	int ret;

	/* do SPI init (and probe) first */
	ret = SPI::init();

	/* if probe/setup failed, bail now */
	if (ret != OK) {
		debug("SPI setup failed");
		return ret;
	}

	/* allocate basic report buffers */
	_accel_reports = new RingBuffer(2, sizeof(accel_report));
	if (_accel_reports == nullptr)
		goto out;

	_gyro_reports = new RingBuffer(2, sizeof(gyro_report));
	if (_gyro_reports == nullptr)
		goto out;

	if (reset() != OK)
		goto out;

	/* Initialize offsets and scales */
	_accel_scale.x_offset = 0;
	_accel_scale.x_scale  = 1.0f;
	_accel_scale.y_offset = 0;
	_accel_scale.y_scale  = 1.0f;
	_accel_scale.z_offset = 0;
	_accel_scale.z_scale  = 1.0f;

	_gyro_scale.x_offset = 0;
	_gyro_scale.x_scale  = 1.0f;
	_gyro_scale.y_offset = 0;
	_gyro_scale.y_scale  = 1.0f;
	_gyro_scale.z_offset = 0;
	_gyro_scale.z_scale  = 1.0f;


	/* do CDev init for the gyro device node, keep it optional */
	ret = _gyro->init();
	/* if probe/setup failed, bail now */
	if (ret != OK) {
		debug("gyro init failed");
		return ret;
	}

	_accel_class_instance = register_class_devname(ACCEL_BASE_DEVICE_PATH);

	measure();

	/* advertise sensor topic, measure manually to initialize valid report */
	struct accel_report arp;
	_accel_reports->get(&arp);

	/* measurement will have generated a report, publish */
	_accel_topic = orb_advertise_multi(ORB_ID(sensor_accel), &arp,
		&_accel_orb_class_instance, (is_external()) ? ORB_PRIO_MAX : ORB_PRIO_HIGH);

	if (_accel_topic < 0) {
		warnx("ADVERT FAIL");
	}


	/* advertise sensor topic, measure manually to initialize valid report */
	struct gyro_report grp;
	_gyro_reports->get(&grp);

	_gyro->_gyro_topic = orb_advertise_multi(ORB_ID(sensor_gyro), &grp,
		&_gyro->_gyro_orb_class_instance, (is_external()) ? ORB_PRIO_MAX : ORB_PRIO_HIGH);

	if (_gyro->_gyro_topic < 0) {
		warnx("ADVERT FAIL");
	}

out:
	return ret;
#endif
	return 0;
}

s32 mpu6000::reset(void)
{
#if 0
	// if the mpu6000 is initialised after the l3gd20 and lsm303d
	// then if we don't do an irqsave/irqrestore here the mpu6000
	// frequenctly comes up in a bad state where all transfers
	// come as zero
	uint8_t tries = 5;
	while (--tries != 0) {
		irqstate_t state;
		state = irqsave();

		write_reg(MPUREG_PWR_MGMT_1, BIT_H_RESET);
		up_udelay(10000);

		// Wake up device and select GyroZ clock. Note that the
		// MPU6000 starts up in sleep mode, and it can take some time
		// for it to come out of sleep
		write_checked_reg(MPUREG_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROZ);
		up_udelay(1000);

		// Disable I2C bus (recommended on datasheet)
		write_checked_reg(MPUREG_USER_CTRL, BIT_I2C_IF_DIS);
		irqrestore(state);

		if (read_reg(MPUREG_PWR_MGMT_1) == MPU_CLK_SEL_PLLGYROZ) {
			break;
		}
		perf_count(_reset_retries);
		usleep(2000);
	}
	if (read_reg(MPUREG_PWR_MGMT_1) != MPU_CLK_SEL_PLLGYROZ) {
		return -EIO;
	}

	usleep(1000);

	// SAMPLE RATE
	_set_sample_rate(_sample_rate);
	usleep(1000);

	// FS & DLPF   FS=2000 deg/s, DLPF = 20Hz (low pass filter)
	// was 90 Hz, but this ruins quality and does not improve the
	// system response
	_set_dlpf_filter(MPU6000_DEFAULT_ONCHIP_FILTER_FREQ);
	usleep(1000);
	// Gyro scale 2000 deg/s ()
	write_checked_reg(MPUREG_GYRO_CONFIG, BITS_FS_2000DPS);
	usleep(1000);

	// correct gyro scale factors
	// scale to rad/s in SI units
	// 2000 deg/s = (2000/180)*PI = 34.906585 rad/s
	// scaling factor:
	// 1/(2^15)*(2000/180)*PI
	_gyro_range_scale = (0.0174532 / 16.4);//1.0f / (32768.0f * (2000.0f / 180.0f) * M_PI_F);
	_gyro_range_rad_s = (2000.0f / 180.0f) * M_PI_F;

	set_accel_range(8);

	usleep(1000);

	// INT CFG => Interrupt on Data Ready
	write_checked_reg(MPUREG_INT_ENABLE, BIT_RAW_RDY_EN);        // INT: Raw data ready
	usleep(1000);
	write_checked_reg(MPUREG_INT_PIN_CFG, BIT_INT_ANYRD_2CLEAR); // INT: Clear on any read
	usleep(1000);

	// Oscillator set
	// write_reg(MPUREG_PWR_MGMT_1,MPU_CLK_SEL_PLLGYROZ);
	usleep(1000);

	return OK;
#endif
	return 0;
}
void mpu6000::measure(void)
{
    struct mpu_report_reg report_reg;

	struct mpu_report {
        	u8		status;
		s16		accel_x;
		s16		accel_y;
		s16		accel_z;
		s16		temp;
		s16		gyro_x;
		s16		gyro_y;
		s16		gyro_z;
	} report;

	/*
	 * Fetch the full set of measurements from the MPU6000 in one pass.
	 */
	if (mpu6000::read_reg(MPUREG_INT_STATUS, (u8 *)&report_reg, sizeof(report_reg)))
        return;

	/*
	 * Convert from big to little endian
	 */
	report.accel_x = s16_from_bytes(report_reg.accel_x);
	report.accel_y = s16_from_bytes(report_reg.accel_y);
	report.accel_z = s16_from_bytes(report_reg.accel_z);

	report.temp = s16_from_bytes(report_reg.temp);

	report.gyro_x = s16_from_bytes(report_reg.gyro_x);
	report.gyro_y = s16_from_bytes(report_reg.gyro_y);
	report.gyro_z = s16_from_bytes(report_reg.gyro_z);

	if (report.accel_x == 0 &&
	    report.accel_y == 0 &&
	    report.accel_z == 0 &&
	    report.temp == 0 &&
	    report.gyro_x == 0 &&
	    report.gyro_y == 0 &&
	    report.gyro_z == 0) {
		// all zero data - probably a SPI bus error
                // note that we don't call reset() here as a reset()
                // costs 20ms with interrupts disabled. That means if
                // the mpu6k does go bad it would cause a FMU failure,
                // regardless of whether another sensor is available,
		return;
	}


	/*
	 * Swap axes and negate y
	 */
#if 0
	s16 accel_xt = report.accel_y;
	s16 accel_yt = ((report.accel_x == -32768) ? 32767 : -report.accel_x);

	s16 gyro_xt = report.gyro_y;
	s16 gyro_yt = ((report.gyro_x == -32768) ? 32767 : -report.gyro_x);

	/*
	 * Apply the swap
	 */
	report.accel_x = accel_xt;
	report.accel_y = accel_yt;
	report.gyro_x = gyro_xt;
	report.gyro_y = gyro_yt;
#endif

	/*
	 * Report buffers.
	 */
	accel_report	arb;
	gyro_report	grb;

	/*
	 * Adjust and scale results to m/s^2.
	 */
	grb.timestamp = arb.timestamp = 0;//hrt_absolute_time();

	// report the error count as the sum of the number of bad
	// transfers and bad register reads. This allows the higher
	// level code to decide if it should use this sensor based on
	// whether it has had failures
    grb.error_count = arb.error_count = 0;//= perf_event_count(_bad_transfers) + perf_event_count(_bad_registers);

	/*
	 * 1) Scale raw value to SI units using scaling from datasheet.
	 * 2) Subtract static offset (in SI units)
	 * 3) Scale the statically calibrated values with a linear
	 *    dynamically obtained factor
	 *
	 * Note: the static sensor offset is the number the sensor outputs
	 * 	 at a nominally 'zero' input. Therefore the offset has to
	 * 	 be subtracted.
	 *
	 *	 Example: A gyro outputs a value of 74 at zero angular rate
	 *	 	  the offset is 74 from the origin and subtracting
	 *		  74 from all measurements centers them around zero.
	 */


	/* NOTE: Axes have been swapped to match the board a few lines above. */

	arb.x_raw = report.accel_x;
	arb.y_raw = report.accel_y;
	arb.z_raw = report.accel_z;

	f32 x_in_new = ((report.accel_x * _accel_range_scale) - _accel_scale.x_offset) * _accel_scale.x_scale;
	f32 y_in_new = ((report.accel_y * _accel_range_scale) - _accel_scale.y_offset) * _accel_scale.y_scale;
	f32 z_in_new = ((report.accel_z * _accel_range_scale) - _accel_scale.z_offset) * _accel_scale.z_scale;


	arb.x = 0;//_accel_filter_x.apply(x_in_new);
	arb.y = 0;//_accel_filter_y.apply(y_in_new);
	arb.z = 0;//_accel_filter_z.apply(z_in_new);

	// apply user specified rotation
	//rotate_3f(_rotation, arb.x, arb.y, arb.z);

	arb.scaling = _accel_range_scale;
	arb.range_m_s2 = _accel_range_m_s2;

	_last_temperature = (report.temp) / 361.0f + 35.0f;

	arb.temperature_raw = report.temp;
	arb.temperature = _last_temperature;

	grb.x_raw = report.gyro_x;
	grb.y_raw = report.gyro_y;
	grb.z_raw = report.gyro_z;

	float x_gyro_in_new = ((report.gyro_x * _gyro_range_scale) - _gyro_scale.x_offset) * _gyro_scale.x_scale;
	float y_gyro_in_new = ((report.gyro_y * _gyro_range_scale) - _gyro_scale.y_offset) * _gyro_scale.y_scale;
	float z_gyro_in_new = ((report.gyro_z * _gyro_range_scale) - _gyro_scale.z_offset) * _gyro_scale.z_scale;

	grb.x = 0;//_gyro_filter_x.apply(x_gyro_in_new);
	grb.y = 0;//_gyro_filter_y.apply(y_gyro_in_new);
	grb.z = 0;//_gyro_filter_z.apply(z_gyro_in_new);

	// apply user specified rotation
	//rotate_3f(_rotation, grb.x, grb.y, grb.z);

	grb.scaling = _gyro_range_scale;
	grb.range_rad_s = _gyro_range_rad_s;

	grb.temperature_raw = report.temp;
	grb.temperature = _last_temperature;

	_accel_reports->force(&arb);
	_gyro_reports->force(&grb);

    return;
}

/*
 * 设置MPU6000陀螺仪传感器满量程范围
 * fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
 * return:成功返回0
 */
s32 mpu6000::set_gyro_fsr(u16 fsr)
{
    s32 ret = 0;
    u8 data;

    switch (fsr) {
    case 250:
        data = (u8)(GYRO_FSR_250DPS << 3);
        break;
    case 500:
        data = (u8)(GYRO_FSR_500DPS << 3);
        break;
    case 1000:
        data = (u8)(GYRO_FSR_1000DPS << 3);
        break;
    case 2000:
        data = (u8)(GYRO_FSR_2000DPS << 3);
        break;
    default:
        return -1;
    }

    ret = write_reg8(MPUREG_GYRO_CONFIG, data);
    if (ret) {
        return -1;
    }

    return 0;
}

/*
 * 设置MPU6000加速度传感器满量程范围
 * fsr:0,±2g;1,±4g;2,±8g;3,±16g
 * return:成功返回0
 */
s32 mpu6000::set_accel_fsr(u8 fsr)
{
    s32 ret = 0;
    u8 data;

    switch (fsr) {
    case 2:
        data = (u8)(ACCEL_FSR_2G << 3);
        break;
    case 4:
        data = (u8)(ACCEL_FSR_4G << 3);
        break;
    case 8:
        data = (u8)(ACCEL_FSR_8G << 3);
        break;
    case 16:
        data = (u8)(ACCEL_FSR_16G << 3);
        break;
    default:
        return -1;
    }

    ret = write_reg8(MPUREG_ACCEL_CONFIG, data);
    if (ret) {
        return -1;
    }

    return 0;
}

#if 0
/*
 * 设置MPU6000的数字低通滤波器
 * lpf:数字低通滤波频率(Hz)
 * return:成功返回0
 */
s32 mpu6000::set_lpf(u16 lpf)
{
    s32 ret = 0;
    u8 data;

    if (lpf >= 188)
        data = INV_FILTER_188HZ;
    else if (lpf >= 98)
        data = INV_FILTER_98HZ;
    else if (lpf >= 42)
        data = INV_FILTER_42HZ;
    else if (lpf >= 20)
        data = INV_FILTER_20HZ;
    else if (lpf >= 10)
        data = INV_FILTER_10HZ;
    else
        data = INV_FILTER_5HZ;

    ret = reg_write_byte(LPF, &data);
    if (ret) {
        return -1;
    }

    return 0;
}
#endif

/*
 * 设置MPU6000的采样率(假定Fs=1KHz)
 * rate:4~1000(Hz)
 * return:成功返回0
 */
s32 mpu6000::set_sample_rate(u16 rate)
{
    s32 ret = 0;
    u8 data = 0;

    if (rate < 4) {
        rate = 4;
    }
    else if (rate > 1000) {
        rate = 1000;
    }

    data = 1000 / rate - 1;
    ret = write_reg8(MPUREG_SAMPLE_RATE_DIV, data);
    if (ret) {
        return -1;
    }

    return 0;
}


/**
 *  @brief      Read raw gyro data directly from the registers.
 *  @param[out] gyro        Raw data in hardware units.
 *  @return     0 if successful.
 *  @note		gyro_x gyro_y gyro_z = [-32768, 32767]
 */
s32 mpu6000::get_gyro_raw(s16 *gyro)
{
    u8 tmp[6];
	if (mpu6000::read_reg(MPUREG_GYRO_XOUTH, tmp, 6))
        return -1;
    gyro[0] = (tmp[0] << 8) | tmp[1];
    gyro[1] = (tmp[2] << 8) | tmp[3];
    gyro[2] = (tmp[4] << 8) | tmp[5];

    return 0;
}

/**
 *  @brief      Read raw accel data directly from the registers.
 *  @param[out] data        Raw data in hardware units.
 *  @return     0 if successful.
 *  @note		accel_x accel_y accel_z = [-32768, 32767]
 */
s32 mpu6000::get_accel_raw(s16 *accel)
{
    u8 tmp[6];
	if (mpu6000::read_reg(MPUREG_ACCEL_XOUTH, tmp, 6))
        return -1;

    accel[0] = (tmp[0] << 8) | tmp[1];
    accel[1] = (tmp[2] << 8) | tmp[3];
    accel[2] = (tmp[4] << 8) | tmp[5];

    return 0;
}

/**
 *  @brief      Read temperature data directly from the registers.
 *  @param[out] temperature	Data in q16 format.
 *  @return     0 if successful.
 */
s32 mpu6000::get_temperature(f32 *temperature)
{
    u8 tmp[2];
    s16 raw;
	if (mpu6000::read_reg(MPUREG_TEMP_OUTH, tmp, 2))
        return -1;
    raw = (tmp[0]<<8) | tmp[1];
    //data[0] = (long)((35 + ((raw - (float)st.hw->temp_offset) / st.hw->temp_sens)) * 65536L);
	temperature[0] = 36.53 + ((double)raw)/340;
	return 0;
}



s32 mpu6000::read_reg8(u8 reg)
{
    s32 ret = 0;
    u8 data = 0;
    mpu6000::read_reg(reg, &data, 1);
    if (ret < 0) {
        return -1;
    }

    return ((s32)data);
}


s32 mpu6000::write_reg8(u8 reg, u8 data)
{
    s32 ret = 0;
    ret = mpu6000::write_reg(reg, &data, 1);
    if (ret < 0) {
        return -1;
    }

    return 0;
}

s32 mpu6000::read_reg(u8 reg, u8 *buf, u8 len)
{
    s32 ret = 0;
    reg = READ_CMD | reg;
    _gpio_cs->set_value(false);
    //core::mdelay(10);
    struct spi_msg msgs[] = {
        [0] = {
            .buf = &reg,
            .len = sizeof(reg),
            .flags = 0,
        },
        [1] = {
            .buf = buf,
            .len = len,
            .flags = SPI_M_RD,
        },
    };
    ret = _spi->transfer(&msgs[0]);
    if (ret < 0) {
        goto fail0;
    }
    ret = _spi->transfer(&msgs[1]);
    if (ret < 0) {
        goto fail1;
    }
    //core::mdelay(10);
    _gpio_cs->set_value(true);

    return 0;

fail1:
fail0:
    _gpio_cs->set_value(true);
    return -1;
}

s32 mpu6000::write_reg(u8 reg, u8 *buf, u8 len)
{
    s32 ret = 0;
    reg = WRITE_CMD & reg;
    _gpio_cs->set_value(false);
    //core::mdelay(10);
    struct spi_msg msgs[] = {
        [0] = {
            .buf = &reg,
            .len = sizeof(reg),
            .flags = 0,
        },
        [1] = {
            .buf = buf,
            .len = len,
            .flags = 0,
        },
    };
    ret = _spi->transfer(&msgs[0]);
    if (ret < 0) {
        goto fail0;
    }
    ret = _spi->transfer(&msgs[1]);
    if (ret < 0) {
        goto fail1;
    }
    //core::mdelay(10);
    _gpio_cs->set_value(true);

    return 0;

fail1:
fail0:
    _gpio_cs->set_value(true);
    return -1;
}


}

#if 0
#include "mpu6000.h"
#include "core.h"

#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

#include <math.h>

#define MPU_USER   0

namespace driver {


mpu6000::mpu6000(PCSTR name, u32 chip_select, spi *spi) :
    _name(name),
    _chip_select(chip_select),
    _spi(spi),
    _gpio(NULL),
    _max_fifo(1024)
{
    _cfg.gyro_fsr         = 2000;         // Matches gyro_cfg >> 3 & 0x03
    _cfg.accel_fsr        = 2;            // Matches accel_cfg >> 3 & 0x03
    _cfg.sensors          = INV_XYZ_GYRO|INV_XYZ_ACCEL; // Enabled sensors. Uses same masks as fifo_en, NOT pwr_mgmt_2.
    _cfg.lpf              = 42;            // Matches config register.
    _cfg.clk_src          = 0;
    _cfg.sample_rate      = 50;   			// Sample rate, NOT rate divider.
    _cfg.fifo_enable      = 1;            // Matches fifo_en register.
    _cfg.int_enable       = 0;            // Matches int enable register.
    _cfg.bypass_mode      = 0;            // 1 if devices on auxiliary I2C bus appear on the primary.

    _cfg.accel_half       = 0;            // 1 if half-sensitivity.
    _cfg.lp_accel_mode    = 0;            // 1 if device in low-power accel-only mode.
    _cfg.int_motion_only  = 0;            // 1 if interrupts are only triggered on motion events.
    _cfg.active_low_int   = 0;            // 1 for active low interrupts.
    _cfg.latched_int      = 0;            // 1 for latched interrupts.
    _cfg.dmp_on           = 0;            // 1 if DMP is enabled.
    _cfg.dmp_loaded       = 0;            // Ensures that DMP will only be loaded once.
    _cfg.dmp_sample_rate  = 0;            // Sampling rate used when DMP is enabled.

    s8 gyro_orientation[9] = {
        1, 0, 0,
        0, 1, 0,
        0, 0, 1
    };

}

mpu6000::~mpu6000(void)
{

}


s32 mpu6000::open(s32 flags)
{
	s32 ret = 0;
    /**
     * @brief Init gpio as chip select
     */
    _gpio = _spi->chip_select_init(_chip_select);
    //_spi->chip_select_deactive(_gpio);

    _gpio_s = _gpio;
    _spi_s = _spi;

#if MPU_USER
    reg_write_byte(MPUREG_PWR_MGMT1, 0X80); 	// 复位MPU6050
    core::mdelay(100);
    reg_write_byte(MPUREG_PWR_MGMT1, 0X00);  	// 唤醒MPU6050
    u8 who_am_i = reg_read_byte(MPUREG_DEVICE_ID);
    if(who_am_i != MPU_I2C_SLAVE_ADDR) {
        // 器件ID不正确
        ERR("Failed to read mpu6000 ID...\n");
        return -1;
    }

	if (mpu_set_sensors(_cfg.sensors))
        return -1;

    set_gyro_fsr(2000);                         //陀螺仪传感器,±2000dps
    set_accel_fsr(2);                           //加速度传感器,±2g
    set_sample_rate(10);                        //设置采样率50Hz
    reg_write_byte(MPUREG_INT_ENABLE, 0X00);    //关闭所有中断
    reg_write_byte(MPUREG_USER_CTRL, 0X00);     //I2C主模式关闭
    if (_cfg.fifo_enable) {
		if (mpu_configure_fifo(_cfg.sensors)) 	// INV_XYZ_GYRO|INV_XYZ_ACCEL
        	return -1;
		reg_write_byte(MPUREG_FIFO_ENABLE, _cfg.sensors);   //开启FIFO
    } else
        reg_write_byte(MPUREG_FIFO_ENABLE, 0X00);   //关闭FIFO

    reg_write_byte(MPUREG_INT_PIN_CFG, 0X80);   //INT引脚低电平有效

    reg_write_byte(MPUREG_PWR_MGMT1, 0X01);     //设置CLKSEL,PLL X轴为参考
    reg_write_byte(MPUREG_PWR_MGMT2, 0X00);     // 加速度与陀螺仪都工作
    //set_sample_rate(50);                        //设置采样率为50Hz
#else
    reg_write_byte(MPUREG_PWR_MGMT1, 0X80); 	// 复位MPU6050
    core::mdelay(100);
    reg_write_byte(MPUREG_PWR_MGMT1, 0X00);  	// 唤醒MPU6050

    if (mpu_init(NULL))
        return -1;

    u8 who_am_i = reg_read_byte(MPUREG_DEVICE_ID);
    if(who_am_i != MPU_I2C_SLAVE_ADDR) {
        // 器件ID不正确
        ERR("Failed to read mpu6000 ID...\n");
        return -1;
    }

    if (mpu_set_sensors(_cfg.sensors))
        return -1;
    if (mpu_set_gyro_fsr(_cfg.gyro_fsr)) 		// ±2000dps 陀螺仪传感器,
        return -1;
    if (mpu_set_accel_fsr(_cfg.accel_fsr)) 	// ±2g 加速度传感器,
        return -1;
    if (mpu_set_lpf(_cfg.lpf)) 				// 42
        return -1;
    if (mpu_set_sample_rate(_cfg.sample_rate))	// 50Hz 设置采样率
        return -1;
	if (_cfg.fifo_enable) {					// 设置FIFO
		if (mpu6000::configure_fifo(_cfg.sensors)) 	// INV_XYZ_GYRO|INV_XYZ_ACCEL
        	return -1;
		//reg_write_byte(MPUREG_FIFO_ENABLE, _cfg.sensors);   //开启FIFO
	} else {
		if (mpu_configure_fifo(0))
        	return -1;
	}

    //if (mpu_set_sensors(_cfg.sensors))
    //    return -1;

    reg_write_byte(MPUREG_PWR_MGMT1, 0X01);     //设置CLKSEL,PLL X轴为参考
    reg_write_byte(MPUREG_PWR_MGMT2, 0X00);     // 加速度与陀螺仪都工作

	u8 buf[2];
	reg_read(MPUREG_FIFO_COUNT_H, 2, buf);

    if (_cfg.dmp_on) {
        // 加载dmp固件
        if (dmp_load_motion_driver_firmware())
            return -1;

        // 设置陀螺仪方向
        if (dmp_set_orientation(orientation_matrix_to_scalar(gyro_orientation)))
            return -1;

        // 设置dmp功能
        if (dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT|DMP_FEATURE_TAP
            |DMP_FEATURE_ANDROID_ORIENT| DMP_FEATURE_SEND_RAW_ACCEL
            |DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL))
            return -1;

        // 设置DMP输出速率(最大不超过200Hz)
        if (dmp_set_fifo_rate(MPU_OUT_HZ))
            return -1;
    }

    // 自检
    //if (mpu6000::chip_self_test())
    //    return -1;

    if (_cfg.dmp_on)  {
         // 使能DMP
        if (mpu_set_dmp_state(1))
            return -1;
    }

#endif
#if 0
	s16 data[6];
    unsigned char sensors = 0;
    unsigned char more = 0;
    core::mdelay(100);
    reg_read(MPUREG_FIFO_COUNT_H, 2, (u8 *)&data[0]);
    if (_cfg.fifo_enable) {
		while(1) {
            //mpu6000::get_accel_raw((u16 *)&data[0]);
           // mpu6000::get_gyro_raw((u16 *)&data[3]);
			if (mpu6000::read_fifo(&data[3], &data[0])) {
            //if (read_fifo((struct mpu_fifo_data *)buf, count / 12) {
			//if (mpu_read_fifo(&data[3], &data[0], NULL, &sensors, &more)) {
				ERR("Failed to read mpu6000...\n");
            }
			INF("\rax=%6.5d, ay=%6.5d, az=%6.5d, gx=%6.5d, gy=%6.5d, gz=%6.5d.",
			data[0], data[1], data[2], data[3], data[4], data[5]);
		}
	}
#endif


    return 0;

err:
	ERR("Failed to open %s.\n", _name);
    return -1;
}

s32 mpu6000::read(s8 *buf, u32 count)
{
    ASSERT(count % 12 == 0);
    u32 readcnt = 0;

    if (_cfg.dmp_on)
    {
        WRN("No support DMP...\n");
        //f32 pitch,roll,yaw; // 欧拉角
        //mpu_dmp_get_data(&pitch, &roll, &yaw);
    }
    else if (_cfg.fifo_enable)
    {
        unsigned char sensors = 0;
        unsigned char more = 0;
        // buf{ax,ay,az, gx,gy,gz}
        for (readcnt = 0; readcnt < count; readcnt += count) {
            if (read_fifo((struct mpu_fifo_packet *)buf, count / 12)) {
            //if (mpu_read_fifo((s16 *)&data[6], (s16 *)&data[0], NULL, &sensors, &more)) {
                ERR("Failed to read mpu6000...\n");
                return readcnt;
            }
         }
    }
    else
    {
        for (readcnt = 0; readcnt < count; readcnt += 12) {
            if (get_accel_raw((s16 *)&buf[readcnt]) ||
                get_gyro_raw((s16 *)&buf[readcnt+6]))
            {
                ERR("Failed to read mpu6000...\n");
                return readcnt;
            }
        }
    }

    return readcnt;
}


s32 mpu6000::self_test(void)
{
    INF("mpu6000 self test Start...\n");
    u32 testcnt = 500;
    s16 buf[120];
    struct mpu_fifo_packet fifo_packet[20];

    for (u32 n = 0; /*n < testcnt*/; n++)
    {
        if (mpu6000::read((s8 *)fifo_packet, sizeof(fifo_packet)) != sizeof(fifo_packet)) {
            ERR("Failed to self test read.\n");
            //return -1;
        }
        #if 0
        for (u32 i = 0; i < 20; i++)
        {
            INF("\rax=%6.5d, ay=%6.5d, az=%6.5d, gx=%6.5d, gy=%6.5d, gz=%6.5d.",
            fifo_packet[i].acce_x,
            fifo_packet[i].acce_y,
            fifo_packet[i].acce_z,
            fifo_packet[i].gyro_x,
            fifo_packet[i].gyro_y,
            fifo_packet[i].gyro_z
            );
        }
        #endif
    }
    INF("mpu6000 self test End...\n");

    return 0;
}

s32 mpu6000::chip_self_test(void)
{
	s32 ret;
	long gyro[3], accel[3];
	ret = mpu_run_self_test(gyro, accel);
	if (ret != 0x3) {
        ERR("Failed to mpu6000 mpu_run_self_test...\n");
        return -1;
	}

    /**
     * Test passed. We can trust the gyro data here, so let's push it down
     * to the DMP.
     */
    float sens;
    unsigned short accel_sens;

    mpu_get_gyro_sens(&sens);
    gyro[0] = (long)(gyro[0] * sens);
    gyro[1] = (long)(gyro[1] * sens);
    gyro[2] = (long)(gyro[2] * sens);
    dmp_set_gyro_bias(gyro);

    mpu_get_accel_sens(&accel_sens);
    accel[0] *= accel_sens;
    accel[1] *= accel_sens;
    accel[2] *= accel_sens;
    dmp_set_accel_bias(accel);
    return 0;
}


// Get one packet from the FIFO.
s32 mpu6000::read_fifo(struct mpu_fifo_packet *data, u32 packet_cnt)
{
    /* Assumes maximum packet size is gyro (6) + accel (6). */
    u8 tmp[2];
    u16 fifo_count;
    u32 packet_size = packet_cnt * sizeof(struct mpu_fifo_packet);

    if (mpu6000::reg_read(MPUREG_FIFO_COUNT_H, 2, tmp))
		return -1;
    fifo_count = (tmp[0] << 8) | tmp[1];
    if (fifo_count < packet_size) {
		WRN("mpu fifo under flow, count: %hd.\n", fifo_count);
        return -1;
	}

    if (fifo_count > (_max_fifo >> 1)) {
        /* FIFO is 50% full, better check overflow bit. */
        if (mpu6000::reg_read(MPUREG_INT_STATUS, 1, tmp))
			return -1;
		DBG("mpu fifo count: %hd.\n", fifo_count);
        if (tmp[0] & BIT_FIFO_OVERFLOW) {
            mpu6000::reset_fifo();
			WRN("mpu fifo over flow, count: %hd.\n", fifo_count);
            return -2;
        }
    }

    if (mpu6000::reg_read(MPUREG_FIFO_RW, packet_size, (u8 *)data))
		return -1;

#if 1
    for (u32 n = 0; n < packet_cnt; n++)
    {
        u8 *tmp_data = (u8 *)&data[n];

        data[n].acce_x = (tmp_data[0] << 8) | tmp_data[1];
        data[n].acce_y = (tmp_data[2] << 8) | tmp_data[3];
        data[n].acce_z = (tmp_data[4] << 8) | tmp_data[5];
        data[n].gyro_x = (tmp_data[6] << 8) | tmp_data[7];
        data[n].gyro_y = (tmp_data[8] << 8) | tmp_data[9];
        data[n].gyro_z = (tmp_data[10] << 8) | tmp_data[11];

		INF("\rax=%6.5d, ay=%6.5d, az=%6.5d, gx=%6.5d, gy=%6.5d, gz=%6.5d.",
			data[n].acce_x,
			data[n].acce_y,
			data[n].acce_z,
			data[n].gyro_x,
			data[n].gyro_y,
			data[n].gyro_z
        );
    }
#if 0
    for (u32 i = 0; i < 20; i++)
    {
        INF("\rax=%6.5d, ay=%6.5d, az=%6.5d, gx=%6.5d, gy=%6.5d, gz=%6.5d.",
            fifo_packet[i].acce_x,
            fifo_packet[i].acce_y,
            fifo_packet[i].acce_z,
            fifo_packet[i].gyro_x,
            fifo_packet[i].gyro_y,
            fifo_packet[i].gyro_z
        );
    }
#endif
#endif

#if 0

    accel[0] = (data[0] << 8) | data[1];
    accel[1] = (data[2] << 8) | data[3];
    accel[2] = (data[4] << 8) | data[5];
    gyro[0]  = (data[6] << 8) | data[7];
    gyro[1]  = (data[8] << 8) | data[9];
    gyro[2]  = (data[10] << 8) | data[11];
#endif

    return 0;
}


/**
 *  @brief  Reset FIFO read/write pointers.
 *  @return 0 if successful.
 */
s32 mpu6000::reset_fifo(void)
{
    u8 data;

    data = 0;
    if (mpu6000::reg_write(MPUREG_INT_ENABLE, 1, &data))
        return -1;
    if (mpu6000::reg_write(MPUREG_FIFO_ENABLE, 1, &data))
        return -1;
    if (mpu6000::reg_write(MPUREG_USER_CTRL, 1, &data))
        return -1;

    data = BIT_FIFO_RST;
    if (mpu6000::reg_write(MPUREG_USER_CTRL, 1, &data))
        return -1;
    data = BIT_FIFO_EN;
    if (mpu6000::reg_write(MPUREG_USER_CTRL, 1, &data))
        return -1;
    core::mdelay(50);
    data = BIT_DATA_RDY_EN;
    if (mpu6000::reg_write(MPUREG_INT_ENABLE, 1, &data))
        return -1;
	core::mdelay(50);
    data = _cfg.sensors;
    if (mpu6000::reg_write(MPUREG_FIFO_ENABLE, 1, &data))
        return -1;

    return 0;
}

/**
 *  @brief      Select which sensors are pushed to FIFO.
 *  @e sensors can contain a combination of the following flags:
 *  \n INV_X_GYRO, INV_Y_GYRO, INV_Z_GYRO
 *  \n INV_XYZ_GYRO
 *  \n INV_XYZ_ACCEL
 *  @param[in]  sensors Mask of sensors to push to FIFO.
 *  @return     0 if successful.
 */
s32 mpu6000::configure_fifo(u8 sensors)
{
    if (sensors) {
        mpu6000::set_int_enable(1);
        if (mpu6000::reset_fifo())
            return -1;

        _cfg.fifo_enable = sensors;
    }

    return 0;
}

/**
 *  @brief      Enable/disable data ready interrupt.
 *  If the DMP is on, the DMP interrupt is enabled. Otherwise, the data ready
 *  interrupt is used.
 *  @param[in]  enable      1 to enable interrupt.
 *  @return     0 if successful.
 */
s32 mpu6000::set_int_enable(u8 enable)
{
    u8 tmp;
    if (enable)
        tmp = BIT_DATA_RDY_EN;
    else
        tmp = 0x00;

    if (mpu6000::reg_write(MPUREG_INT_ENABLE, 1, &tmp))
        return -1;

    _cfg.int_enable = tmp;

    return 0;
}


/*
 * 设置MPU6000陀螺仪传感器满量程范围
 * fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
 * return:成功返回0
 */
s8 mpu6000::set_gyro_fsr(u16 fsr)
{
    return mpu_set_gyro_fsr(fsr);
}

/*
 * 设置MPU6000加速度传感器满量程范围
 * fsr:0,±2g;1,±4g;2,±8g;3,±16g
 * return:成功返回0
 */
s8 mpu6000::set_accel_fsr(u8 fsr)
{
    return mpu_set_accel_fsr(fsr);
}

/*
 * 设置MPU6000的数字低通滤波器
 * lpf:数字低通滤波频率(Hz)
 * return:成功返回0
 */
s8 mpu6000::set_lpf(u16 lpf)
{
    return mpu_set_lpf(lpf);
}


/*
 * 设置MPU6000的采样率(假定Fs=1KHz)
 * rate:4~1000(Hz)
 * return:成功返回0
 */
s8 mpu6000::set_sample_rate(u16 rate)
{
    return mpu_set_sample_rate(rate);
}



/**
 *  @brief      Read raw gyro data directly from the registers.
 *  @param[out] gyro        Raw data in hardware units.
 *  @return     0 if successful.
 *  @note		gyro_x gyro_y gyro_z = [-32768, 32767]
 */
s32 mpu6000::get_gyro_raw(s16 *gyro)
{
    u8 tmp[6];
	if (mpu6000::reg_read(MPUREG_GYRO_XOUTH, 6, tmp))
        return -1;
    gyro[0] = (tmp[0] << 8) | tmp[1];
    gyro[1] = (tmp[2] << 8) | tmp[3];
    gyro[2] = (tmp[4] << 8) | tmp[5];

    return 0;
}

/**
 *  @brief      Read raw accel data directly from the registers.
 *  @param[out] data        Raw data in hardware units.
 *  @return     0 if successful.
 *  @note		accel_x accel_y accel_z = [-32768, 32767]
 */
s32 mpu6000::get_accel_raw(s16 *accel)
{
    u8 tmp[6];
	if (mpu6000::reg_read(MPUREG_ACCEL_XOUTH, 6, tmp))
        return -1;

    accel[0] = (tmp[0] << 8) | tmp[1];
    accel[1] = (tmp[2] << 8) | tmp[3];
    accel[2] = (tmp[4] << 8) | tmp[5];

    return 0;
}

/**
 *  @brief      Read temperature data directly from the registers.
 *  @param[out] temperature	Data in q16 format.
 *  @return     0 if successful.
 */
s32 mpu6000::get_temperature(f32 *temperature)
{
    u8 tmp[2];
    s16 raw;
	if (mpu6000::reg_read(MPUREG_TEMP_OUTH, 2, tmp))
        return -1;
    raw = (tmp[0]<<8) | tmp[1];
    //data[0] = (long)((35 + ((raw - (float)st.hw->temp_offset) / st.hw->temp_sens)) * 65536L);
	temperature[0] = 36.53 + ((double)raw)/340;
	return 0;
}


//陀螺仪方向控制
u16 mpu6000::orientation_matrix_to_scalar(s8 *mtx)
{
    u16 scalar;
    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */
    scalar =  mpu6000::row_2_scale(mtx);
    scalar |= mpu6000::row_2_scale(mtx + 3) << 3;
    scalar |= mpu6000::row_2_scale(mtx + 6) << 6;

    return scalar;
}

//方向转换
u16 mpu6000::row_2_scale(s8 *row)
{
    u16 b;

    if (row[0] > 0) b = 0;
    else if (row[0] < 0)    b = 4;
    else if (row[1] > 0)    b = 1;
    else if (row[1] < 0)    b = 5;
    else if (row[2] > 0)    b = 2;
    else if (row[2] < 0)    b = 6;
    else    b = 7;      // error
    return b;
}

//得到dmp处理后的数据(注意,本函数需要比较多堆栈,局部变量有点多)
//pitch:俯仰角 精度:0.1°   范围:-90.0° <---> +90.0°
//roll:横滚角  精度:0.1°   范围:-180.0°<---> +180.0°
//yaw:航向角   精度:0.1°   范围:-180.0°<---> +180.0°
//返回值:0,正常
//    其他,失败
s8 mpu6000::dmp_get_data(f32 *pitch, f32 *roll, f32 *yaw)
{
	s8 ret = 0;
	f32 q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
	unsigned long sensor_timestamp;
	s16 gyro[3], accel[3], sensors;
	u8 more;
	long quat[4];
	ret = dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors,&more);
	if(ret) return -1;
	/* Gyro and accel data are written to the FIFO by the DMP in chip frame and hardware units.
	 * This behavior is convenient because it keeps the gyro and accel outputs of dmp_read_fifo and mpu_read_fifo consistent.
	**/
	/*if (sensors & INV_XYZ_GYRO )
	send_packet(PACKET_TYPE_GYRO, gyro);
	if (sensors & INV_XYZ_ACCEL)
	send_packet(PACKET_TYPE_ACCEL, accel); */
	/* Unlike gyro and accel, quaternions are written to the FIFO in the body frame, q30.
	 * The orientation is set by the scalar passed to dmp_set_orientation during initialization.
	**/
	if(sensors&INV_WXYZ_QUAT)
	{
		return -2;
	}
	q0 = quat[0] / q30;	// q30格式转换为浮点数
	q1 = quat[1] / q30;
	q2 = quat[2] / q30;
	q3 = quat[3] / q30;
	// 计算得到俯仰角/横滚角/航向角
	*pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3;	// pitch
	*roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3;	// roll
	*yaw   = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;	//yaw

	return 0;
}

}


/**
 *  struct mpu6050_sensor - Cached chip configuration data
 *  @client:		I2C client
 *  @dev:		device structure
 *  @accel_dev:		accelerometer input device structure
 *  @gyro_dev:		gyroscope input device structure
 *  @accel_cdev:		sensor class device structure for accelerometer
 *  @gyro_cdev:		sensor class device structure for gyroscope
 *  @pdata:	device platform dependent data
 *  @op_lock:	device operation mutex
 *  @chip_type:	sensor hardware model
 *  @accel_poll_work:	accelerometer delay work structur
 *  @gyro_poll_work:	gyroscope delay work structure
 *  @fifo_flush_work:	work structure to flush sensor fifo
 *  @reg:		notable slave registers
 *  @cfg:		cached chip configuration data
 *  @axis:	axis data reading
 *  @gyro_poll_ms:	gyroscope polling delay
 *  @accel_poll_ms:	accelerometer polling delay
 *  @accel_latency_ms:	max latency for accelerometer batching
 *  @gyro_latency_ms:	max latency for gyroscope batching
 *  @accel_en:	accelerometer enabling flag
 *  @gyro_en:	gyroscope enabling flag
 *  @use_poll:		use polling mode instead of  interrupt mode
 *  @motion_det_en:	motion detection wakeup is enabled
 *  @batch_accel:	accelerometer is working on batch mode
 *  @batch_gyro:	gyroscope is working on batch mode
 *  @vlogic:	regulator data for Vlogic
 *  @vdd:	regulator data for Vdd
 *  @vi2c:	I2C bus pullup
 *  @enable_gpio:	enable GPIO
 *  @power_enabled:	flag of device power state
 *  @pinctrl:	pinctrl struct for interrupt pin
 *  @pin_default:	pinctrl default state
 *  @pin_sleep:	pinctrl sleep state
 *  @flush_count:	number of flush
 *  @fifo_start_ns:		timestamp of first fifo data
 */
struct mpu6050_sensor {
    const char *name;
	struct i2c_client *client;
	struct device *dev;
	struct hrtimer gyro_timer;
	struct hrtimer accel_timer;
	struct input_dev *accel_dev;
	struct input_dev *gyro_dev;
	struct sensors_classdev accel_cdev;
	struct sensors_classdev gyro_cdev;
	struct mpu6050_platform_data *pdata;
	struct mutex op_lock;
	enum inv_devices chip_type;
	struct workqueue_struct *data_wq;
	struct delayed_work accel_poll_work;
	struct delayed_work gyro_poll_work;
	struct delayed_work fifo_flush_work;
	struct mpu_reg_map reg;
	struct mpu_chip_config cfg;
	struct axis_data axis;
	struct cali_data cali;
	u32 gyro_poll_ms;
	u32 accel_poll_ms;
	u32 accel_latency_ms;
	u32 gyro_latency_ms;
	atomic_t accel_en;
	atomic_t gyro_en;
	bool use_poll;
	bool motion_det_en;
	bool batch_accel;
	bool batch_gyro;

	/* calibration */
	char acc_cal_buf[MPU_ACC_CAL_BUF_SIZE];
	int acc_cal_params[3];
	bool acc_use_cal;

	/* power control */
#if 0
	struct regulator *vlogic;
#endif
	struct regulator *vdd;
	struct regulator *vi2c;
	int enable_gpio;
	bool power_enabled;

	/* pinctrl */
	struct pinctrl *pinctrl;
	struct pinctrl_state *pin_default;
	struct pinctrl_state *pin_sleep;

	u32 flush_count;
	u64 fifo_start_ns;
	int gyro_wkp_flag;
	int accel_wkp_flag;
	struct task_struct *gyr_task;
	struct task_struct *accel_task;
	bool gyro_delay_change;
	bool accel_delay_change;
	wait_queue_head_t	gyro_wq;
	wait_queue_head_t	accel_wq;
};
void mpu6000::setup_mpu6000_reg(struct mpu_reg_map *reg)
{
	reg->sample_rate_div	= REG_SAMPLE_RATE_DIV;
	reg->lpf		= REG_CONFIG;
	reg->fifo_en		= REG_FIFO_EN;
	reg->gyro_config	= REG_GYRO_CONFIG;
	reg->accel_config	= REG_ACCEL_CONFIG;
	reg->mot_thr		= REG_ACCEL_MOT_THR;
	reg->mot_dur		= REG_ACCEL_MOT_DUR;
	reg->fifo_count_h	= REG_FIFO_COUNT_H;
	reg->fifo_r_w		= REG_FIFO_R_W;
	reg->raw_gyro		= REG_RAW_GYRO;
	reg->raw_accel		= REG_RAW_ACCEL;
	reg->temperature	= REG_TEMPERATURE;
	reg->int_pin_cfg	= REG_INT_PIN_CFG;
	reg->int_enable		= REG_INT_ENABLE;
	reg->int_status		= REG_INT_STATUS;
	reg->user_ctrl		= REG_USER_CTRL;
	reg->pwr_mgmt_1		= REG_PWR_MGMT_1;
	reg->pwr_mgmt_2		= REG_PWR_MGMT_2;
};

/**
 * mpu_check_chip_type() - check and setup chip type.
 */
int mpu6000::mpu_check_chip_type(void)
{
	s32 ret;
	struct mpu_reg_map *reg;

	if (!strcmp(sensor->name, "mpu6050"))
		sensor->chip_type = INV_MPU6050;
	else if (!strcmp(sensor->name, "mpu6500"))
		sensor->chip_type = INV_MPU6500;
	else if (!strcmp(sensor->name, "mpu6xxx"))
		sensor->chip_type = INV_MPU6050;
	else
		return -EPERM;

	reg = &sensor->reg;
	setup_mpu6050_reg(reg);

	/* turn off and turn on power to ensure gyro engine is on */
	ret = mpu6050_set_power_mode(sensor, false);
	if (ret)
		return ret;
	ret = mpu6050_set_power_mode(sensor, true);
	if (ret)
		return ret;

	if (!strcmp(id->name, "mpu6xxx")) {
		ret = i2c_smbus_read_byte_data(client,
				REG_WHOAMI);
		if (ret < 0)
			return ret;

		if (ret == MPU6500_ID) {
			sensor->chip_type = INV_MPU6500;
		} else if (ret == MPU6050_ID) {
			sensor->chip_type = INV_MPU6050;
		} else {
			dev_err(&client->dev,
				"Invalid chip ID %d\n", ret);
			return -ENODEV;
		}
	}
	return 0;
}

}
#endif

/***********************************************************************
** End of file
***********************************************************************/
