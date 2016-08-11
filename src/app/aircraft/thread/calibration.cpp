/*******************************Copyright (c)***************************
**
** Porject name:	leader
** Created by:	zhuzheng<happyzhull@163.com>
** Created date:	2016/07/23
** Modified by:
** Modified date:
** Descriptions:
**
***********************************************************************/
#include "calibration.h"

#include "leader_system.h"

namespace app {

calibration::calibration(void)
{
	_params.name = "calibration_thread";
	_params.priority = 0;
	_params.stacksize = 512;
	_params.func = (void *)thread::func;
	_params.parg = this;
}

calibration::~calibration(void)
{

}

void calibration::run(void *parg)
{
    niming *niming = leader_system::get_instance()->get_niming();

	mpu6000 *mpu6000 = leader_system::get_instance()->get_mpu6000();
	mpu6000->open(NULL);
	ms5611 *ms5611 = leader_system::get_instance()->get_ms5611();
	ms5611->open(NULL);
	hmc5883 *hmc5883 = leader_system::get_instance()->get_hmc5883();
	hmc5883->open(NULL);

	struct accel_report accel;
	memset(&accel, 0, sizeof(accel));
	struct gyro_report gyro;
	memset(&gyro, 0, sizeof(gyro));
    struct mag_report mag;
	memset(&mag, 0, sizeof(mag));
    struct baro_report baro;
	memset(&baro, 0, sizeof(baro));

	for ( ; ;)
	{
		mpu6000->read_accel((u8 *)&accel, sizeof(accel_report));
		mpu6000->read_gyro((u8 *)&gyro, sizeof(gyro_report));
		//DBG("%s: accel.x_raw=%d, accel.y_raw=%d, accel.z_raw=%d, "
		//	"gyro.x_raw=%d, gyro.y_raw=%d, gyro.z_raw=%d.\n",
		//	_os_name, accel.x_raw, accel.y_raw, accel.z_raw,
		//	gyro.x_raw, gyro.y_raw, gyro.z_raw);

        hmc5883->read((u8 *)&mag, sizeof(mag_report));
		//DBG("%s: mag.x_raw=%d, mag.y_raw=%d, mag.z_raw=%d.\n",
		//	_os_name, mag.x_raw, mag.y_raw, mag.z_raw);

        ms5611->read((u8 *)&baro, sizeof(baro_report));
		//DBG("%s: temperature=%f, pressure=%f, altitude=%f.\n",
		//	_os_name, baro.temperature, baro.pressure, baro.altitude);

        niming->report_status(NULL);
        niming->report_sensor(&accel, &gyro, &mag);
        //niming->report_rc(&pitem_rc->data_rc[i]);

        msleep(2);
	}
}

}
/***********************************************************************
** End of file
***********************************************************************/


