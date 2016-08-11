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
#include "autopilot.h"

#include "leader_system.h"

namespace app {

autopilot::autopilot(void)
{
	_params.name = "receive";
	_params.priority = 0;
	_params.stacksize = 512;
	_params.func = (void *)thread::func;
	_params.parg = this;
}

autopilot::~autopilot(void)
{

}

void autopilot::run(void *parg)
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
		DBG("%s: temperature=%f, pressure=%f, altitude=%f.\n",
			_os_name, baro.temperature, baro.pressure, baro.altitude);

        niming->report_status(NULL);
        niming->report_sensor(&accel, &gyro, &mag);
        //niming->report_rc(&pitem_rc->data_rc[i]);
#if 0
        ms5611->read((u8 *)(pitem_baro->data_baro), pattr->num_baro * sizeof(data_baro_t));
		for (u32 i = 0; /*i < pattr->num_baro*/ i < 1; i++)
		{
            DBG("%s: temp=%d, pres=%d, alt=%f.\n",
                _os_name,
                pitem_baro->data_baro[i].temperature,
                pitem_baro->data_baro[i].pressure,
                pitem_baro->data_baro[i].altitude
                );
		}

		for (u32 i = 0; i < pattr->num_magn; i++)
		{
			pitem_magn->data_magn[i].x = 1;
			pitem_magn->data_magn[i].y = 2;
			pitem_magn->data_magn[i].z = 3;
		}

		for (u32 i = 0; i < pattr->num_attitude; i++)
		{
			pitem_atti->data_attitude[i].roll= 1;
			pitem_atti->data_attitude[i].pitch= 2;
			pitem_atti->data_attitude[i].yaw = 3;
			pitem_atti->data_attitude[i].altitude = 4;
		}

		for (u32 i = 0; i < pattr->num_rc; i++)
		{
			pitem_rc->data_rc[i].throttle= 1;
			pitem_rc->data_rc[i].roll= 2;
			pitem_rc->data_rc[i].pitch= 3;
			pitem_rc->data_rc[i].yaw = 4;
			pitem_rc->data_rc[i].aux1 = 1;
			pitem_rc->data_rc[i].aux2 = 2;
			pitem_rc->data_rc[i].aux3 = 3;
			pitem_rc->data_rc[i].aux4 = 4;
			pitem_rc->data_rc[i].aux5 = 5;
			pitem_rc->data_rc[i].aux5 = 6;
		}
#endif

        msleep(10);

		//DBG("%s: task is active[%u]...\n", _os_name, cnt++);

	}
}

}
/***********************************************************************
** End of file
***********************************************************************/


