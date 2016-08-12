/*******************************Copyright (c)***************************
**
** Porject name:	leader
** Created by:	zhuzheng<happyzhull@163.com>
** Created date:	2016/04/08
** Modified by:
** Modified date:
** Descriptions:
**
***********************************************************************/
#include "aircraft.h"

namespace app {

aircraft::aircraft(void) :
	_into_calibrate(false)
{

}

aircraft::~aircraft(void)
{

}

s32 aircraft::init(void)
{
	leader_system::init();
	INF("========Init LeaderUAV Aircraft App ========\n");

	kernel::init();

    _puart3 = new uart("uart-3", 3);
	_puart3->probe();
	//_puart3->self_test();
	_niming = new niming;
	_niming->attach(_puart3);

	_puart1 = new uart("uart-1", 1);
	_puart1->probe();
	_puart1->open(NULL);
	// _puart1->self_test();
	gps *pgps = new gps("gps", -1);
	//pgps->probe(_puart1);

	_spi1 = new spi("spi-1", 1);
	_spi1->probe();

	_mpu6000_gpio_cs = new gpio("mpu6000_gpio_cs-34", 34);
	_mpu6000_gpio_cs->probe();
	_mpu6000 = new mpu6000("mpu6000", -1);
	_mpu6000->probe(_spi1, _mpu6000_gpio_cs);

	_ms5611_gpio_cs = new gpio("ms5611_gpio_cs-55", 55);
	_ms5611_gpio_cs->probe();
    _ms5611 = new ms5611("ms5611", -1);
    _ms5611->probe(_spi1, _ms5611_gpio_cs);

	_i2c2 = new i2c("i2c-2", 2);
    _i2c2->probe();
    _hmc5883 = new hmc5883("hmc5883", -1);
    _hmc5883->probe(_i2c2, HMC5883_SLAVE_ADDRESS);


	_heartbeat = new heartbeat;
	_terminal = new terminal;
	_autopilot = new autopilot;
	_calibration = new calibration;

	_logger->create(NULL);
	_mpu6000->create(NULL);
    _hmc5883->create(NULL);
    _ms5611->create(NULL);
	_heartbeat->create(NULL);
	_terminal->create(NULL);

    _into_calibrate = false;
	if (_into_calibrate == true) {
		_calibration->create(NULL);
	} else {
		_autopilot->create(NULL);
	}

	return 0;
}

void aircraft::start(void)
{
	INF("========Start LeaderUAV Aircraft App ========\n");
	kernel::start();
}

s32 aircraft::exit(void)
{

	return 0;
}

}


/***********************************************************************
** End of file
***********************************************************************/

