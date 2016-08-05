/*******************************Copyright (c)***************************
**
** Porject name:	leader
** Created by:		zhuzheng<happyzhull@163.com>
** Created date:	2016/04/05
** Modified by:
** Modified date:
** Descriptions:
**
***********************************************************************/
#include "leader_system.h"

#include "bootloader.h"
#include "aircraft.h"

#include "demo_main.h"

#include "cmsis_os.h"


namespace app {

leader_system *leader_system::s_pactive_instance = NULL;

leader_system::leader_system(void)
{

}

leader_system::~leader_system(void)
{
	delete s_pactive_instance;
	s_pactive_instance = NULL;
}

s32 leader_system::init(enum leader_system_mode mode)
{
	// TODO: 调试时在此强制指定系统模式
	//mode = M_BOOTLOADER;

	_mode = mode;
	switch(mode)
	{
	case M_AUTO:
#if defined(BOOTLOADER)
        s_pactive_instance = new bootloader;
		_mode = M_BOOTLOADER;
#elif defined(AIRCRAFT)
		s_pactive_instance = new aircraft;
		_mode = M_AIRCRAFT;
#else
#error Pre-defined macros required.
#endif
        break;
	case M_BOOTLOADER:
		s_pactive_instance = new bootloader;
		break;
	case M_AIRCRAFT:
		s_pactive_instance = new aircraft;
		break;
	default:
		break;
	}

	if (s_pactive_instance) {
		return s_pactive_instance->init();
	}

	return -1;
}

void timer_func(void *arg)
{
	INF("timer_func: success call!\n");
}


s32 leader_system::init(void)
{
#if USE_STM32F4_DEMO
	demo_main();
#endif

	s32 ret = 0;
	ret = core::init();
	ret = interrupt::irq_init();
	kernel::init();


	_logger = new logger;

	_puart2 = new uart("uart-2", 2);
	_puart2->probe();
	//_puart2->self_test();

	_logger->attach(_puart2);
	//_logger->self_test();
	//在此之前不能使用log输出

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


#if 0

	_pflash = new flash("flash", -1);
	_pflash->probe();

	_i2c1 = new i2c("i2c-1", 1);
	_i2c1->probe();
	_i2c1->self_test();

	_usb_dev = new usb_dev("usb_dev", -1);
	_usb_dev->probe();
	//_usbd->self_test();

	_sensorhub = new sensorhub("sensorhub", -1);
	_sensorhub->probe(_usb_dev);
	//_sensorhub->self_test();

	_gpio_irq = new gpio("gpio-0", 0);
	_gpio_irq->probe();
	_gpio_irq->set_gpio_to_irq();
	//while(1);

	_timer = new timer("timer-2", 2);
	_timer->probe();
	//_timer->self_test();
	_timer->set_timeout(1000);
	_timer->set_function(timer_func, (void *)1111);
//	_timer->start();
	//while(1);

	_pwm = new pwm("timer-3", 3);
	_pwm->probe();
	_pwm->set_dutycycle(PWM_CHANNEL_1, 50);
	_pwm->start(PWM_CHANNEL_1);
	_pwm->set_dutycycle(PWM_CHANNEL_2, 25);
	_pwm->start(PWM_CHANNEL_2);
	_pwm->set_dutycycle(PWM_CHANNEL_3, 10);
	_pwm->start(PWM_CHANNEL_3);
	_pwm->set_dutycycle(PWM_CHANNEL_4, 0);
	_pwm->start(PWM_CHANNEL_4);


    while(1);
#endif

    _packet = new packet;
    _packet->create(NULL);

    _sync_rc = new msgque;
    _sync_ct = new msgque;
    _sync = new msgque;
    _sync_rc->create("sync_rc", 1);
    _sync_ct->create("sync_ct", 1);
    _sync->create("sync", 1);

    _heartbeat = new heartbeat;
    _terminal = new terminal;
    _receive = new receive;
    _calculate = new calculate;
    _transmit = new transmit;

	_logger->create(NULL);
	_heartbeat->create(NULL);
	_terminal->create(NULL);
	_receive->create(NULL);
	_calculate->create(NULL);
	_transmit->create(NULL);

	if(ret < 0) {
		INF("Failed to leader_system::init");
		CAPTURE_ERR();
	}

	return 0;
}

void leader_system::start(void)
{
	//kernel::start();
}

s32 leader_system::exit(void)
{
	//kernel::exit();
	return 0;
}

}


/***********************************************************************
** End of file
***********************************************************************/

