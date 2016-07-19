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
#error pre-defined macros required.
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
	//demo_main();
    
	s32 ret = 0;
	ret = core::init();
	ret = interrupt::irq_init();
	_puart = new uart("uart-2", 2);
	_puart->probe();
	_puart->open(NULL);
    //_puart->self_test();
    
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

	_led = new gpio("gpio-79", 79);
	_led->probe();
	_led->set_direction_output();
	//_led->self_test();

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


	gpio *pgpio = new gpio("gpio-67", 67);
	pgpio->probe();
	pgpio->set_direction_output();
	pgpio->set_value(1);
	_spi = new spi("spi-1", 1);
	_spi->probe();


  	u8 dev_id = 0;
	u8 addr = 0;//L3GD20_WHO_AM_I_ADDR;
	if(sizeof(dev_id) > 0x01) {
		addr |= (u8)(READWRITE_CMD | MULTIPLEBYTE_CMD);
  	} else {
		addr |= (u8)READWRITE_CMD;
  	}
  
	pgpio->set_value(0);
	_spi->tx(&addr, sizeof(addr));
	_spi->rx(&dev_id, sizeof(dev_id));
	INF("dev_id = 0x%02x.\n", dev_id);
	pgpio->set_value(1);
	
    while(1);
   	//interrupt::irq_init();
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

