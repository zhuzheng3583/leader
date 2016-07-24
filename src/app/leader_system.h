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
#pragma once
#include "leader_type.h"
#include "leader_misc.h"

#include "device.h"
#include "core.h"
#include "flash.h"
#include "uart.h"
#include "i2c.h"
#include "usb_dev.h"
#include "sensorhub.h"
#include "gpio.h"
#include "timer.h"
#include "pwm.h"
#include "spi.h"

#include "mpu6000.h"

#include "demo_main.h"

#include "kernel.h"
#include "heartbeat.h"
#include "receive.h"
#include "calculate.h"
#include "transmit.h"
#include "terminal.h"

#include "msgque.h"


using namespace driver;
using namespace os;

namespace app {

class leader_system
{
public:
	leader_system(void);
	~leader_system(void);

public:
    /**
     *  @enum  leader_system_mode
     *  @brief 平台模式
     */
    enum leader_system_mode
    {
        M_AUTO = 0,
        M_BOOTLOADER,
        M_AIRCRAFT,
    };

public:
    enum leader_system_mode _mode;
    uart                    *_puart;
    uart                    *_puart1;
    uart                    *_puart2;
    uart                    *_puart3;

    flash                   *_pflash;

    i2c                     *_i2c1;
    i2c                     *_i2c2;

    usb_dev                 *_usb_dev;
    sensorhub               *_sensorhub;

    gpio                    *_led;
    gpio                    *_gpio_irq;

    timer                   *_timer;
    pwm                     *_pwm;
    spi                     *_spi;

	mpu6000					*_mpu6000;

	msgque					*_sync_r_c;
	msgque					*_sync_c_t;


	heartbeat               *_heartbeat;
	receive					*_receive;
	calculate				*_calculate;
	transmit				*_transmit;
	terminal				*_terminal;

public:
    enum leader_system_mode    get_mode(void)		{ return _mode; }
    uart        *get_uart(void)		    	{ return _puart; }
    uart        *get_uart1(void)		    { return _puart1; }
    uart        *get_uart2(void)		    { return _puart2; }
    uart        *get_uart3(void)		    { return _puart3; }

    flash       *get_flash(void)            { return _pflash; }

    i2c         *get_i2c1(void)             { return _i2c1; }
    i2c         *get_i2c2(void)             { return _i2c2; }

    usb_dev     *get_usb_dev(void)          { return _usb_dev; }
    sensorhub   *get_sensorhub(void)        { return _sensorhub; }

	mpu6000   	*get_mpu6000(void)        	{ return _mpu6000; }

	msgque   	*get_sync_r_c(void)			{ return _sync_r_c; }
	msgque   	*get_sync_c_t(void)			{ return _sync_c_t; }

protected:
    static leader_system *s_pactive_instance;

public:
    s32 init(enum leader_system_mode mode);
    virtual s32 init(void);
    virtual void start(void);
    virtual s32 exit(void);

    // 获取唯一全局实例
    inline static leader_system* get_instance(void) { return s_pactive_instance; }
};

}

/***********************************************************************
** End of file
***********************************************************************/


