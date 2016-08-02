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
#include "ms5611.h"
#include "hmc5883.h"
#include "gps.h"

#include "demo_main.h"

#include "kernel.h"
#include "heartbeat.h"
#include "receive.h"
#include "calculate.h"
#include "transmit.h"
#include "terminal.h"
#include "msgque.h"

#include "packet.h"

#include "niming.h"

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
    uart                    *_puart1;
    uart                    *_puart2;
    uart                    *_puart3;

    i2c                     *_i2c1;
    i2c                     *_i2c2;

    spi                     *_spi1;

    gpio                    *_mpu6000_gpio_cs;
    gpio                    *_ms5611_gpio_cs;
    mpu6000					*_mpu6000;
    ms5611					*_ms5611;
	hmc5883					*_hmc5883;

    flash                   *_pflash;

    usb_dev                 *_usb_dev;
    sensorhub               *_sensorhub;

    gpio                    *_led;
    gpio                    *_gpio_irq;

    timer                   *_timer;
    pwm                     *_pwm;


    msgque					*_sync_rc;
    msgque					*_sync_ct;
    msgque					*_sync;

    heartbeat               *_heartbeat;
    receive					*_receive;
    calculate				*_calculate;
    transmit				*_transmit;
    terminal				*_terminal;

    packet                  *_packet;

    niming                  *_niming;

public:
    enum leader_system_mode    get_mode(void)		{ return _mode; }
    uart        *get_uart1(void)		    { return _puart1; }
    uart        *get_uart2(void)		    { return _puart2; }
    uart        *get_uart3(void)		    { return _puart3; }

    flash       *get_flash(void)            { return _pflash; }

    i2c         *get_i2c1(void)             { return _i2c1; }
    i2c         *get_i2c2(void)             { return _i2c2; }

    usb_dev     *get_usb_dev(void)          { return _usb_dev; }
    sensorhub   *get_sensorhub(void)        { return _sensorhub; }

    mpu6000   	*get_mpu6000(void)        	{ return _mpu6000; }
	ms5611		*get_ms5611(void)        	{ return _ms5611; }
	hmc5883		*get_hmc5883(void)        	{ return _hmc5883; }

    msgque   	*get_sync_rc(void)			{ return _sync_rc; }
    msgque   	*get_sync_ct(void)			{ return _sync_ct; }
    msgque   	*get_sync(void)			    { return _sync; }

    packet      *get_packet(void)           { return _packet; }

    niming      *get_niming(void)           { return _niming; }
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


