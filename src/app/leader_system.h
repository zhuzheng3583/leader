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

#include "demo_main.h"

using namespace driver;

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
        M_GLASSES_SYSTEM,
    };
    
public:
    enum leader_system_mode _mode;
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
    
protected:
    static leader_system *s_pactive_instance;

public:
    virtual s32 init(void);
    s32 init(enum leader_system_mode mode);
    void start(void);
    s32 exit(void);

    // 获取唯一全局实例
    inline static leader_system* get_instance(void) { return s_pactive_instance; }
};

}

/***********************************************************************
** End of file
***********************************************************************/


