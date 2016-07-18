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
#include "flash.h"
#include "i2c.h"
#include "usb_dev.h"
#include "sensorhub.h"

#include "leader_system.h"

typedef  void (*iap_func)(void);				//定义一个函数类型的参数.   
//保留0X08000000~0X0800FFFF的空间为Bootloader使用(共64KB)	  
#define FLASH_APP_ADDR		FLASH_USER_START_ADDR//0x08010000  	//第一个应用程序起始地址(存放在FLASH)


using namespace driver;

namespace app {

class bootloader : public leader_system
{
public:
	bootloader(void);
	~bootloader(void);

public:
    iap_func _jump2app; 
    flash*   _pflash;
    i2c*     _i2c;
    usb_dev* _usb_dev;
    sensorhub* _sensorhub;
    
public:
	s32 init(void);
	void start(void);

    s32 iap_download_firmware(u32 start_addr, u8 *buf, u32 len);	    //在指定地址开始,写入bin
    void iap_upload_firmware(u32 start_addr);			            //跳转到APP程序执行
};

}

/***********************************************************************
** End of file
***********************************************************************/


