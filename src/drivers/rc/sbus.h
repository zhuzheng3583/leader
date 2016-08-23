/*******************************Copyright (c)***************************
**
** Porject name:	leader
** Created by:	zhuzheng<happyzhull@163.com>
** Created date:	2016/08/22
** Modified by:
** Modified date:
** Descriptions:
**
***********************************************************************/
#pragma once
#include "device.h"
#include "gpio.h"

#define BAUD9600_DELAY_US       ((1*1000*1000) / 9600)
#define BAUD100000_DELAY_US     ((1*1000*1000) / 100000)
#define BAUD115200_DELAY_US     ((1*1000*1000) / 115200)

#define DEFAULT_BAUD_DELAY_US   (BAUD9600_DELAY_US)      

namespace driver {

class sbus : public device
{
public:
    sbus(PCSTR name, s32 id);
    ~sbus(void);

public:
	gpio *_rx;
	gpio *_tx;

public:
    s32 probe(void);
    s32 remove(void);

public:
    s32 init(void);
    s32 reset(void);
    void measure(void);
	void write_byte(s8 c);
    s8 read_byte(void);
};

}
/***********************************************************************
** End of file
***********************************************************************/
