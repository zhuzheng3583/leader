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
#include "uart.h"

namespace driver {

class sbus : public device
{
public:
    sbus(PCSTR name, s32 id);
    ~sbus(void);

public:
	gpio *_rx;
	gpio *_tx;
    uart *_puart;
    
public:
    s32 probe(uart *puart);
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
