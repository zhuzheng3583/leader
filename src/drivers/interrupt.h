/*******************************Copyright (c)***************************
** 
** Porject name:	leader
** Created by:	zhuzheng<happyzhull@163.com>
** Created date:	2016/07/07
** Modified by:
** Modified date:
** Descriptions:
**
***********************************************************************/
#pragma once
#include "leader_type.h"
#include "leader_misc.h"

//id = [0,81]
#define STM32F373XC_USER_IRQNUM_MAX	    82

namespace driver {

typedef  void (*vector_addr_t)(void);

class irq_handler
{
    friend class interrupt;

public:
    virtual void isr(void) = 0; 
};

struct map_table
{
public:
	s32 irq;
	irq_handler *handler;
};

class interrupt {

public:
	interrupt(void);
	~interrupt(void);

public:
	static struct map_table s_map[STM32F373XC_USER_IRQNUM_MAX];

public:
	static s32 irq_init(void);

	static s32 request_irq(s32 irq, irq_handler *handler);
	static void free_irq(s32 irq);

	static void enable_irq(s32 irq);
	static void disable_irq(s32 irq);

	static void enable_all_irq(void);
	static void disable_all_irq(void);
};




}
/***********************************************************************
** End of file
***********************************************************************/

