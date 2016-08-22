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
#include "sbus.h"

namespace driver {

sbus::sbus(PCSTR devname, s32 devid) :
	device(devname, devid)
{
	
}

sbus::~sbus(void)
{

}

s32 sbus::probe(void)
{

	return 0;
}


s32 sbus::remove(void)
{
	return 0;
}


s32 sbus::init(void)
{

}

s32 sbus::reset(void)
{

}

void sbus::measure(void)
{

}

}

/***********************************************************************
** End of file
***********************************************************************/
