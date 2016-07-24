/*******************************Copyright (c)***************************
**
** Porject name:	leader
** Created by:	zhuzheng<happyzhull@163.com>
** Created date:	2016/07/20
** Modified by:
** Modified date:
** Descriptions:
**
***********************************************************************/
#include "heartbeat.h"

#include "leader_system.h"

namespace app {

heartbeat::heartbeat(void)
{
	_params.name = "heartbeat";
	_params.priority = 12;
	_params.stackbase = NULL;
	_params.stacksize = 512;
	_params.func = (void *)thread::func;
	_params.parg = this;
}

heartbeat::~heartbeat(void)
{

}

void heartbeat::run(void *parg)
{
    u32 cnt = 0;
	for (cnt = 0; ; cnt++)
	{
		INF("%s: LEADER_UAV_HEART_BEAT[%u sec]...\n", _name, cnt);
		// 获取CPU使用率
		msleep(1000);
	}
}

}
/***********************************************************************
** End of file
***********************************************************************/



