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
	_params.priority = 0;
	_params.stacksize = 128;
	_params.func = (void *)thread::func;
	_params.parg = this;
}

heartbeat::~heartbeat(void)
{

}

void heartbeat::run(void *parg)
{
  	heartbeat *p = (heartbeat *)parg;
	for (u32 cnt = 0; ;cnt++)
	{
		INF("%s: LEADER_UAV_HEART_BEAT[%u sec]...\n", _name, cnt);
		// ��ȡCPUʹ����
		msleep(1000);
	}
}

}
/***********************************************************************
** End of file
***********************************************************************/


