/*******************************Copyright (c)***************************
**
** Porject name:	leader
** Created by:	zhuzheng<happyzhull@163.com>
** Created date:	2016/07/23
** Modified by:
** Modified date:
** Descriptions:
**
***********************************************************************/

#include "terminal.h"

#include "leader_system.h"

namespace app {

terminal::terminal(void)
{
	_params.name = "terminal";
	_params.priority = 0;
	_params.stacksize = configMINIMAL_STACK_SIZE;//;
	_params.func = (void *)thread::func;
	_params.parg = this;
}

terminal::~terminal(void)
{

}

void terminal::run(void *parg)
{
#if 0
    uint32_t cnt = 0;
	for (cnt = 0; ;cnt++)
	{
		/* TODO:÷’∂ÀΩªª• */
		INF("%s: task is active[%u]...\n", _params.name, cnt);
		msleep(200);
	}
#endif
}

}
/***********************************************************************
** End of file
***********************************************************************/

