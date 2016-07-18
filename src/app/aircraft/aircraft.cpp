/*******************************Copyright (c)***************************
** 
** Porject name:	leader
** Created by:	zhuzheng<happyzhull@163.com>
** Created date:	2016/04/08
** Modified by:
** Modified date:
** Descriptions:
**
***********************************************************************/
#include "aircraft.h"

namespace app {

aircraft::aircraft(void)
{

}

aircraft::~aircraft(void)
{

}

s32 aircraft::init(void)
{
	leader_system::init();
	INF("========Init Skyview_H Aircraft App ========\n");
	
	return 0;
}

void aircraft::start(void)
{

}

s32 aircraft::exit(void)
{

	return 0;
}

}


/***********************************************************************
** End of file
***********************************************************************/

