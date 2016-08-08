/*******************************Copyright (c)***************************
**
** Porject name:	LeaderUAV-Plus
** Created by:		zhuzheng<happyzhull@163.com>
** Created date:	2015/08/28
** Modified by:
** Modified date:
** Descriptions:
**
***********************************************************************/
#pragma once
#include "leader_type.h"
#include "leader_misc.h"

#include "os/kernel.h"

namespace os {

class event : public os_object
{
public:
	event(void);
	~event(void);

public:
	BOOL create(PCSTR os_name);
	BOOL e_delete(void);
	BOOL pend(s32 timeoutms);
	BOOL post(s32 timeoutms);
};

}
/***********************************************************************
** End of file
***********************************************************************/
