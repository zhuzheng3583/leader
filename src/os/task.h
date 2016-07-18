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

struct task_params
{
	PCSTR name;								// 任务名称
	s32	priority;						// 任务优先级
	u32 stackbase;						// 任务堆栈基地址(注意字节对齐，当前系统8字节对齐)
	u32 stacksize;						// 任务堆栈大小
	void *run;								// 任务功能
	void *parg;								// 任务功能参数
};

class task : public os_object
{
public:
	task(void);
	~task(void);

public:
	struct task_params _params;

public:
	BOOL create(struct task_params *pparams);
	BOOL t_delete(void);
	void sleep(s32 timeouts);
	void msleep(s32 timeoutms);
	void usleep(s32 timeoutus);

	u32 get_cpu_usage(void);

public:
	virtual void run(void *parg) = 0;	// 纯虚函数

protected:
    static BOOL func(task* ptask);

public:
	inline BOOL set_param(struct task_params *param);
	inline BOOL get_param(struct task_params *param) const;
};

}

/***********************************************************************
** End of file
***********************************************************************/

