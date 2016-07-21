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
#include "os/mutex.h"

#include "includes.h"

namespace os {

mutex::mutex(void)
{

}

mutex::~mutex(void)
{

}

BOOL mutex::create(PCSTR name)
{
	s32 error = OS_ERR_NONE;
    _name = name;
	_handle = (HANDLE)malloc(sizeof(OS_MUTEX));
    if (_handle == NULL) {
        goto fail0;
    }

	CPU_SR_ALLOC();
	OS_CRITICAL_ENTER();	//进入临界区
	//函数原型: void OSMutexCreate(OS_MUTEX *p_mutex, CPU_CHAR *p_name, OS_ERR *p_err)
	OSMutexCreate(
		(OS_MUTEX *)(_handle),
		(CPU_CHAR *)(_name),
		(OS_ERR *)(&error)
		);
	OS_CRITICAL_EXIT();		// 退出临界区
	if (error != OS_ERR_NONE) {
        DBG("%s error code = %d.\n", __FUNCTION__, error);
        kernel::on_error(ERR_OPERATION_FAILED, this);
		goto fail1;
	}

	return true;

fail1:
	free((void *)_handle);
	_handle = NULL;
fail0:
	return false;
}

BOOL mutex::m_delete(void)
{
    s32 error = OS_ERR_NONE;
	u32 count = 0;
	//函数原型: OS_OBJ_QTY OSMutexDel(OS_MUTEX *p_mutex, OS_OPT opt, OS_ERR *p_err)
	//opt: OS_OPT_DEL_NO_PEND
	//	   OS_OPT_DEL_ALWAYS
	count = OSMutexDel(
		(OS_MUTEX *)(_handle),
		(OS_OPT)OS_OPT_DEL_NO_PEND,
		(OS_ERR *)&error
		);
	if (error != OS_ERR_NONE) {
        DBG("%s error code = %d.\n", __FUNCTION__, error);
        kernel::on_error(ERR_OPERATION_FAILED, this);
		return false;
	}
    if (count > 0) {
        WRN("%d tasks waiting on the mutex are now readied and informed.\n", count);
        return false;
    }

	free((void *)_handle);
	_handle = NULL;

	return true;
}

BOOL mutex::pend(s32 timeoutms)
{
	s32 error = OS_ERR_NONE;
	u32 tick = kernel::convertmstotick(timeoutms);

	//函数原型: void OSMutexPend(OS_MUTEX  *p_mutex, OS_TICK timeout, OS_OPT opt, CPU_TS *p_ts,OS_ERR *p_err)
	//opt: OS_OPT_PEND_BLOCKING
	//	   OS_OPT_PEND_NON_BLOCKING
	OSMutexPend(
		(OS_MUTEX *)(_handle),
		(OS_TICK)tick,
		(OS_OPT)OS_OPT_PEND_BLOCKING,	// 阻塞直到超时
		(CPU_TS)0,						// 时间戳相关
		(OS_ERR *)&error
		);
	if (error != OS_ERR_NONE) {
        DBG("%s error code = %d.\n", __FUNCTION__, error);
        kernel::on_error(ERR_OPERATION_FAILED, this);
		return false;
	}

	return true;
}

BOOL mutex::post(s32 timeoutms)
{
	s32 error = OS_ERR_NONE;
	s32 dlyerror = OS_ERR_NONE;

	//函数原型: void OSMutexPost(OS_MUTEX *p_mutex, OS_OPT opt, OS_ERR *p_err)
	//opt: OS_OPT_POST_NONE
	//	   OS_OPT_POST_NO_SCHED
loopOSMutexPost:
	OSMutexPost(
		(OS_MUTEX *)(_handle),
		(OS_OPT)OS_OPT_POST_NONE,
		(OS_ERR *)&error
		);
#if 0
	if (error == /* ? */) {
		if (timeoutms == 0) { goto out; }
		if (timeoutms < 0)	{ timeoutms++; }
		timeoutms--;
		OSTimeDly(kernel::convertmstotick(1), OS_OPT_TIME_PERIODIC, &dlyerror);
		if (dlyerror != OS_ERR_NONE)
		{
            DBG("%s error code = %d.\n", __FUNCTION__, error);
            kernel::on_error(ERR_OPERATION_FAILED, this);
		    return false;
		}
		goto loopOSMutexPost;
	}
#endif
out:
	if (error != OS_ERR_NONE) {
        DBG("%s error code = %d.\n", __FUNCTION__, error);
        kernel::on_error(ERR_OPERATION_FAILED, this);
		return false;
	}

	return true;
}

}
/***********************************************************************
** End of file
***********************************************************************/