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
#include "os/semaphore.h"

#include "includes.h"

namespace os {

semaphore::semaphore(void) :
    _semcnt(0)
{

}

semaphore::~semaphore(void)
{

}

BOOL semaphore::create(PCSTR name, u32 semcnt)
{
	s32 error = OS_ERR_NONE;
    _name = name;
    _semcnt = semcnt;
	_handle= (HANDLE)malloc(sizeof(OS_SEM));
    if (_handle == NULL) {
        goto fail0;
    }

	CPU_SR_ALLOC();
	OS_CRITICAL_ENTER();	//进入临界区
	//函数原型: void OSSemCreate(OS_SEM *p_sem, CPU_CHAR *p_name, OS_SEM_CTR cnt, OS_ERR *p_err)
	OSSemCreate(
		(OS_SEM *)(_handle),
		(CPU_CHAR *)_name,
		(OS_SEM_CTR)_semcnt,
		(OS_ERR *)&error
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

BOOL semaphore::s_delete(void)
{
	s32 error = OS_ERR_NONE;
	u32 count = 0;
	//函数原型: OS_OBJ_QTY OSSemDel(OS_SEM *p_sem, OS_OPT opt, OS_ERR *p_err)
	//opt: OS_OPT_DEL_NO_PEND
	//	   OS_OPT_DEL_ALWAYS
	count = OSSemDel(
		(OS_SEM *)(_handle),
		(OS_OPT)OS_OPT_DEL_NO_PEND,
		(OS_ERR *)&error
		);
	if (error != OS_ERR_NONE) {
        DBG("%s error code = %d.\n", __FUNCTION__, error);
        kernel::on_error(ERR_OPERATION_FAILED, this);

		return false;
	}
    if (count > 0) {
        WRN("%d tasks waiting on the semaphore are now readied and informed.\n", count);
        return false;
    }

	free((void *)_handle);
	_handle = NULL;

	return true;
}

BOOL semaphore::pend(s32 timeoutms)
{
	s32 error = OS_ERR_NONE;
	u32 count = 0;
	u32 tick = kernel::convertmstotick(timeoutms);

	//函数原型: OS_SEM_CTR OSSemPend(OS_SEM *p_sem, OS_TICK timeout, OS_OPT opt, CPU_TS *p_ts, OS_ERR *p_err)
	//opt: OS_OPT_PEND_BLOCKING
	//	   OS_OPT_PEND_NON_BLOCKING
	count = OSSemPend(
		(OS_SEM *)(_handle),
		(OS_TICK)tick,
		(OS_OPT)OS_OPT_PEND_BLOCKING,	// 阻塞直到超时
		(CPU_TS)0,						// 时间戳相关
		(OS_ERR *)&error
		);
	if (error != OS_ERR_NONE)
	{
        DBG("%s error code = %d.\n", __FUNCTION__, error);
        kernel::on_error(ERR_OPERATION_FAILED, this);
		return false;
	}

    //if (count ) {
    INF("func_name: %s, count = %d.\n", __FUNCTION__, count);
    //}

	return true;
}

BOOL semaphore::post(s32 timeoutms)
{
	s32 error = OS_ERR_NONE;
	s32 dlyerror = OS_ERR_NONE;
	u32 count = 0;

	//函数原型: OS_SEM_CTR OSSemPost (OS_SEM  *p_sem, OS_OPT opt, OS_ERR *p_err)r)
	//opt: OS_OPT_POST_1
	//	   OS_OPT_POST_ALL
	//	   OS_OPT_POST_NO_SCHED
loopOSSemPost:
	count = OSSemPost(
		(OS_SEM *)(_handle),
		(OS_OPT)OS_OPT_POST_1,
		(OS_ERR *)&error
		);
	if (error == OS_ERR_SEM_OVF)
	{
		if (timeoutms == 0) { goto out; }
		if (timeoutms < 0)	{ timeoutms++; }
		timeoutms--;
		OSTimeDly(kernel::convertmstotick(1), OS_OPT_TIME_PERIODIC, (OS_ERR *)&dlyerror);
		if (dlyerror != OS_ERR_NONE)
		{
            DBG("%s error code = %d.\n", __FUNCTION__, error);
            kernel::on_error(ERR_OPERATION_FAILED, this);

			return false;
		}
		goto loopOSSemPost;
	}

out:
	if (error != OS_ERR_NONE) {
        DBG("%s error code = %d.\n", __FUNCTION__, error);
        kernel::on_error(ERR_OPERATION_FAILED, this);
		return false;
	}
    //if (count ) {
    INF("func_name: %s, count = %d.\n", __FUNCTION__, count);
    //}

	return true;
}

void semaphore::reset(void)
{

}

}
/***********************************************************************
** End of file
***********************************************************************/
