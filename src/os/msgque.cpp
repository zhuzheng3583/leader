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
#include "os/msgque.h"

#include "includes.h"

namespace os {

msgque::msgque(void) :
    _msgcnt(0)
{

}

msgque::~msgque(void)
{

}

BOOL msgque::create(PCSTR name, u32 msgcnt)
{
	s32 error = OS_ERR_NONE;

    _name = name;
    _msgcnt = msgcnt;
	_handle = (HANDLE)malloc(sizeof(OS_Q));
    if (_handle == NULL) {
        goto fail0;
    }

	CPU_SR_ALLOC();
	OS_CRITICAL_ENTER();	//进入临界区
	//函数原型: void OSQCreate(OS_Q *p_q, CPU_CHAR *p_name, OS_MSG_QTY max_qty, OS_ERR *p_err)
	OSQCreate(
		(OS_Q *)(_handle),
		(CPU_CHAR *)_name,
		(OS_MSG_QTY)_msgcnt,
		(OS_ERR *)&error
		);
	OS_CRITICAL_EXIT();		// 退出临界区
	if (error != OS_ERR_NONE) {
        DBG("%s error code = %d.\n", __FUNCTION__, error);
        kernel::on_error(ERR_OPERATION_FAILED, this);
		goto fail1;
	}

	return TRUE;

fail1:
	free((void *)_handle);
	_handle = NULL;
fail0:
	return FALSE;
}

BOOL msgque::q_delete(void)
{
	s32 error = OS_ERR_NONE;
	u32 count = 0;
	//函数原型: OS_OBJ_QTY OSQDel(OS_Q *p_q, OS_OPT opt, OS_ERR *p_err)
	//opt: OS_OPT_DEL_NO_PEND
	//	   OS_OPT_DEL_ALWAYS
	count = OSQDel(
		(OS_Q *)(_handle),
		(OS_OPT)OS_OPT_DEL_NO_PEND,
		(OS_ERR *)&error
		);
	if (error != OS_ERR_NONE) {
        DBG("%s error code = %d.\n", __FUNCTION__, error);
        kernel::on_error(ERR_OPERATION_FAILED, this);
		return FALSE;
	}
    if (count > 0) {
        WRN("%d tasks waiting on the queue are now readied and informed.\n", count);
        return FALSE;
    }

	free((void *)_handle);
	_handle = NULL;

	return TRUE;
}

BOOL msgque::pend(void *pmessage, u32 *psize, s32 timeoutms)
{
	s32 error = OS_ERR_NONE;
	u32 tick = kernel::convertmstotick(timeoutms);

	//函数原型: void *OSQPend (OS_Q *p_q, OS_TICK timeout, OS_OPT opt, OS_MSG_SIZE *p_msg_size, CPU_TS *p_ts, OS_ERR *p_err)
	//opt: OS_OPT_PEND_BLOCKING
	//	   OS_OPT_PEND_NON_BLOCKING
	*((u32 *)pmessage) = (u32)OSQPend(
		(OS_Q *)(_handle),
		(OS_TICK)tick,
		(OS_OPT)OS_OPT_PEND_BLOCKING,	// 阻塞直到超时
		(OS_MSG_SIZE *)psize,
		(CPU_TS)0,						// 时间戳相关
		(OS_ERR *)&error
		);
	if (error != OS_ERR_NONE) {
        DBG("%s error code = %d.\n", __FUNCTION__, error);
        kernel::on_error(ERR_OPERATION_FAILED, this);
		return FALSE;
	}

	return TRUE;
}

BOOL msgque::post(void *pmessage, u32 size, s32 timeoutms)
{
	s32 error = OS_ERR_NONE;
	s32 dlyerror = OS_ERR_NONE;
	//函数原型: void OSQPost(OS_Q *p_q, void *p_void, OS_MSG_SIZE msg_size, OS_OPT opt, OS_ERR *p_err)
	//opt: OS_OPT_POST_ALL
	//	   OS_OPT_POST_FIFO
	//	   OS_OPT_POST_LIFO
	//	   OS_OPT_POST_NO_SCHED
loopOSQPost:
	OSQPost(
		(OS_Q *)(_handle),
		pmessage,
		(OS_MSG_SIZE)size,
		(OS_OPT)OS_OPT_POST_FIFO,
		(OS_ERR *)&error
		);
	if (error == OS_ERR_Q_MAX) {
		if (timeoutms == 0) { goto out; }
		if (timeoutms < 0)	{ timeoutms++; }
		timeoutms--;
		OSTimeDly(kernel::convertmstotick(1)/*1*/, OS_OPT_TIME_PERIODIC, (OS_ERR *)&dlyerror);
		if (dlyerror != OS_ERR_NONE) {
            DBG("%s error code = %d.\n", __FUNCTION__, error);
            kernel::on_error(ERR_OPERATION_FAILED, this);
		    return FALSE;
		}

		goto loopOSQPost;
	}

out:
	if (error != OS_ERR_NONE) {
        DBG("%s error code = %d.\n", __FUNCTION__, error);
        kernel::on_error(ERR_OPERATION_FAILED, this);

		return FALSE;
	}

	return TRUE;
}

void msgque::reset(void)
{

}

}
/***********************************************************************
** End of file
***********************************************************************/