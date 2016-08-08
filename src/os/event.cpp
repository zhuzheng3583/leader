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
#include "os/event.h"

#include "includes.h"

namespace os {

event::event(void)
{

}


event::~event(void)
{

}

BOOL event::create(PCSTR os_name)
{
	s32 error = OS_ERR_NONE;;

    _os_name = os_name;
	_os_handle = (HANDLE)malloc(sizeof(OS_FLAG_GRP));
    if (_os_handle == NULL) {
        goto fail0;
    }

	CPU_SR_ALLOC();
	OS_CRITICAL_ENTER();	//进入临界区
	//函数原型: void OSFlagCreate(OS_FLAG_GRP  *p_grp, CPU_CHAR *p_name, OS_FLAGS flags OS_ERR *p_err)
	OSFlagCreate(
		(OS_FLAG_GRP *)_os_handle,
		(CPU_CHAR *)_os_name,
		(OS_FLAGS)0x00000000,
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
	free((void *)_os_handle);
	_os_handle = NULL;
fail0:
	return false;
}

BOOL event::e_delete(void)
{
	s32 error = OS_ERR_NONE;;
	u32 count = 0;
	//函数原型: OS_OBJ_QTY OSFlagDel(OS_FLAG_GRP *p_grp, OS_OPT opt, OS_ERR *p_err)
	//opt: OS_OPT_DEL_NO_PEND
	//	   OS_OPT_DEL_ALWAYS
	count = OSFlagDel(
		(OS_FLAG_GRP *)_os_handle,
		(OS_OPT)OS_OPT_DEL_NO_PEND,
		(OS_ERR *)&error
		);
	if (error != OS_ERR_NONE) {
        DBG("%s error code = %d.\n", __FUNCTION__, error);
        kernel::on_error(ERR_OPERATION_FAILED, this);

		return false;
	}
    if (count > 0) {
        WRN("%d tasks waiting on the event flag group are now readied and informed.\n", count);
        return false;
    }

	free((void *)_os_handle);
	_os_handle = NULL;

	return true;
}

BOOL event::pend(s32 timeoutms)
{
    s32 error = OS_ERR_NONE;
	u32 flags = 0;

	u32 tick = kernel::convertmstotick(timeoutms);

	//函数原型: OS_FLAGS OSFlagPend(OS_FLAG_GRP *p_grp, OS_FLAGS flags, OS_TICK timeout, OS_OPT opt,CPU_TS *p_ts, OS_ERR *p_err)
	//opt: OS_OPT_PEND_BLOCKING
	//	   OS_OPT_PEND_NON_BLOCKING
	flags = OSFlagPend(
		(OS_FLAG_GRP *)_os_handle,
		(OS_FLAGS)0x00000001,
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
    if (flags == 0) {
        WRN("flah = %d ,a timeout or an error occurred.\n", flags);
        return false;
    }

	return true;
}

BOOL event::post(s32 timeoutms)
{
	s32 error = OS_ERR_NONE;
	s32 dlyerror = OS_ERR_NONE;
	u32	flags = 0;
	//函数原型: OS_FLAGS OSFlagPost(OS_FLAG_GRP *p_grp, OS_FLAGS flags, OS_OPT opt, OS_ERR *p_err)
	//opt: OS_OPT_POST_FLAG_SET       set
	//	   OS_OPT_POST_FLAG_CLR       cleared
loopOSEventPost:
	flags = OSFlagPost(
		(OS_FLAG_GRP *)_os_handle,
		(OS_FLAGS)0x00000001,
		(OS_OPT)OS_OPT_POST_FLAG_SET,
		(OS_ERR *)&error
		);
	if (error == OS_ERR_SEM_OVF) {
		if (timeoutms == 0) { goto out; }
		if (timeoutms < 0)	{ timeoutms++; }
		timeoutms--;
		OSTimeDly(kernel::convertmstotick(1), OS_OPT_TIME_PERIODIC, (OS_ERR *)&dlyerror);
		if (dlyerror != OS_ERR_NONE) {
            DBG("%s error code = %d.\n", __FUNCTION__, error);
            kernel::on_error(ERR_OPERATION_FAILED, this);

			return false;
		}
		goto loopOSEventPost;
	}

out:
	if (error != OS_ERR_NONE) {
        DBG("%s error code = %d.\n", __FUNCTION__, error);
        kernel::on_error(ERR_OPERATION_FAILED, this);

		return false;
	}
    if (flags == 0) {
        WRN("the new value=%d of the event flags bits that are still set.\n", flags);
        return false;
    }

	return true;
}

}
/***********************************************************************
** End of file
***********************************************************************/
