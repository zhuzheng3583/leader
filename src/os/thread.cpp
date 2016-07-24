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
#include "os/thread.h"

#include "includes.h"

namespace os {

thread::thread(void)
{

}

thread::~thread(void)
{

}

BOOL thread::create(struct thread_params *pparams)
{
    s32 error = OS_ERR_NONE;
    if (pparams != NULL) {
        _params = *pparams;
    }

    /*
     * 以下优先级用户不能使用
     * 优先级0：中断服务服务管理任务 OS_IntQTask()
     * 优先级1：时钟节拍任务 OS_TickTask()
     * 优先级2：定时任务 OS_TmrTask()
     * 优先级OS_CFG_PRIO_MAX-2：统计任务 OS_StatTask()
     * 优先级OS_CFG_PRIO_MAX-1：空闲任务 OS_IdleTask()
     */
    if (_params.priority > OS_CFG_PRIO_MAX - 3) {
        _params.priority = OS_CFG_PRIO_MAX - 3;
    }
    else if (_params.priority < 3) {
        _params.priority = 3;
    }

    if (_params.stacksize < OS_CFG_STK_SIZE_MIN) {
        _params.stacksize = OS_CFG_STK_SIZE_MIN;
    }

    _name   = _params.name;
    _handle = (HANDLE)malloc(sizeof(OS_TCB));
    //_handle = (HANDLE)new char[sizeof(OS_TCB)];
    if (_handle == NULL) {
        goto fail0;
    }
    /* TODO: 动态分配任务堆栈 */
    _params.stackbase = (u32)malloc(_params.stacksize * 4);
    if (_params.stackbase == NULL) {
        goto fail1;
    }

    /*
     * 创建任务:UCOSIII要求任务创建期间必须进入临界区
     */
    CPU_SR_ALLOC();
    OS_CRITICAL_ENTER();        // 进入临界区
    OSTaskCreate(
        (OS_TCB *)      (_handle),                 // 任务控制块
        (CPU_CHAR *)    (_params.name),             // 任务名字
        (OS_TASK_PTR)   (_params.func),              // 任务函数
        (void *)        (_params.parg),             // 传递给任务函数的参数
        (OS_PRIO)       (_params.priority),         // 任务优先级
        (CPU_STK *)     (_params.stackbase),        // 任务堆栈基地址
        (CPU_STK_SIZE)  ((_params.stacksize)/10),   // 任务堆栈深度限位
        (CPU_STK_SIZE)  (_params.stacksize),        // 任务堆栈大小
        (OS_MSG_QTY)    0,                          // 任务内部消息队列能够接收的最大消息数目,为0时禁止接收消息
        (OS_TICK)       0,                          // 当使能时间片轮转时的时间片长度，为0时为默认长度，
        (void *)        0,                          // 用户补充的存储区
        (OS_OPT)        (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR), // 任务选项
        (OS_ERR *)      &error                      // 存放该函数错误时的返回值(0:正确返回，<0:出错返回)
        );
    OS_CRITICAL_EXIT();         // 退出临界区

    if (error != OS_ERR_NONE) {
        DBG("%s error code = %d.\n", __FUNCTION__, error);
        kernel::on_error(ERR_OPERATION_FAILED, this);
        goto fail2;
    }

    DBG("OSTaskCreate: %s\n", _params.name);
    return true;

fail2:
    free((void *)_params.stackbase);
    _params.stackbase = NULL;
fail1:
    free((void *)_handle);
    _handle = NULL;
fail0:
    return false;
}

BOOL thread::t_delete(void)
{
	s32 error = OS_ERR_NONE;

	//OSTaskDel(0, (OS_ERR *)&error);
	OSTaskDel((OS_TCB *)(_handle), (OS_ERR *)&error);
	if (error != OS_ERR_NONE) {
        DBG("%s error code = %d.\n", __FUNCTION__, error);
        kernel::on_error(ERR_OPERATION_FAILED, this);
		return false;
	}

	free((void *)_params.stackbase);
    _params.stackbase = NULL;

	free((void *)_handle);
    _handle = NULL;

	return true;
}

/**
 *  使当前任务睡眠指定时间(不精确延迟，精确延迟使用定时器中断POST信号量)
 *  timeoutms 睡眠时间（单位：ms）
 *  note 1.当timeoutms>=0时，使当前任务睡眠timeoutms时间，直到超时或被唤醒
 *		 2.当timeoutms<0时，使当前任务永久睡眠(最多睡眠0xffffffff tich)，直到被唤醒
 */
void thread::msleep(s32 timeoutms)
{
    s32 error = OS_ERR_NONE;
	u32 tick = 0;
	if (timeoutms < 0) {
		// note:当ms为-1时，不适用于OSTimeDly的延迟参数转换
		tick = 0xffffffff;
		//OSTimeDlyHMSM(99, 59, 59, 999, OS_OPT_TIME_PERIODIC/*OS_OPT_TIME_HMSM_STRICT*/, (OS_ERR *)&error);
	}
	else {
		tick = kernel::convertmstotick(timeoutms);
		//OSTimeDlyHMSM(0, 0, 0, timeoutms, OS_OPT_TIME_PERIODIC/*OS_OPT_TIME_HMSM_STRICT*/, (OS_ERR *)&error);
	}
	OSTimeDly (tick, OS_OPT_TIME_PERIODIC, (OS_ERR *)&error);
	if (error != OS_ERR_NONE) {
        DBG("%s error code = %d.\n", __FUNCTION__, error);
        kernel::on_error(ERR_OPERATION_FAILED, this);
    }

}

BOOL thread::func(thread* pthread)
{
	INF("START======================>\n");
	INF("name: %s.\n",          pthread->_params.name);
	INF("priority: %d.\n",      pthread->_params.priority);
	INF("stackbase: 0x%08x.\n", pthread->_params.stackbase);
	INF("stacksize: 0x%08x.\n", pthread->_params.stacksize);
	INF("END<========================\n");

	pthread->run(pthread->_params.parg);

	return true;
}

void thread::run(void *parg)
{
	ERR("Pure virtual function called: %s(...).\n", __FUNCTION__);
	ASSERT(0);
}


BOOL thread::set_param(struct thread_params *param)
{
    if (param == NULL) return false;
    _params = *param;
    return true;
}

BOOL thread::get_param(struct thread_params *param) const
{
    if (param == NULL) return false;
    *param = _params;
    return true;
}

}

/***********************************************************************
** End of file
***********************************************************************/

