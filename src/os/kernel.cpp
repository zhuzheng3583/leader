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
#include "os/kernel.h"

#include "driver/core.h"
#include "includes.h"
#include "os_cfg_app.h"
/*
CPU_CFG_CRITICAL_METHOD

控制进入临界区代码模式
OS_CFG_ISR_POST_DEFERRED_EN 在os_cfg.h文件中定义
0:使用直接发布模式:禁止中断
	对临界段代码采用关闭中断的保护措施，这样就会延长中断的响应时间
1:使用延迟发布模式:禁止调度
	不是通过关中断，而是通过给任务调度器上锁的方法来保护临界段代码，
	在延迟发布模式下基本不存在关闭中断的情况

//进入临界区和退出临界区
#define  OS_CRITICAL_ENTER()                    CPU_CRITICAL_ENTER()
#define  OS_CRITICAL_EXIT()                     CPU_CRITICAL_EXIT()
#define  OS_CRITICAL_EXIT_NO_SCHED()            CPU_CRITICAL_EXIT()
*/

#define INIT_CRITICAL_SECTION() 		CPU_SR_ALLOC()		// 初始化临界区
#define ENTER_CRITICAL_SECTION()		OS_CRITICAL_ENTER() // 进入临界区
#define EXIT_CRITICAL_SECTION() 		OS_CRITICAL_EXIT()	// 退出临界区


/**
 * 系统Tick中断服务函数
 */
extern "C" void SysTick_Handler(void)
{
	OSIntEnter();							// 进入中断
    OSTimeTick();       					// 调用ucos的时钟服务程序
    OSIntExit();        					// 触发任务切换软中断
}



namespace os {

kernel::kernel(void)
{

}

kernel::~kernel(void)
{

}

/**
 *  初始化操作系统Tick
 */
void kernel::systick_init(void)
{
	/*
	 * 根据OS_TICKS_PER_SEC设定溢出时间
	 * reload为24位寄存器,最大值:16777216,在168M下,约合0.7989s左右
	 */
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
	u32 sysclk = driver::core::get_cpu_freq();
	u32 reload = sysclk / 8 / OS_CFG_TICK_RATE_HZ;
#if 0
	if (SysTick_Config(reload))
	{
		/* Capture error */
		while (1);
	}
#elif 1
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk; 	// 开启SYSTICK中断
	/* TODO: OS_CFG_TICK_RATE_HZ 直接更改reload/2，看能否缩短tick周期两倍 */
	SysTick->LOAD = reload;					 	// 每1/OS_CFG_TICK_RATE_HZ秒中断一次
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;  	// 开启SYSTICK
#else
	OS_CPU_SysTickInit(OS_CFG_TICK_RATE_HZ);
#endif
}

/**
 * 初始化操作系统
 * note 在调用操作系统的任何API之前初始化操作系统
 */
void kernel::init(void)
{
    s32 error = 0;

    kernel::systick_init();

	// 初始化UCOSIII
	OSInit((OS_ERR *)&error);
	if (error != OS_ERR_NONE)
	{
		ERR("error: %s failed, error = %d.\n", __FUNCTION__, error);
	}
}

/**
 *  启动操作系统
 */
void kernel::start(void)
{
	s32 error = 0;
	// 开启UCOSIII
	OSStart((OS_ERR *)&error);
	if (error != OS_ERR_NONE)
	{
		ERR("error: %s failed, error = %d.\n", __FUNCTION__, error);
	}
}

/**
 *  退出操作系统
 */
void kernel::exit(void)
{

}

/**
 *  ?????CPU???
 *  return ????CPU???,??????(%)
 *  note   ???? OS_StatTask()
 */
u32 kernel::get_cpu_load(void)
{
	return 0;
}

/**
 *  ???????Tick???
 *  return ????Tick??,???us
 */
u32 kernel::get_tick_period(void)
{
    return (1 * 1000000 / OS_CFG_TICK_RATE_HZ);
}

/**
 *  ??????ms??????????Tick??
 *  ms	   ?????????,??(ms)
 *  return ????????Tick?,??(tick)
 *  note:?ms?-1?,????OSTimeDly???????
 */
u32 kernel::convertmstotick(s32 ms)
{
	/**
	 * ucosiii???:
	 * 1.??OSTimeDly,tick=0?????????(No delay will result if the specified delay is 0.)
	 * 2.?????????????,?????,tick=0???????
	 *   (wait forever until the resource becomes available or the event occurs.)
	 */
	u32 tick = 0;
	u32 period = get_tick_period();
	if (ms < 0)			// ????
	{
		tick = 0;
	}
	else if (ms == 0)	// ???
	{
		tick = 1;
	}
	else				// ??ms?????
	{
		//?????:tick = ms * 1000 / period;
		//tick = CEIL_DIV(ms * 1000, period);
		tick = (ms * 1000 + period - 1) / period;
	}

	return tick;
}

void kernel::on_error(enum os_error_code errcode, os_object* pobject)
{
	static const char* error_message_tab[] =
	{
		"err_no",
		"err_unknown",
		"err_invalid_handle",
		"err_invalid_param",
		"err_out_memory",
		"err_operation_timeout",
		"err_magque_full",
		"err_operation_failed",
		"err_operation_unsupported",
		"err_application",
	};

	// Default error handler
	ERR("OS ERROR: %s [Code=0x%08x, Object=0x%08x, Name=%s, Handle=0x%08x].\n",
		errcode < ARRAYSIZE(error_message_tab) ? error_message_tab[errcode] : "",
		errcode,
		pobject,
		pobject ? pobject->get_name() : "<NULL>",
		pobject ? pobject->get_handle() : 0
		);
}

}
/***********************************************************************
** End of file
***********************************************************************/