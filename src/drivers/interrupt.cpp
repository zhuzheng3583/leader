/*******************************Copyright (c)***************************
**
** Porject name:	leader
** Created by:	zhuzheng<happyzhull@163.com>
** Created date:	2016/07/07
** Modified by:
** Modified date:
** Descriptions:
**
***********************************************************************/
#include "interrupt.h"

#include "device.h"

#include "includes.h"
#include "os_cfg_app.h"

#include "cmsis_os.h"

namespace driver {

struct map_table interrupt::s_map[STM32F4xx_USER_IRQNUM_MAX];

interrupt::interrupt(void)
{

}

interrupt::~interrupt(void)
{

}

/*
 * 初始化中断
 * @return	操作成功返回 0
 */
s32 interrupt::irq_init(void)
{
	/*
	 * 分配抢占优先级和响应优先级位域:
	 * NVIC_PRIORITYGROUP_4: 4位抢占优先级，0位响应优先级
	 */
	/* Set Interrupt Group Priority */
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_2);

	return 0;
}

/*
 * 中断资源请求
 * @param[in]	irq		设备中断逻辑编号
 * @param[in]	handler	设备中断服务程序基类
 * @return	操作成功返回 0
 * @note		填充向量表，绑定中断逻辑编号与中断服务程序
 */
s32 interrupt::request_irq(s32 irq, irq_handler *handler)
{
	if (irq >= ARRAYSIZE(s_map) || handler == NULL) {
		return -1;
	}

	s_map[irq].irq = irq;
	s_map[irq].handler = handler;

	HAL_NVIC_SetPriority((IRQn_Type)(s_map[irq].irq), 1, 1);

	return 0;
}

void interrupt::free_irq(s32 irq)
{
	if (irq < ARRAYSIZE(s_map)) {
        s_map[irq].irq = -1;
        s_map[irq].handler = NULL;
	}
}

void interrupt::enable_irq(s32 irq)
{
	HAL_NVIC_EnableIRQ((IRQn_Type)(s_map[irq].irq));
}

void interrupt::disable_irq(s32 irq)
{
	HAL_NVIC_DisableIRQ((IRQn_Type)(s_map[irq].irq));
}

void interrupt::enable_all_irq(void)
{

}

void interrupt::disable_all_irq(void)
{

}

/*
 * 中断向量一级分发表
 * @note TODO:在此添加一级中断向量，二级中断向量在irq_handler的派生类中实现
 */
#ifdef __cplusplus
extern "C" {
#endif


/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
#if 0
void SysTick_Handler(void)
{
#if USE_UCOS3
	OSIntEnter();							// 进入中断
#endif

#if defined(USE_HAL_DRIVER)
    HAL_IncTick();
#if (USE_STM32F4_DEMO)
	/* Call user callback */
	HAL_SYSTICK_IRQHandler();
#endif
#endif
    
#if USE_UCOS3
	OSTimeTick();       					// 调用ucos的时钟服务程序
	OSIntExit();        					// 触发任务切换软中断
#endif
}
#else
void SysTick_Handler(void)
{
    osSystickHandler();
}
#endif

void USART1_IRQHandler(void)
{
#if USE_UCOS3
	OSIntEnter();		//进入中断
#endif
	interrupt::s_map[USART1_IRQn].handler->isr();
#if USE_UCOS3
	OSIntExit();        //触发任务切换软中断
#endif
}

void USART2_IRQHandler(void)
{
#if USE_UCOS3
	OSIntEnter();
#endif
	interrupt::s_map[USART2_IRQn].handler->isr();
#if USE_UCOS3
	OSIntExit();
#endif
}
void DMA1_Stream5_IRQHandler(void)
{
#if USE_UCOS3
	OSIntEnter();
#endif
	interrupt::s_map[DMA1_Stream5_IRQn].handler->isr();
#if USE_UCOS3
	OSIntExit();
#endif
}
void DMA1_Stream6_IRQHandler(void)
{
#if USE_UCOS3
	OSIntEnter();
#endif
	interrupt::s_map[DMA1_Stream6_IRQn].handler->isr();
#if USE_UCOS3
	OSIntExit();
#endif
}

void USART3_IRQHandler(void)
{
#if USE_UCOS3
	OSIntEnter();
#endif
	interrupt::s_map[USART3_IRQn].handler->isr();
#if USE_UCOS3
	OSIntExit();
#endif
}
void DMA1_Stream1_IRQHandler(void)
{
#if USE_UCOS3
	OSIntEnter();
#endif
	interrupt::s_map[DMA1_Stream1_IRQn].handler->isr();
#if USE_UCOS3
	OSIntExit();
#endif
}
void DMA1_Stream3_IRQHandler(void)
{
#if USE_UCOS3
	OSIntEnter();
#endif
	interrupt::s_map[DMA1_Stream3_IRQn].handler->isr();
#if USE_UCOS3
	OSIntExit();
#endif
}


void DMA2_Stream2_IRQHandler(void)
{
#if USE_UCOS3
	OSIntEnter();
#endif
	interrupt::s_map[DMA2_Stream2_IRQn].handler->isr();
#if USE_UCOS3
	OSIntExit();
#endif
}
void DMA2_Stream3_IRQHandler(void)
{
#if USE_UCOS3
	OSIntEnter();
#endif
	interrupt::s_map[DMA2_Stream3_IRQn].handler->isr();
#if USE_UCOS3
	OSIntExit();
#endif
}


#if 0
//I2C1_DMA_TX
void DMA1_Channel6_IRQHandler(void)
{
	interrupt::s_map[DMA1_Channel6_IRQn].handler->isr();
}
//I2C1_DMA_RX
void DMA1_Channel7_IRQHandler(void)
{
	interrupt::s_map[DMA1_Channel7_IRQn].handler->isr();
}


//EXTI
void EXTI0_IRQHandler(void)
{
	interrupt::s_map[EXTI0_IRQn].handler->isr();
}
void EXTI1_IRQHandler(void)
{
	interrupt::s_map[EXTI1_IRQn].handler->isr();
}
void EXTI2_IRQHandler(void)
{
	interrupt::s_map[EXTI2_TSC_IRQn].handler->isr();
}
void EXTI3_IRQHandler(void)
{
	interrupt::s_map[EXTI3_IRQn].handler->isr();
}
void EXTI4_IRQHandler(void)
{
	interrupt::s_map[EXTI4_IRQn].handler->isr();
}

#endif

void TIM2_IRQHandler(void)
{
	interrupt::s_map[TIM2_IRQn].handler->isr();
}
void TIM3_IRQHandler(void)
{
	interrupt::s_map[TIM3_IRQn].handler->isr();
}

#ifdef __cplusplus
}
#endif

}
/***********************************************************************
** End of file
***********************************************************************/

