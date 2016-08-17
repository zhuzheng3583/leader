/*******************************Copyright (c)***************************
**
** Porject name:	leader
** Created by:		zhuzheng<happyzhull@163.com>
** Created date:	2016/04/05
** Modified by:
** Modified date:
** Descriptions:
**
***********************************************************************/
#include "device.h"

namespace driver {

device::device(void) :
	_name(NULL),
    _id(-1),
    _handle(NULL),
    _devname(NULL),
    _devid(-1),
    _devhandle(NULL),
    _irq(-1),
    _probed(0),
    _opened(0)
{
	/* TODO register*/

}

device::device(PCSTR name, s32 id) :
	_name(name),
    _id(id),
    _handle(NULL),
    _devname(name),
    _devid(id),
    _devhandle(NULL),
    _irq(-1),
    _probed(0),
    _opened(0)
{

}

device::~device(void)
{
	/* TODO unregister*/
}

s32 device::probe(void)
{
	if (_probed != 0) {
		ERR("%s: Has been probed.\n", _devname);
		return -1;
	}

	_probed = 1;
	return 0;
}

s32 device::is_probed(void)
{
	if (_probed != 0) {
		return 0;
	}

	ERR("%s: Has no been probed.\n", _devname);
	return -1;
}

s32 device::remove(void)
{
	if (_probed == 0) {
		ERR("%s: Has been removed.\n", _devname);
		return -1;
	}

	_probed = 0;
	_handle = NULL;

	return 0;
}

s32 device::open(s32 flags)
{
	if (_opened != 0) {
		ERR("%s: Has been opened.\n", _devname);
		return -1;
	}

	_opened = 1;
	return 0;
}

s32 device::close(void)
{
	if (_opened == 0) {
		ERR("%s: Has been close.\n", _devname);
		return -1;
	}

	_opened = 0;
	_handle = NULL;

	return 0;
}

s32 device::read(u8 *buf, u32 size)
{
	WRN("Called to device base class, Please check...\n");
	return 0;
}

s32 device::write(u8 *buf, u32 size)
{
	WRN("Called to device base class, Please check...\n");
	return 0;
}



s32 device::ioctl(enum ioctl_cmd cmd, u32 arg)
{
	WRN("Called to device base class, Please check...\n");
	return 0;
}

s32 device::seek(s32 offset, enum seek_mode mode)
{
	WRN("Called to device base class, Please check...\n");
	return 0;
}

s32 device::tell(void)
{
	WRN("Called to device base class, Please check...\n");
	return 0;
}

s32 device::flush(void)
{
	WRN("Called to device base class, Please check...\n");
	return 0;

}


/**
 * Interrupt dispatch table entry.
 */
struct irq_entry {
	s32	    irq;
	device  *owner;
};

//id = [0,81]
#define STM32F4xx_USER_IRQNUM_MAX	    82
static irq_entry irq_entries[STM32F4xx_USER_IRQNUM_MAX];	/**< interrupt dispatch table (XXX should be a vector) */

/*
 * 初始化中断
 * @return	操作成功返回 0
 */
s32 device::irq_init(void)
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
s32 device::request_irq(s32 irq, device *owner)
{
	if (irq >= ARRAYSIZE(irq_entries) || owner == NULL) {
		return -1;
	}

	irq_entries[irq].irq = irq;
	irq_entries[irq].owner = owner;

	HAL_NVIC_SetPriority((IRQn_Type)(irq_entries[irq].irq), 1, 1);

	return 0;
}

void device::free_irq(s32 irq)
{
	if (irq < ARRAYSIZE(irq_entries)) {
        irq_entries[irq].irq = -1;
        irq_entries[irq].owner = NULL;
	}
}

void device::enable_irq(s32 irq)
{
	HAL_NVIC_EnableIRQ((IRQn_Type)(irq_entries[irq].irq));
}

void device::disable_irq(s32 irq)
{
	HAL_NVIC_DisableIRQ((IRQn_Type)(irq_entries[irq].irq));
}

void device::enable_all_irq(void)
{

}

void device::disable_all_irq(void)
{

}

void device::isr(void)
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
void SysTick_Handler(void)
{
#if defined(USE_HAL_DRIVER)
    HAL_IncTick();
#if (USE_STM32F4_DEMO)
    /* Call user callback */
    HAL_SYSTICK_IRQHandler();
#endif
#endif
    osSystickHandler();
}

void USART1_IRQHandler(void)
{
	irq_entries[USART1_IRQn].owner->isr();
}
void DMA2_Stream5_IRQHandler(void)
{
	irq_entries[DMA2_Stream5_IRQn].owner->isr();
}
void DMA2_Stream7_IRQHandler(void)
{
	irq_entries[DMA2_Stream7_IRQn].owner->isr();
}

void USART2_IRQHandler(void)
{
	irq_entries[USART2_IRQn].owner->isr();
}
void DMA1_Stream5_IRQHandler(void)
{
	irq_entries[DMA1_Stream5_IRQn].owner->isr();
}
void DMA1_Stream6_IRQHandler(void)
{
	irq_entries[DMA1_Stream6_IRQn].owner->isr();
}

void USART3_IRQHandler(void)
{
	irq_entries[USART3_IRQn].owner->isr();
}
void DMA1_Stream1_IRQHandler(void)
{
	irq_entries[DMA1_Stream1_IRQn].owner->isr();
}
void DMA1_Stream3_IRQHandler(void)
{
	irq_entries[DMA1_Stream3_IRQn].owner->isr();
}


void DMA2_Stream2_IRQHandler(void)
{
	irq_entries[DMA2_Stream2_IRQn].owner->isr();
}
void DMA2_Stream3_IRQHandler(void)
{
	irq_entries[DMA2_Stream3_IRQn].owner->isr();
}


#if 0
//I2C1_DMA_TX
void DMA1_Channel6_IRQHandler(void)
{
	irq_entries[DMA1_Channel6_IRQn].owner->isr();
}
//I2C1_DMA_RX
void DMA1_Channel7_IRQHandler(void)
{
	irq_entries[DMA1_Channel7_IRQn].owner->isr();
}


//EXTI
void EXTI0_IRQHandler(void)
{
	irq_entries[EXTI0_IRQn].owner->isr();
}
void EXTI1_IRQHandler(void)
{
	irq_entries[EXTI1_IRQn].owner->isr();
}
void EXTI2_IRQHandler(void)
{
	irq_entries[EXTI2_TSC_IRQn].owner->isr();
}
void EXTI3_IRQHandler(void)
{
	irq_entries[EXTI3_IRQn].owner->isr();
}
void EXTI4_IRQHandler(void)
{
	irq_entries[EXTI4_IRQn].owner->isr();
}

#endif

void TIM2_IRQHandler(void)
{
	irq_entries[TIM2_IRQn].owner->isr();
}
void TIM3_IRQHandler(void)
{
	irq_entries[TIM3_IRQn].owner->isr();
}

#ifdef __cplusplus
}
#endif


}

/***********************************************************************
** End of file
***********************************************************************/
