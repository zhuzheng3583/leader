/*******************************Copyright (c)***************************
** 
** Porject name:	leader
** Created by:	zhuzheng<happyzhull@163.com>
** Created date:	2016/07/11
** Modified by:
** Modified date:
** Descriptions:
**
***********************************************************************/
#pragma once
#include "leader_type.h"
#include "leader_misc.h"

#include "core.h"
#include "interrupt.h"
#include "device.h"
#include "dma.h"

namespace driver {

enum dma_dir {
    DMA_DIR_PERIPH_TO_MEM	= 0,
    DMA_DIR_MEM_TO_PERIPH 	= 1,
    DMA_DIR_MEM_TO_MEM 	= 2,
};

enum dma_align {
    DMA_ALIGN_BYTE      = 0,
    DMA_ALIGN_HALF_WORD = 1,
    DMA_ALIGN_WORD      = 2,
};

enum dma_pri {
    DMA_PRI_LOW        = 0,
    DMA_PRI_MEDIUM     = 1,
    DMA_PRI_HIGH       = 2,
    DMA_PRI_VERY_HIGH  = 3,
};


class dma : public device, public irq_handler
{
public:
	dma(PCSTR name, s32 id);
	~dma(void);

public:
    s32 _dma_id;
    s32 _channel_id;

public:
    s32 probe(void);
    s32 remove(void);

public:
    s32 config(enum dma_dir mode, enum dma_align align, enum dma_pri priority, 
        s32 en_src_step, s32 en_dst_step);
    s32 async_copy(void *psrc_addr, void *pdst_addr, u32 size);
    s32 wait_complete(s32 timeoutms);
    
public:
    virtual void isr(void); 
};

struct stm32f3_dma_hw_table
{
	DMA_TypeDef 			*DMAx;
	IRQn_Type 			IRQn;
	DMA_HandleTypeDef		DMA_Handle;
};

static struct stm32f3_dma_hw_table dma_hw_table[] = {
	[0] = { NULL },
#if 0
	[4] = { 
		.DMAx = DMA1,
		.IRQn = DMA1_Channel4_IRQn,
		.DMA_Handle = { 
			.Instance = DMA1_Channel4,
			.Init = {
				.Mode = DMA_NORMAL,
			},
		},
	},
	[5] = { 
		.DMAx = DMA1,
		.IRQn = DMA1_Channel5_IRQn,
		.DMA_Handle = { 
			.Instance = DMA1_Channel5,
			.Init = {
				.Mode = DMA_NORMAL,
			},
		},
	},
	[6] = { 
		.DMAx = DMA1,
		.IRQn = DMA1_Channel6_IRQn,
		.DMA_Handle = { 
			.Instance = DMA1_Channel6,
			.Init = {
				.Mode = DMA_NORMAL,
			},
		},
	},
	[7] = { 
		.DMAx = DMA1,
		.IRQn = DMA1_Channel7_IRQn,
		.DMA_Handle = { 
			.Instance = DMA1_Channel7,
			.Init = {
				.Mode = DMA_NORMAL,
			},
		},
	},
#endif
};

}
/***********************************************************************
** End of file
***********************************************************************/







