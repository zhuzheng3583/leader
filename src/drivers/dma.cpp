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
#include "dma.h"

//id = [1,12]
#define DMA1_CHANNEL_NUM	 	7
#define DMA2_CHANNEL_NUM	 	5
#define DMA_CHANNEL_NUM	 	(DMA1_CHANNEL_NUM + DMA2_CHANNEL_NUM)

namespace driver {

dma::dma(PCSTR name, s32 id) : 
	device(name, id)
{

}

dma::~dma(void)
{

}
	
s32 dma::probe(void)
{
	//ASSERT(_id <= DMA_CHANNEL_MAX_NUM && _id > 0);
	if (device::probe() < 0) {
		CAPTURE_ERR();
		//ERR("%s: failed to probe.\n", _name);
		goto fail0;
	}

	// _dma_id = [1,2]
	// _channel_id = (dma1)[1,7]
	// _channel_id = (dma2)[1,5]
	if (_id <= DMA1_CHANNEL_NUM) {
		_dma_id = 1;
		_channel_id = _id;
	} else if (_id <= DMA_CHANNEL_NUM) {
		_dma_id = 2;
		_channel_id = _id - DMA1_CHANNEL_NUM;	
	}

	DMA_HandleTypeDef *hdma = &dma_hw_table[_id].DMA_Handle;
	if(HAL_DMA_GetState(hdma) != HAL_DMA_STATE_RESET)
	{
		//ERR("%s: failed HAL_DMA_GetState.\n", _name);
		goto fail1;	
	}
	
	switch(_dma_id)
    	{
	case 1:
		__HAL_RCC_DMA1_CLK_ENABLE();
		break;
	case 2:
		__HAL_RCC_DMA2_CLK_ENABLE();
		break;
	default:
		break;
    	}

	_irq = dma_hw_table[_id].IRQn;
	_handle = (u32)hdma;

	/*  Configure the NVIC for DMA */
  	/* NVIC configuration for DMA transfer complete interrupt*/
	interrupt::request_irq(_irq, this);
	interrupt::enable_irq(_irq); 	
	//INF("%s: probe success.\n", _name);
	return 0;

fail2:
fail1:
	device::remove();
fail0:
	free(hdma);
	return -1;	
}

s32 dma::remove(void)
{
	DMA_HandleTypeDef *hdma = (DMA_HandleTypeDef *)_handle;
	if (device::remove() < 0) {
		goto fail0;
	}

	interrupt::disable_irq(_irq); 
	
	if(HAL_DMA_DeInit(hdma)!= HAL_OK) {
    		goto fail1;  
	}
	_handle = NULL;

	return 0;

fail1:
fail0:
    return -1;
}

s32 dma::config(enum dma_dir mode, enum dma_align align, enum dma_pri priority, 
	s32 en_src_step, s32 en_dst_step)
{
	DMA_HandleTypeDef *hdma = (DMA_HandleTypeDef *)_handle;
	/* Configure the DMA */
	/* Configure the DMA handler for Transmission process */
    switch (mode)
    {
	case DMA_DIR_PERIPH_TO_MEM:
		hdma->Init.Direction = DMA_PERIPH_TO_MEMORY;
        	hdma->Init.MemInc = en_src_step ? DMA_PINC_ENABLE : DMA_PINC_DISABLE;	    	
        	hdma->Init.PeriphInc = en_dst_step ? DMA_MINC_ENABLE : DMA_MINC_DISABLE;
        	break;
    	case DMA_DIR_MEM_TO_PERIPH:
		hdma->Init.Direction = DMA_MEMORY_TO_PERIPH;
        	hdma->Init.MemInc = en_src_step ? DMA_MINC_ENABLE : DMA_MINC_DISABLE;
        	hdma->Init.PeriphInc = en_dst_step ? DMA_PINC_ENABLE : DMA_PINC_DISABLE;
		break;
    	case DMA_DIR_MEM_TO_MEM:
		hdma->Init.Direction = DMA_MEMORY_TO_MEMORY;
		hdma->Init.MemInc = en_src_step ? DMA_MINC_ENABLE : DMA_MINC_DISABLE;	    	
		hdma->Init.PeriphInc = en_dst_step ? DMA_MINC_ENABLE : DMA_MINC_DISABLE;
        break;
    	default:
        //ERR("%s: error DMA direct mode.\n", _name);
        CAPTURE_ERR();
        break;
    }
	
	switch (align)
	{
    	case DMA_ALIGN_BYTE:
  		hdma->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  		hdma->Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
		break;
    	case DMA_ALIGN_HALF_WORD:
  		hdma->Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  		hdma->Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
        	break;
    	case DMA_ALIGN_WORD:
  		hdma->Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  		hdma->Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
        	break;
    	default:
		//ERR("%s: error DMA align.\n", _name);
		CAPTURE_ERR();
		break;
	}
	
 	switch (priority)
	{
    	case DMA_PRI_LOW:
		hdma->Init.Priority = DMA_PRIORITY_LOW;
		break;
    	case DMA_PRI_MEDIUM:
  		hdma->Init.Priority = DMA_PRIORITY_MEDIUM;
        	break;
    	case DMA_PRI_HIGH:
  		hdma->Init.Priority = DMA_PRIORITY_HIGH;
        	break;
    	case DMA_PRI_VERY_HIGH:
  		hdma->Init.Priority = DMA_PRIORITY_VERY_HIGH;
        	break;
    	default:
		//ERR("%s: error DMA priority.\n", _name);
		CAPTURE_ERR();
		break;
	}   
	
  	if(HAL_DMA_Init(hdma) != HAL_OK) {
		//ERR("%s: faile to config.\n", _name);
		CAPTURE_ERR();
		return -1;  
  	}
	
	return 0;
}

//stm32 cube库在每个外设中已经封装了DMA相关，暂不使用改方法
s32 dma::async_copy(void *psrc_addr, void *pdst_addr, u32 size)
{

	DMA_HandleTypeDef *hdma = (DMA_HandleTypeDef *)_handle;
#if 0
	DBG("dma transfer size = %d.\n", size);
	/* Set the UART DMA transfer complete callback */
    	hdma->XferCpltCallback = UART_DMATransmitCplt;

    	/* Set the UART DMA Half transfer complete callback */
    	hdma->XferHalfCpltCallback = UART_DMATxHalfCplt;

    	/* Set the DMA error callback */
    	hdma->XferErrorCallback = UART_DMAError;
#endif
    	/* Enable the DMA channel */
    	HAL_DMA_Start_IT(hdma, (uint32_t)psrc_addr, (uint32_t)pdst_addr, size);

	return 0;
}

//stm32 cube库在每个外设中已经封装了DMA相关，暂不使用改方法
s32 dma::wait_complete(s32 timeoutms)
{
#if 1 //polling
	HAL_StatusTypeDef status;
	status = HAL_DMA_PollForTransfer((DMA_HandleTypeDef *)_handle, HAL_DMA_FULL_TRANSFER, (uint32_t)timeoutms);
	if (status != HAL_OK) {
		return -1;
	}
#else
	//IT
#endif
	return 0;
}

void dma::isr(void)
{
	HAL_DMA_IRQHandler((DMA_HandleTypeDef *)_handle);
}

//TODO 读取传输剩余字节数

}
/***********************************************************************
** End of file
***********************************************************************/

