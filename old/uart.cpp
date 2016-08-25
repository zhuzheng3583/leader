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
#include "uart.h"
#include "core.h"

#define UART_POLLING_TIMEOUT_MS 	1000
#define UART_IT_TIMEOUT_MS			10
#define UART_DMA_TIMEOUT_MS 		10

#define UART_POLLING_MODE			(1 << 0)
#define UART_IT_MODE				(1 << 1)
#define UART_DMA_MODE				(1 << 2)
#define UART_MODE 				UART_POLLING_MODE


namespace driver {

struct stm32_uart_hw_table
{
	GPIO_TypeDef  		*GPIOx;
	IRQn_Type 			IRQn;
	UART_HandleTypeDef	UART_Handle;
	GPIO_InitTypeDef  	GPIO_Init;

	s32                 dma_tx_id;
	s32             	dma_rx_id;
	uart 				*puart;
};

static struct stm32_uart_hw_table uart_hw_table[] = {
	[0] = { NULL },
	[1] = {
		.GPIOx = GPIOB,
		.IRQn = USART1_IRQn,
		.UART_Handle = {
			.Instance = USART1,
			.Init = {
				.BaudRate   = 9600,
				.WordLength = UART_WORDLENGTH_8B,
				.StopBits   = UART_STOPBITS_1,
				.Parity     = UART_PARITY_NONE,
				.HwFlowCtl  = UART_HWCONTROL_NONE,
				.Mode       = UART_MODE_TX_RX,
			},
		},
		.GPIO_Init = {
			.Pin       = GPIO_PIN_6 | GPIO_PIN_7,
			.Mode      = GPIO_MODE_AF_PP,
			.Pull      = GPIO_NOPULL,
			.Speed     = GPIO_SPEED_FREQ_HIGH,
			.Alternate = GPIO_AF7_USART1,
		},
		.dma_tx_id = 124,
		.dma_rx_id = 108,
	},
	[2] = {
		.GPIOx = GPIOA,
		.IRQn = USART2_IRQn,
		.UART_Handle = {
			.Instance = USART2,
			.Init = {
				.BaudRate   = 115200,
				.WordLength = UART_WORDLENGTH_8B,
				.StopBits   = UART_STOPBITS_1,
				.Parity     = UART_PARITY_NONE,
				.HwFlowCtl  = UART_HWCONTROL_NONE,
				.Mode       = UART_MODE_TX_RX,
			},
		},
		.GPIO_Init = {
			.Pin       = GPIO_PIN_2 | GPIO_PIN_3,
			.Mode      = GPIO_MODE_AF_PP,
			.Pull      = GPIO_NOPULL,
			.Speed     = GPIO_SPEED_FREQ_HIGH,
			.Alternate = GPIO_AF7_USART2,
		},
		.dma_tx_id = 52,
		.dma_rx_id = 44,
	},
	[3] = {
		.GPIOx = GPIOD,//GPIOC,
		.IRQn = USART3_IRQn,
		.UART_Handle = {
			.Instance = USART3,
			.Init = {
				.BaudRate   = 115200,
				.WordLength = UART_WORDLENGTH_8B,
				.StopBits   = UART_STOPBITS_1,
				.Parity     = UART_PARITY_NONE,
				.HwFlowCtl  = UART_HWCONTROL_NONE,
				.Mode       = UART_MODE_TX_RX,
			},
		},
		.GPIO_Init = {
			.Pin       = GPIO_PIN_8 | GPIO_PIN_9,//GPIO_PIN_10 | GPIO_PIN_11,
			.Mode      = GPIO_MODE_AF_PP,
			.Pull      = GPIO_NOPULL,
			.Speed     = GPIO_SPEED_FREQ_HIGH,
			.Alternate = GPIO_AF7_USART3,
		},
		.dma_tx_id = 28,
		.dma_rx_id = 12,
	},
};

//id = [1,3]
uart::uart(PCSTR name, s32 id) :
	device(name, id),
	_baudrate(115200),
	_flag_tx(0),
	_flag_rx(0)
{
	// For stm32_cube_lib C callback
	uart_hw_table[_id].puart = this;
}

uart::~uart(void)
{
	uart_hw_table[_id].puart = NULL;
}

s32 uart::probe(void)
{
	UART_HandleTypeDef *huart = &uart_hw_table[_id].UART_Handle;
	if(HAL_UART_GetState(huart) != HAL_UART_STATE_RESET)
	{
		//ERR("%s: failed HAL_UART_GetState.\n", _name);
		goto fail0;
	}

	//void HAL_UART_MspInit(UART_HandleTypeDef *huart)
    	switch(_id)
    	{
	case 1:
		/* 1- Enable USARTx peripherals and GPIO Clocks */
		//__HAL_RCC_GPIOE_CLK_ENABLE();
		__HAL_RCC_GPIOB_CLK_ENABLE();
		__HAL_RCC_USART1_CLK_ENABLE();
		break;
	case 2:
		__HAL_RCC_GPIOA_CLK_ENABLE();
  		__HAL_RCC_USART2_CLK_ENABLE();
		break;
	case 3:
        __HAL_RCC_GPIOD_CLK_ENABLE();
		__HAL_RCC_GPIOC_CLK_ENABLE();
  		__HAL_RCC_USART3_CLK_ENABLE();
		break;
	default:
		break;
    	}
	/* 2- Configure peripheral GPIO */
	/* UART TX/RX GPIO pin configuration  */
	GPIO_TypeDef  *GPIOx = uart_hw_table[_id].GPIOx;
	GPIO_InitTypeDef *GPIO_Init= &uart_hw_table[_id].GPIO_Init;
	HAL_GPIO_Init(GPIOx, GPIO_Init);

	_irq = uart_hw_table[_id].IRQn;
	_dma_tx_id = uart_hw_table[_id].dma_tx_id;
	_dma_rx_id = uart_hw_table[_id].dma_rx_id;

	/* 3- Configure the UART peripheral*/
  	if(HAL_UART_Init(huart) != HAL_OK) {
		goto fail1;
  	}
	_handle = (u32)huart;

#if (UART_MODE == UART_DMA_MODE)
	s8 str[16];
	snprintf((char *)str, 16, "dma-%d", _dma_tx_id);
	_dmatx = new dma((PCSTR)str, _dma_tx_id);
	//INF("%s: new dmatx %s[dma%d,stream%d,channel%d].\n", _name, str, _dmatx->_dma_id, _dmatx->_stream_id, _dmatx->_channel_id);
	_dmatx->probe();
	_dmatx->config(DMA_DIR_MEM_TO_PERIPH, DMA_ALIGN_BYTE, DMA_PRI_LOW, 1, 0);
	/* Associate the initialized DMA handle to the UART handle */
	__HAL_LINKDMA(huart, hdmatx, *(DMA_HandleTypeDef *)(_dmatx->_handle));

    snprintf((char *)str, 16, "dma-%d", _dma_rx_id);
	_dmarx = new dma((PCSTR)str, _dma_rx_id);
	//INF("%s: new dmarx %s[dma%d,stream%d,channel%d].\n", _name, str, _dmarx->_dma_id, _dmarx->_stream_id, _dmarx->_channel_id);
	_dmarx->probe();
	_dmarx->config(DMA_DIR_PERIPH_TO_MEM, DMA_ALIGN_BYTE, DMA_PRI_HIGH, 0, 1);
	__HAL_LINKDMA(huart, hdmarx, *(DMA_HandleTypeDef *)(_dmarx->_handle));
#endif
#if (UART_MODE == UART_IT_MODE || UART_MODE == UART_DMA_MODE)
	/*##-3- Configure the NVIC for UART ########################################*/
	/* NVIC for USART */
	device::request_irq(_irq, this);
	device::enable_irq(_irq);
#endif

	//INF("%s: probe success.\n", _name);
	return 0;

fail1:
	HAL_GPIO_DeInit(GPIOx, GPIO_Init->Pin);
fail0:
	return -1;
}

s32 uart::remove(void)
{
	UART_HandleTypeDef *huart = (UART_HandleTypeDef *)_handle;
#if (UART_MODE == UART_IT_MODE || UART_MODE == UART_DMA_MODE)
	device::disable_irq(_irq);
#endif
#if (UART_MODE == UART_DMA_MODE)
	_dmatx->remove();
	_dmarx->remove();
	delete _dmatx;
	delete _dmarx;
	_dmatx = NULL;
	_dmarx = NULL;
#endif
	if(HAL_UART_DeInit(huart)!= HAL_OK) {
        goto fail0;
	}

	//void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
    	switch(_id)
    	{
	case 1:

		/*##-1- Reset peripherals ##################################################*/
	 	__HAL_RCC_USART1_FORCE_RESET();
	 	__HAL_RCC_USART1_RELEASE_RESET();
		break;
	case 2:
		__HAL_RCC_USART2_FORCE_RESET();
	 	__HAL_RCC_USART2_RELEASE_RESET();
		break;
	case 3:
		__HAL_RCC_USART3_FORCE_RESET();
	 	__HAL_RCC_USART3_RELEASE_RESET();
		break;
	default:
		break;
    	}
	/*##-2- Disable peripherals and GPIO Clocks #################################*/
	GPIO_TypeDef  *GPIOx = uart_hw_table[_id].GPIOx;
	uint32_t GPIO_Pin = uart_hw_table[_id].GPIO_Init.Pin;
	HAL_GPIO_DeInit(GPIOx, GPIO_Pin);

	_handle = NULL;
	return 0;

fail0:
    return -1;
}

s32 uart::recv(u8 *buf, u32 count)
{
    u32 readcnt = 0;

#if (UART_MODE == UART_POLLING_MODE)
	if(HAL_UART_Receive((UART_HandleTypeDef*)_handle, (uint8_t*)buf, count, UART_POLLING_TIMEOUT_MS) != HAL_OK) {
    		//CAPTURE_ERR();
	}
    readcnt = count;
#elif (UART_MODE == UART_IT_MODE)
	if(HAL_UART_Receive_IT((UART_HandleTypeDef*)_handle, (uint8_t*)buf, count) != HAL_OK) {
		//CAPTURE_ERR();
	}
	//pend _flag_rx
	s32 ret = 0;
	wait_condition_ms(_flag_rx == 1, UART_IT_TIMEOUT_MS, &ret);
	if (ret < 0) {
		return -1;
	} else {
		_flag_rx = 0;
	}
    readcnt = count;
#elif (UART_MODE == UART_DMA_MODE)
	if(HAL_UART_Receive_DMA((UART_HandleTypeDef*)_handle, (uint8_t*)buf, count) != HAL_OK) {
		//CAPTURE_ERR();
	}
	//pend _flag_rx
	s32 ret = 0;
	wait_condition_ms(_flag_rx == 1, UART_DMA_TIMEOUT_MS, &ret);
	if (ret < 0) {
		readcnt = count - _dmarx->get_leftover_count();
	} else {
		_flag_rx = 0;
		readcnt = count;
	}
    
#endif


	return readcnt;
}

s32 uart::send(u8 *buf, u32 count)
{
    u32 writecnt = 0;

#if (UART_MODE == UART_POLLING_MODE)
	if(HAL_UART_Transmit((UART_HandleTypeDef*)_handle, (uint8_t*)buf, count, UART_POLLING_TIMEOUT_MS) != HAL_OK) {
        //CAPTURE_ERR();
	}
    writecnt = count;
#elif (UART_MODE == UART_IT_MODE)
	if(HAL_UART_Transmit_IT((UART_HandleTypeDef*)_handle, (uint8_t*)buf, count) != HAL_OK) {
		//CAPTURE_ERR();
	}
	//pend tx_event
	s32 ret = 0;
	wait_condition_ms(_flag_tx == 1, UART_IT_TIMEOUT_MS, &ret);
	if (ret < 0) {
		return -1;
	} else {
		_flag_tx = 0;
	}
    writecnt = count;
#elif (UART_MODE == UART_DMA_MODE)
	if(HAL_UART_Transmit_DMA((UART_HandleTypeDef*)_handle, (uint8_t*)buf, count) != HAL_OK) {
		//CAPTURE_ERR();
	}
	//pend tx_event
	s32 ret = 0;
	wait_condition_ms(_flag_tx == 1, UART_DMA_TIMEOUT_MS, &ret);
	if (ret < 0) {
		writecnt = count - _dmatx->get_leftover_count();
	} else {
		_flag_tx = 0;
        	writecnt = count;
	}
    
#endif


	return writecnt;
}

s32 uart::self_test(void)
{
	uart::open();
#if 1
	u8 wbuf[16] = "hello world!";
	u8 rbuf[16] = { 0 };

    uart::write(wbuf, ARRAYSIZE(wbuf));
    while (1) {
      uart::read(rbuf, ARRAYSIZE(rbuf));
      //INF("%s", rbuf);

      uart::write(rbuf, ARRAYSIZE(wbuf));
      core::mdelay(500);
    }
#else
	u32 n = 0;
	while (1) {
		INF("%d-%d-%d-%d-%d-%d-%d-%d-%d-%d-%d-%d-%d-%d-%d-%d.\n",
			n, n, n, n, n, n, n, n, n, n, n, n, n, n, n, n);
		n++;
	}
	core::mdelay(100);
#endif
	uart::close();
}

s32 uart::read(u8 *buf, u32 size)
{
	return recv(buf, size);
}

s32 uart::write(u8 *buf, u32 size)
{
	return send(buf, size);
}

void uart::isr(void)
{
	HAL_UART_IRQHandler((UART_HandleTypeDef *)_handle);
}

#ifdef __cplusplus
extern "C" {
#endif

#if (UART_MODE == UART_IT_MODE || UART_MODE == UART_DMA_MODE)
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	//post tx_event
	uart *puart = NULL;
	for (s32 id = 0; id < ARRAYSIZE(uart_hw_table); id++) {
		if (uart_hw_table[id].puart->_id == id) {
			puart = uart_hw_table[id].puart;
		}
	}

	if (puart != NULL) {
		puart->_flag_tx = 1;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//post rx_event
	uart *puart = NULL;
	for (s32 id = 0; id < ARRAYSIZE(uart_hw_table); id++) {
		if (uart_hw_table[id].puart->_id == id) {
			puart = uart_hw_table[id].puart;
		}
	}

	if (puart != NULL) {
		puart->_flag_rx = 1;
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	uart *puart = NULL;
	for (s32 id = 0; id < ARRAYSIZE(uart_hw_table); id++) {
		if (uart_hw_table[id].puart->_id == id) {
			puart = uart_hw_table[id].puart;
		}
	}

	if (puart != NULL) {
	}
}
#endif

#ifdef __cplusplus
}
#endif

}



/***********************************************************************
** End of file
***********************************************************************/

