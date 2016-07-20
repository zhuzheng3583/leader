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
#include "core.h"


#define time_after(a,b)		((s64)(b) - (s64)(a) < 0)
#define time_before(a,b)		time_after(b,a)
#define time_after_eq(a,b) 	((s64)(a) - (s64)(b) >= 0)
#define time_before_eq(a,b)	time_after_eq(b,a)

namespace driver {

u32 core::s_freq_khz = 0;
u32 core::s_freq_mhz = 0;

core::core(void)
{

}

core::~core(void)
{

}

/*
 * 初始化延迟函数
 */
s32 core::init(void)
{
  /* STM32F4xx HAL library initialization:
       - Configure the Flash prefetch, instruction and Data caches
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Global MSP (MCU Support Package) initialization
     */
  	HAL_Init();

  	/* Configure the system clock to 168 MHz */
  	core::system_clock_config();
	core::dwt_config();

	u32 sysclk = core::get_cpu_freq();
	s_freq_khz = sysclk / 1000;
	s_freq_mhz = sysclk / 1000000;

	return 0;
}

void core::dwt_config(void)
{
    	//DEMCR寄存器的第24位,如果要使用DWT ETM ITM和TPIU的话DEMCR寄存器的第24位置1
	#define  BIT_DEM_CR_TRCENA		(1 << 24)
	//DWTCR寄存器的第0位,当为1的时候使能CYCCNT计数器,使用CYCCNT之前应当先初始化
	#define  BIT_DWT_CR_CYCCNTENA	(1 << 0)

    CoreDebug->DEMCR |= BIT_DEM_CR_TRCENA;  //使用DWT  Enable Cortex-M4's DWT CYCCNT reg.   
	DWT->CYCCNT = 0u;					    //初始化CYCCNT寄存器
	DWT->CTRL |= BIT_DWT_CR_CYCCNTENA;	    //开启CYCCNT
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 168000000
  *            HCLK(Hz)                       = 168000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 24000000//8000000 HSE_VALUE
  *            PLL_M                          = 24//8
  *            PLL_N                          = 336
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
void core::system_clock_config(void)
{
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;

  	/* Enable Power Control clock */
  	__HAL_RCC_PWR_CLK_ENABLE();
  
  	/* The voltage scaling allows optimizing the power consumption when the device is 
     	clocked below the maximum system frequency, to update the voltage scaling value 
     	regarding system frequency refer to product datasheet.  */
  	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  
  	/* Enable HSE Oscillator and activate PLL with HSE as source */
  	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;

#if !defined  (HSE_VALUE) 
#error "No macro definition HSE_VALUE, Please check..."
#endif

    if (HSE_VALUE == 24000000U) {
      RCC_OscInitStruct.PLL.PLLM = 24;
    } else if (HSE_VALUE == 8000000U) {
      RCC_OscInitStruct.PLL.PLLM = 8;
    } else {
      return;
    }
  	RCC_OscInitStruct.PLL.PLLN = 336;
  	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  	RCC_OscInitStruct.PLL.PLLQ = 7;
  	HAL_RCC_OscConfig(&RCC_OscInitStruct);
  
  	/* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     	clocks dividers */
  	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  	/* STM32F405x/407x/415x/417x Revision Z devices: prefetch is supported  */
  	if (HAL_GetREVID() == 0x1001)
  	{
    		/* Enable the Flash prefetch */
    		__HAL_FLASH_PREFETCH_BUFFER_ENABLE();
  	}
    
    return;
}

/*
 * 毫秒延迟（暴力循环）
 * ms  延迟时间，单位：ms
 */
void core::mdelay(u32 ms)
{
#if 1
	u32 t0 = 0;
	while (ms--) {
		t0 = DWT->CYCCNT;
		while (((u32)(DWT->CYCCNT - t0)) < s_freq_khz);
	}
#elif 0
	HAL_Delay(ms);
#else
	u32 timeout = SysTick->VAL + ms * s_freq_khz;
	while (time_after(SysTick->VAL, timeout));
#endif
}

/*
 * 微秒延迟（暴力循环）
 * us  延迟时间，单位：us
 */
void core::udelay(u32 us)
{
#if 1
	u32 t0 = 0;
	while (us--) {
		t0 = DWT->CYCCNT;
		while (((u32)(DWT->CYCCNT - t0)) < s_freq_mhz);
	}
#else
	u32 timeout = SysTick->VAL + us * s_freq_mhz;
	while (time_after(SysTick->VAL, timeout));
#endif
}

/*
 * 获取当前的CPU运行频率（主频）
 * return 当前的CPU运行频率（主频）
 */
u32 core::get_cpu_freq(void)
{
#if 1
	return HAL_RCC_GetSysClockFreq();
#else
	return 168000000u;
#endif
}

/*
 * 获取时间戳
 * return 当前CPU时间戳
 * note 根据CPU运行频率(默认168M)进行计数(32位计数器)
 */
u32 core::get_timestamp(void)
{
	return DWT->CYCCNT;
}

#if 0
s32  timestamp_to_us (u32 ts_cnts)
{
    CPU_INT64U  ts_us;
    CPU_INT64U  fclk_freq;

    fclk_freq = BSP_CPU_ClkFreq();
    ts_us     = ts_cnts / (fclk_freq / DEF_TIME_NBR_uS_PER_SEC);

    return (ts_us);
}

CPU_INT64U  CPU_TS64_to_uSec (CPU_TS64  ts_cnts)
{
    CPU_INT64U  ts_us;
    CPU_INT64U  fclk_freq;

    fclk_freq = BSP_CPU_ClkFreq();
    ts_us     = ts_cnts / (fclk_freq / DEF_TIME_NBR_uS_PER_SEC);

    return (ts_us);
}
#endif


void core::self_test(void)
{
	s32 state = 0;
	u32 cnt = 0;
	u32 ms = 0;
	u32 us = 0;
    char str[64];

    INF("Start test PROFILE....\n");
    for (us = 1; us <= 1000000; us = us * 10) {
        sprintf(str, "udelay[%d us] profile", us);
        START_PROFILE()
        core::udelay(us);
        STOP_PROFILE(str);
    }
    for (ms = 1; ms <= 2001; ms = ms + 500) {
        sprintf(str, "mdelay[%d ms] profile", ms);
        START_PROFILE()
        core::mdelay(ms);
        STOP_PROFILE(str);
    }
    INF("End test PROFILE....\n");



	cnt = 1000;
    INF("Start wait_condition_cntout[%d times] test...\n", cnt);
    wait_condition_count(0 != 1, cnt, &state);
    if (state < 0) {
        INF("Succeed testing wait_condition_cntout, state = %d.\n", state);
    } else {
        INF("Failed to wait_condition_cntout test, state = %d.\n", state);
    }
    INF("End wait_condition_cntout test.\n");

    us = 1000000;
    INF("Start wait_condition_timeoutus[%d us] test...\n", us);
    wait_condition_us(0 != 1, us, &state);
    if (state < 0) {
        INF("Succeed testing wait_condition_timeoutus, state = %d.\n", state);
    } else {
        INF("Failed to wait_condition_timeoutus test, state = %d.\n", state);
    }
    INF("End wait_condition_timeoutus test.\n");

    ms = 1000;
    INF("Start wait_condition_timeoutms[%d ms] test...\n", ms);
    wait_condition_ms(0 != 1, ms, &state);
    if (state < 0) {
        INF("Succeed testing wait_condition_timeoutms, state = %d.\n", state);
    } else {
        INF("Failed to wait_condition_timeoutms test, state = %d.\n", state);
    }
    INF("End wait_condition_timeoutms test.\n");
}

}
/***********************************************************************
** End of file
***********************************************************************/

