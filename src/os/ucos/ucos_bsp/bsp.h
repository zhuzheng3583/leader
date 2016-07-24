#ifndef  BSP_PRESENT
#define  BSP_PRESENT


#ifdef   BSP_MODULE
#define  BSP_EXT
#else
#define  BSP_EXT  extern
#endif


#include  <stdio.h>
#include  <stdarg.h>
#include  <cpu.h>
#include  <cpu_core.h>
#include  <lib_def.h>
#include  <lib_ascii.h>
#include  "stm32f4xx.h"

#define  BSP_REG_DEM_CR                           (*(CPU_REG32 *)0xE000EDFC)	//DEMCR寄存器
#define  BSP_REG_DWT_CR                           (*(CPU_REG32 *)0xE0001000)  	//DWT控制寄存器
#define  BSP_REG_DWT_CYCCNT                       (*(CPU_REG32 *)0xE0001004)	//DWT时钟计数寄存器
#define  BSP_REG_DBGMCU_CR                        (*(CPU_REG32 *)0xE0042004)

//DEMCR寄存器的第24位,如果要使用DWT ETM ITM和TPIU的话DEMCR寄存器的第24位置1
#define  BSP_BIT_DEM_CR_TRCENA                    DEF_BIT_24

//DWTCR寄存器的第0位,当为1的时候使能CYCCNT计数器,使用CYCCNT之前应当先初始化
#define  BSP_BIT_DWT_CR_CYCCNTENA                 DEF_BIT_00

#endif


