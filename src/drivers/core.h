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
#pragma once
#include "leader_type.h"
#include "leader_misc.h"

/*
 * 时间探测器，用于测量指定起始位运行到指定末尾位置之间所消耗的时间
 * note 时间戳为32位计数器，在168MHz下，溢出周期 = (0xFFFFFFFF+1) / 168M = 25.6s
 * 		因此测量中难免遇到t1 < t0,这将导致测到的时间概率性的出错
 * TODO:使用64位时间戳
 */
#define START_PROFILE()                     { u32 __fc = driver::core::get_cpu_freq();              \
                                              u32 __t0 = driver::core::get_timestamp();

#define STOP_PROFILE(title)                   u32 __t1 = driver::core::get_timestamp();              \
                                              u32 __t  = 1000000u * (__t1 - __t0) / __fc;          \
                                              INF("PROFILE[%s] = %d us.\n", (title), ((u32)__t)); }


/*
 * 设备超时操作宏定义
 */
#define wait_condition_count(_condition, _count, _retaddr)                  \
    do														        \
    {														        \
        *(s32 *)(_retaddr) = 0;									        \
        u32 cnt = _count;										        \
        for(; (_condition) == 0; cnt--)					                    \
        {															\
            if(cnt == 0) {										\
                *(s32 *)(_retaddr) = -1;							\
                break;												\
            }														\
        }															\
    }while(0)


#define wait_condition_us(_condition, _timeoutus, _retaddr)                 \
	do																\
	{																\
		*(s32 *)(_retaddr) = 0;										\
		u32 us = _timeoutus;									        \
		for(; (_condition) == 0; us--)					\
		{															\
			if(us == 0) {									\
				*(s32 *)(_retaddr) = -1;							\
				break;												\
			}														\
			driver::core::udelay(1);								\
		}															\
	}while(0)

#define wait_condition_ms(_condition, _timeoutms, _retaddr)	\
	do																\
	{																\
		*(s32 *)(_retaddr) = 0;										\
		u32 ms = _timeoutms;									\
		for(; (_condition) == 0; ms--)					\
		{															\
			if(ms == 0) {									\
				*(s32 *)(_retaddr) = -1;							\
				break;												\
			}														\
			driver::core::mdelay(1);								\
		}															\
	}while(0)


namespace driver {

class core
{
public:
	core(void);
	~core(void);

public:
    static u32 s_freq_mhz;
    static u32 s_freq_khz;

public:
    static s32 init(void);
    static void mdelay(u32 ms);
    static void udelay(u32 us);

    static u32 get_cpu_freq(void);
    static u32 get_timestamp(void);

    static void system_clock_config(void);
    static void dwt_config(void);

    static void self_test(void);


};

}
/***********************************************************************
** End of file
***********************************************************************/


