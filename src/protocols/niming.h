/*******************************Copyright (c)***************************
**
** Porject name:	leader
** Created by:	zhuzheng<happyzhull@163.com>
** Created date:	2016/04/05
** Modified by:
** Modified date:
** Descriptions:
**
***********************************************************************/
#if 0
#pragma once
#include "type.h"

#define NIMING_V26							1

#if NIMING_V26
static void niming_report(u8 fun, u8 *data, u8 len);

void mpu_send_data(
	s16 aacx, s16 aacy, s16 aacz,
	s16 gyrox, s16 gyroy, s16 gyroz
	);

void imu_send_data(
	s16 aacx,s16 aacy,s16 aacz,
	s16 gyrox,s16 gyroy,s16 gyroz,
	s16 roll,s16 pitch,s16 yaw
	);
#else

#endif
#endif

#pragma once
#include "leader_type.h"
#include "leader_misc.h"
    
#include "uart.h"
#include "packet.h"

using namespace driver;

namespace app {
    
class niming
{
public:
    niming(void);
    ~niming(void);

public:
    uart *_uart;
    
public:
    void attach(uart *puart);
    void detach(void);
    
    void report_status(data_attitude_t *atti);
    void report_sensor(data_mpu_t *mpu, data_magn_t *magn);
    void report_rc(data_rc_t *rc);
};
    
}



/***********************************************************************
** End of file
***********************************************************************/

