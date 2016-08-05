/*******************************Copyright (c)***************************
**
** Porject name:	leader
** Created by:	zhuzheng<happyzhull@163.com>
** Created date:	2016/07/23
** Modified by:
** Modified date:
** Descriptions:
**
***********************************************************************/

#include "transmit.h"

#include "leader_system.h"
#include "niming.h"

namespace app {

transmit::transmit(void)
{
	_params.name = "transmit";
	_params.priority = 0;
	_params.stacksize = 1024;
	_params.func = (void *)thread::func;
	_params.parg = this;
}

transmit::~transmit(void)
{

}

void transmit::run(void *parg)
{
	transmit *p = (transmit *)parg;

    u32 cnt = 0;
	u32 msg_data = 0;
	u32 msg_size = 0;
	msgque *sync_ct = leader_system::get_instance()->get_sync_ct();
	niming *niming = leader_system::get_instance()->get_niming();

	u32 packet_addr = 0;
	packet *ppacket  = NULL;
	packet_attribute_t *pattr = NULL;
	item_mpu_t  *pitem_mpu  = NULL;
	item_magn_t *pitem_magn = NULL;
	item_baro_t *pitem_baro = NULL;
	item_gps_t  *pitem_gps  = NULL;
	item_attitude_t *pitem_attitude = NULL;
	item_rc_t *pitem_rc = NULL;
	for ( ; ;)
	{
		// 等待calc任务发送的消息队列，获取数据包缓冲区首地址
		sync_ct->pend(&packet_addr, NULL, 1000);
		ppacket = (packet *)packet_addr;
		pattr = ppacket->get_attribute();

		/* TODO:发起DMA传输，将各个传感器数据分通道发送至上位机 */
		pitem_mpu  = (item_mpu_t *)(ppacket->get_item_data(ID_ITEM_MPU));
		pitem_magn  = (item_magn_t *)(ppacket->get_item_data(ID_ITEM_MAGN));
		if (pattr->num_mpu == pattr->num_magn) {
			for (u32 i = 0; i < pattr->num_mpu; i++) {
				niming->report_sensor(&pitem_mpu->data_mpu[i], &pitem_magn->data_magn[i]);
			}
		}

		pitem_attitude  = (item_attitude_t *)(ppacket->get_item_data(ID_ITEM_ATTITUDE));
		for (u32 i = 0; i < pattr->num_attitude; i++) {
			niming->report_status(&pitem_attitude->data_attitude[i]);
		}

		pitem_rc  = (item_rc_t *)(ppacket->get_item_data(ID_ITEM_RC));
		for (u32 i = 0; i < pattr->num_rc; i++) {
			niming->report_rc(&pitem_rc->data_rc[i]);
		}
        msleep(0);
		//msleep(20);
		//DBG("%s: task is active[%u]...\n", _name, cnt++);
		//msleep(10);
	}
}

}
/***********************************************************************
** End of file
***********************************************************************/

