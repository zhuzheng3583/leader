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

namespace app {

transmit::transmit(void)
{
	_params.name = "transmit";
	_params.priority = 9;
	_params.stackbase = NULL;
	_params.stacksize = 512;
	_params.func = (void *)thread::func;
	_params.parg = this;
}

transmit::~transmit(void)
{

}

void transmit::run(void *parg)
{
    u32 cnt = 0;
    u32 msg_data = 0;
	u32 msg_size = 0;
    msgque *sync_c_t = leader_system::get_instance()->get_sync_c_t();

	for (cnt = 0; ; cnt++)
	{
		// 等待calc任务发送的消息队列，获取数据包缓冲区首地址
		//msgque_pend(tran.syncq_calc, &msg_pend, &msg_size, -1);
		//DBG("%s[%d]: pend: msg[0x%08x], size[%d].\n",
		//	tran.ptask->taskname, cnt, msg_pend, msg_size);
		/* TODO:发起DMA传输，将各个传感器数据分通道发送至上位机 */

        sync_c_t->pend(&msg_data, &msg_size, 1000);

        INF("%s: task is active[%u]...\n", _params.name, cnt);
		msleep(100);
	}
}

}
/***********************************************************************
** End of file
***********************************************************************/

