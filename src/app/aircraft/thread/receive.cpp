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
#include "receive.h"

#include "leader_system.h"

namespace app {

receive::receive(void)
{
	_params.name = "receive";
	_params.priority = 0;
	_params.stacksize = 512;
	_params.func = (void *)thread::func;
	_params.parg = this;
}

receive::~receive(void)
{

}

void receive::run(void *parg)
{
  receive *p = (receive *)parg;
    u32 cnt = 0;
    u32 msg_data = 1;
    msgque *sync_rc = leader_system::get_instance()->get_sync_rc();

    mpu6000 *mpu6000 = leader_system::get_instance()->get_mpu6000();

	for (cnt = 0; ;cnt++)
	{
        sync_rc->post((void *)0xffffffff/*&msg_data*/, sizeof(msg_data), 1000);
        
        INF("%s: task is active[%u]...\n", _name, cnt);
        msleep(1);
    }
    #if 0
    Mpu6000 *pmpu6000 = SystemUav::get_system_uav()->get_mpu6000();
    Packet  *ppacket  = SystemUav::get_system_uav()->get_packet();
    packet_attribute_t *pattr = &(ppacket->m_attr);

    item_mpu_t  *pitem_mpu  = (item_mpu_t *)(ppacket->get_item_data(ID_ITEM_MPU));
    item_magn_t *pitem_magn = (item_magn_t *)(ppacket->get_item_data(ID_ITEM_MAGN));
    item_baro_t *pitem_baro = (item_baro_t *)(ppacket->get_item_data(ID_ITEM_BARO));
    item_gps_t  *pitem_gps  = (item_gps_t *)(ppacket->get_item_data(ID_ITEM_GPS));
    item_attitude_t *pitem_attitude = (item_attitude_t *)(ppacket->get_item_data(ID_ITEM_ATTITUDE));

    data_mpu_t *data_mpu_buf = pitem_mpu->data_mpu;
    uint32_t data_mpu_size = pattr->num_raw_mpu * sizeof(*(pitem_mpu->data_mpu));

	for (uint32_t cnt = 0, index = 0; ;)
	{
		/* TODO: 调用驱动程序传感器数据读取函数，发起DMA传输 */
		// 获取mpu传感器数据
		// 发送消息队列给calc任务，将数据缓冲区首地址推向calc任务
		//msgque_post(rece.syncq_calc, &packet[index], msg_size, -1);
		//DBG("%s[%d]: post: msg[0x%08x], size[%d].\n",
		//	rece.ptask->taskname, cnt, &packet[index], msg_size);

		// 协调传感器更新率和电机控制频率
		//task_sleep(500);

		INF("active: %s.\n", m_param.name);
        pmpu6000->read((int8_t *)data_mpu_buf, data_mpu_size);

        for (uint32_t i = 0; i < pattr->num_raw_mpu; i++)
        {
            INF("\rax=%6.5d, ay=%6.5d, az=%6.5d, gx=%6.5d, gy=%6.5d, gz=%6.5d.",
    		    data_mpu_buf[i].acce_x,
    		    data_mpu_buf[i].acce_y,
    		    data_mpu_buf[i].acce_z,
    		    data_mpu_buf[i].gyro_x,
    		    data_mpu_buf[i].gyro_y,
    		    data_mpu_buf[i].gyro_z
		        );
        }

       // m_pmsgque->post(ppacket, NULL, WAIT_FOREVER);
        msleep(300);
	}
    #endif
}

}
/***********************************************************************
** End of file
***********************************************************************/


