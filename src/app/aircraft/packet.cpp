/*******************************Copyright (c)***************************
**
** Porject name:	leader
** Created by:	zhuzheng<happyzhull@163.com>
** Created date:	2015/08/28
** Modified by:
** Modified date:
** Descriptions:
**
***********************************************************************/
#include "packet.h"

namespace app {

packet::packet(void) :
    _pheader(NULL)
{
	_attr.revision = PACKET_REVISION;
	_attr.num_acce = 20;
	_attr.num_gyro = 20;
	_attr.num_mpu	= 20;
	_attr.num_magn = 20;
	_attr.num_baro = 10;
	_attr.num_gps	= 2;
	_attr.num_attitude = 1;
	_attr.num_rc = 1;
}

packet::~packet(void)
{

}

BOOL packet::create(const packet_attribute_t* pattr)
{
	u32  packet_size = 0;
	u32  memory_size = 0;
	void *pmemory = NULL;

	if (pattr != NULL) {
		_attr = *pattr;
	}

	packet_size = packet::calc_total_size(&_attr);
#if 0
	/**
     *  @TODO HARDWARE_DATA_ALIGN required
     */
	memory_size = CEIL_ALIGN(packet_size, __UAV_PACKET_DATA_ALIGN);
	pmemory = ALIGNED_NEW(BYTE, memory_size, __UAV_PACKET_DATA_ALIGN);
	DBG("Packet memory align, Alloc mem of %d bytes.\n", memory_size);
#else
	memory_size = packet_size;
	pmemory = new char[memory_size];
	DBG("Packet memory unalign, Alloc mem of %d bytes.\n", memory_size);
#endif
	DBG("Memory address: 0x%08x, memory_size: %d, packet_size%d.\n", pmemory, memory_size, packet_size);
	_pheader = (packet_header_t *)pmemory;
	packet::init(&_attr, _pheader);

	return true;
}

packet_attribute_t *packet::get_attribute(void)
{
	return (&_attr);
}


u32 packet::get_total_size(void)
{
	return _pheader ? _pheader->size : 0;
}

u32 packet::get_item_count(void)
{
    u32 i = 0;
	u32 count = 0;
	item_index_t *pindex = _pheader->item_index_table;

	if (_pheader == NULL) return 0;

	for (i = 0; i < _pheader->item_count; i++) {
		if (pindex[i].size > 0) {
			count++;
		}
	}

	return count;
}

u32 packet::get_packet_tag(void)
{
	return _pheader ? _pheader->packettag : 0;
}

u32 packet::get_config_key(void)
{
	return _pheader ? _pheader->configkey : 0;
}

u32 packet::get_timestamp(void)
{
	return _pheader ? _pheader->timestamp : 0;
}

const packet_header_t* packet::get_packet_data(void) const
{
	return _pheader;
}


void *packet::get_item_data(u32 item_id) const
{
    u32 item_magic = 0;
    item_index_t *pindex = NULL;

    item_magic = ITEM_ID_TO_MAGIC(item_id);
    pindex = _pheader->item_index_table;

    if (item_id < _pheader->item_count) {
         /**
          *  @brief 为了保证取数据时的效率，在创建包时需要保证
          *         各个项目均被创建在与其magic对应的索引ID处。
          */
		if (pindex[item_id].magic != item_magic) {
			ERR("ERROR: packet::get_item_data(): pindex[%d].magic=0x%08x, item_magic=0x%08x.\n",
				item_id, pindex[item_id].magic, item_magic);
		}
		ASSERT(pindex[item_id].magic == item_magic);
		if (pindex[item_id].size == 0)
			return NULL;

		return (((char*)_pheader) + pindex[item_id].offset);
	}

	return NULL;
}

u32 packet::get_item_size(u32 item_id) const
{
    u32 item_magic = 0;
    item_index_t *pindex = NULL;

    item_magic = ITEM_ID_TO_MAGIC(item_id);
    pindex = _pheader->item_index_table;

    if (item_id < _pheader->item_count) {
		/**
          *  @brief 为了保证取数据时的效率，在创建包时需要保证
          *         各个项目均被创建在与其magic对应的索引ID处。
          */
		if (pindex[item_id].magic != item_magic) {
			ERR("ERROR: packet::get_item_size(): pindex[%d].magic=0x%08x, item_magic=0x%08x.\n",
				item_id, pindex[item_id].magic, item_magic);
		}
		ASSERT(pindex[item_id].magic == item_magic);

		return pindex[item_id].size;
	}

	return 0;
}

u32 packet::calc_total_size(const packet_attribute_t* pattr)
{
	u32 packet_total_size = 0;
	item_index_t item_index[ITEM_COUNT];

	packet_total_size = packet::init_item_index(pattr, item_index);

	return packet_total_size;
}

u32 packet::calc_checksum(const packet_header_t* ppacket)
{
	u32 checksum = 0;

	for (u32 i = 0; i < sizeof(*ppacket) / sizeof(u32); i++) {
		checksum += ((u32 *)ppacket)[i];
	}

	return checksum;
}

/*!
 * 根据PA数据包的属性结构，初始化该包的包头索引数组
 * @param[out] pInfoTable 初始化后的包头索引数组
 * @param[in]  pAttr      PA数据包的属性结构
 * @param[in]  nItemCount 包头索引数组的索引个数，通常为PA_DEF_ITEM_COUNT或更多
 * @return     数据包所需的内存空间的总大小，单位为字节
 */
u32 packet::init_item_index(const packet_attribute_t*	pattr,
	item_index_t pindex[ITEM_COUNT])
{
    /**
     *  @brief init item_index_table
     */
	u32 offset = 0; //offset按顺序叠加偏置
	u32 packet_total_size = 0;

	offset = sizeof(packet_header_t);
	pindex[ID_ITEM_ACCE].magic = ITEM_ID_TO_MAGIC(ID_ITEM_ACCE);
	pindex[ID_ITEM_ACCE].size = sizeof(item_acce_t)  \
    	+ sizeof(data_acce_t) * pattr->num_acce;
	pindex[ID_ITEM_ACCE].offset = offset;
    offset += pindex[ID_ITEM_ACCE].size;

    pindex[ID_ITEM_GYRO].magic = ITEM_ID_TO_MAGIC(ID_ITEM_GYRO);
    pindex[ID_ITEM_GYRO].size = sizeof(item_gyro_t)  \
        + sizeof(data_gyro_t) * pattr->num_gyro,
    pindex[ID_ITEM_GYRO].offset = offset;
    offset += pindex[ID_ITEM_GYRO].size;

    pindex[ID_ITEM_MPU].magic = ITEM_ID_TO_MAGIC(ID_ITEM_MPU);
    pindex[ID_ITEM_MPU].size = sizeof(item_mpu_t)        \
        + sizeof(data_mpu_t) * pattr->num_mpu,
    pindex[ID_ITEM_MPU].offset = offset;
    offset += pindex[ID_ITEM_MPU].size;

    pindex[ID_ITEM_MAGN].magic = ITEM_ID_TO_MAGIC(ID_ITEM_MAGN);
    pindex[ID_ITEM_MAGN].size = sizeof(item_magn_t)  \
        + sizeof(data_magn_t) * pattr->num_magn,
    pindex[ID_ITEM_MAGN].offset = offset;
    offset += pindex[ID_ITEM_MAGN].size;

    pindex[ID_ITEM_BARO].magic = ITEM_ID_TO_MAGIC(ID_ITEM_BARO);
    pindex[ID_ITEM_BARO].size = sizeof(item_baro_t)  \
        + sizeof(data_baro_t) * pattr->num_baro,
    pindex[ID_ITEM_BARO].offset = offset;
    offset += pindex[ID_ITEM_BARO].size;

    pindex[ID_ITEM_GPS].magic = ITEM_ID_TO_MAGIC(ID_ITEM_GPS);
    pindex[ID_ITEM_GPS].size = sizeof(item_gps_t)        \
        + sizeof(data_gps_t) * pattr->num_gps,
    pindex[ID_ITEM_GPS].offset = offset;
    offset += pindex[ID_ITEM_GPS].size;

   	pindex[ID_ITEM_ATTITUDE].magic = ITEM_ID_TO_MAGIC(ID_ITEM_ATTITUDE);
    pindex[ID_ITEM_ATTITUDE].size = sizeof(item_attitude_t)  \
        + sizeof(data_attitude_t) * pattr->num_attitude,
    pindex[ID_ITEM_ATTITUDE].offset = offset;
    offset += pindex[ID_ITEM_ATTITUDE].size;

   	pindex[ID_ITEM_RC].magic = ITEM_ID_TO_MAGIC(ID_ITEM_RC);
    pindex[ID_ITEM_RC].size = sizeof(item_rc_t)  \
       	+ sizeof(data_rc_t) * pattr->num_rc,
    pindex[ID_ITEM_RC].offset = offset;
    offset += pindex[ID_ITEM_RC].size;

	/**
     *  @brief packet tail
     */
	offset += sizeof(packet_tail_t);

	//packet total size
	packet_total_size = offset;

	return packet_total_size;
}


BOOL packet::init(const packet_attribute_t *pattr, packet_header_t *ppacket)
{
	u32 item_count = ITEM_COUNT;
	u32 packet_total_size = 0;

    /**
     *  @brief init packet header
     */
	memset(ppacket, 0, sizeof(packet_header_t));
	ppacket->magic        = PACKET_SOP_MAGIC;
	ppacket->revision     = pattr->revision;
	ppacket->size         = 0;
	ppacket->checksum     = 0;
	ppacket->packettag    = 0;
    	ppacket->configkey    = 0;
    	ppacket->timestamp    = 0;
    	ppacket->item_count   = item_count;

    /**
     *  @brief init item_index_table
     */
	packet_total_size = packet::init_item_index(pattr,
		ppacket->item_index_table);

	ppacket->size = packet_total_size;


	/**
     *  @brief init item data
     */
 	item_acce_t *pacc   = (item_acce_t *)packet::get_item_data(ID_ITEM_ACCE);
	item_gyro_t *pgyro  = (item_gyro_t *)packet::get_item_data(ID_ITEM_GYRO);
    	item_mpu_t  *pmpu   = (item_mpu_t *)packet::get_item_data(ID_ITEM_MPU);
    	item_magn_t *pmagn  = (item_magn_t *)packet::get_item_data(ID_ITEM_MAGN);
    	item_baro_t *pbaro  = (item_baro_t *)packet::get_item_data(ID_ITEM_BARO);
    	item_gps_t  *pgps   = (item_gps_t *)packet::get_item_data(ID_ITEM_GPS);
    	item_attitude_t *pattitude = (item_attitude_t *)packet::get_item_data(ID_ITEM_ATTITUDE);
	item_rc_t *prc = (item_rc_t *)packet::get_item_data(ID_ITEM_RC);

    	if (pacc) {
		pacc->num   		= pattr->num_acce;
        	pacc->timestamp 	= 0;
        	pacc->data_acce  	= (data_acce_t *)((u32)pacc + sizeof(item_acce_t));
    	}

	if (pgyro) {
        	pgyro->num  		= pattr->num_gyro;
        	pgyro->timestamp 	= 0;
        	pgyro->data_gyro 	= (data_gyro_t *)((u32)pgyro + sizeof(item_gyro_t));
    	}

    	if (pmpu) {
        	pmpu->num   		= pattr->num_mpu;
        	pmpu->timestamp  	= 0;
        	pmpu->data_mpu   	= (data_mpu_t *)((u32)pmpu + sizeof(item_mpu_t));
    	}

    	if (pmagn) {
        	pmagn->num  		= pattr->num_magn;
        	pmagn->timestamp 	= 0;
        	pmagn->data_magn 	= (data_magn_t *)((u32)pmagn + sizeof(item_magn_t));
    	}

    	if (pbaro) {
        	pbaro->num  		= pattr->num_gyro;
        	pbaro->timestamp 	= 0;
        	pbaro->data_baro 	= (data_baro_t *)((u32)pbaro + sizeof(item_baro_t));
    	}

    	if (pgps) {
        	pgps->num   		= pattr->num_gps;
        	pgps->timestamp  	= 0;
        	pgps->data_gps   	= (data_gps_t *)((u32)pgps + sizeof(item_gps_t));
    	}

    	if (pattitude) {
        	pattitude->num  			= pattr->num_attitude;
        	pattitude->timestamp     	= 0;
        	pattitude->data_attitude  	= (data_attitude_t *)((u32)pattitude + sizeof(item_attitude_t));
    	}

	if (prc) {
        	prc->num  		= pattr->num_rc;
        	prc->timestamp   	= 0;
        	prc->data_rc		= (data_rc_t *)((u32)prc + sizeof(item_rc_t));
    	}

    /**
     *  @brief init packet tail
     */
	u32 offset = packet_total_size - sizeof(packet_tail_t);
    	((u32*)((u32)ppacket + offset))[0] = 0xBFBFBFBF;
    	((u32*)((u32)ppacket + offset))[1] = 0xBFBFBFBF;
    	((u32*)((u32)ppacket + offset))[2] = 0xBFBFBFBF;
    	((u32*)((u32)ppacket + offset))[3] = PACKET_EOP_MAGIC;


    	ppacket->checksum = packet::calc_checksum(ppacket);

    	return true;
}

}
/***********************************************************************
** End of file
***********************************************************************/
