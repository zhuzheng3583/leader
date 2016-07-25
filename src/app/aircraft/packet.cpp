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
    _ppacket(NULL)
{
	_attr.revision = PACKET_REVISION;
	_attr.num_raw_acce = 20;
	_attr.num_raw_gyro = 20;
	_attr.num_raw_mpu	= 20;
	_attr.num_raw_magn	= 10;
	_attr.num_raw_baro = 10;
	_attr.num_raw_gps	= 2;
	_attr.num_raw_attitude = 1;
}

packet::~packet(void)
{

}

BOOL packet::create(const packet_attribute_t* pattr)
{
    BOOL b = false;
    u32  packet_size = 0;
    u32  memory_size = 0;
    void *pmemory = NULL;

    if (pattr != NULL) {
        _attr = *pattr;
    }

    packet_size = packet::calculate_total_size(&_attr);

#if 0
	/**
     *  @TODO HARDWARE_DATA_ALIGN required
     */
	DBG("packet memory align.\n");
    memory_size = CEIL_ALIGN(packet_size, __UAV_PACKET_DATA_ALIGN);
    DBG("Alloc mem of %d bytes.\n", memory_size);
    pmemory   = ALIGNED_NEW(BYTE, memory_size, __UAV_PACKET_DATA_ALIGN);
    DBG("Memory base: 0x%08x.\n", pmemory);
#else
	DBG("packet memory unalign, packet_size = %d.\n", packet_size);
	memory_size = packet_size;
	pmemory = new char[memory_size];
#endif
    _pheader = (packet_header_t *)pmemory;
    b = packet::init_packet(&_attr, _pheader, packet_size);
    if (b == false)
        return false;

	return true;
}

BOOL packet::attach(void *pmemory, const packet_attribute_t* pattr)
{
    _ppacket = (packet_header_t *)pmemory;
    packet::init_packet(pattr, _ppacket);

    return true;
}

u32 packet::get_total_size(void)
{
	return _ppacket ? _ppacket->size : 0;
}

u32 packet::get_item_count(void)
{
    u32 i = 0;
	u32 count = 0;
	item_index_t *pindex = _ppacket->index_table;

	if (_ppacket == NULL) return 0;

	for (i = 0; i < _ppacket->item_count; i++) {
		if (pindex[i].size > 0) {
			count++;
		}
	}

	return count;
}

u32 packet::get_packet_tag(void)
{
	return _ppacket ? _ppacket->packettag : 0;
}

u32 packet::get_config_key(void)
{
	return _ppacket ? _ppacket->configkey : 0;
}

u32 packet::get_timestamp(void)
{
	return _ppacket ? _ppacket->timestamp : 0;
}

const packet_header_t* packet::get_packet_data(void) const
{
	return _ppacket;
}


void *packet::get_item_data(u32 item_id) const
{
    u32 item_magic = 0;
    item_index_t *pindex = NULL;

    item_magic = ITEM_ID_TO_MAGIC(item_id);
    pindex = _ppacket->index_table;

    if (item_id < _ppacket->item_count) {
        /**
         *  @brief 为了保证取数据时的效率，在创建包时需要保证
         *         各个项目均被创建在与其magic对应的索引ID处。
         */
        if (pindex[item_id].magic != item_magic) {
            ERR("ERROR: packet::get_item_data(): pindex[%d].magic=0x%08x, item_magic=0x%08x.\n",
                item_id,
                pindex[item_id].magic,
                item_magic
                );
        }
        ASSERT(pindex[item_id].magic == item_magic);
        if (pindex[item_id].size == 0)
            return NULL;

        return (((char*)_ppacket) + pindex[item_id].offset);
    }

    return NULL;
}

u32 packet::get_item_size(u32 item_id) const
{
    u32 item_magic = 0;
    item_index_t *pindex = NULL;

    item_magic = ITEM_ID_TO_MAGIC(item_id);
    pindex = _ppacket->index_table;

    if (item_id < _ppacket->item_count) {
        /**
         *  @brief 为了保证取数据时的效率，在创建包时需要保证
         *         各个项目均被创建在与其magic对应的索引ID处。
         */
        if (pindex[item_id].magic != item_magic) {
            ERR("ERROR: packet::get_item_size(): pindex[%d].magic=0x%08x, item_magic=0x%08x.\n",
                item_id,
                pindex[item_id].magic,
                item_magic
                );
        }
        ASSERT(pindex[item_id].magic == item_magic);

        return pindex[item_id].size;
    }

	return 0;
}

BOOL packet::init(const packet_attribute_t *pattr,
    packet_header_t *ppacket)
{
    u32 item_count = ITEM_COUNT;

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
    item_index_t *pindex = ppacket->item_index_table;

    u32 offset = 0; //offset按顺序叠加偏置
    offset = sizeof(packet_header_t);
    pindex[ID_ITEM_ACCE].magic = ITEM_ID_TO_MAGIC(ID_ITEM_ACCE);
    pindex[ID_ITEM_ACCE].size = sizeof(item_acce_t)  \
        + sizeof(data_acce_t) * pattr->num_raw_acce;
    pindex[ID_ITEM_ACCE].offset = offset;
    offset += pindex[ID_ITEM_ACCE].size;

    pindex[ID_ITEM_GYRO].magic = ITEM_ID_TO_MAGIC(ID_ITEM_GYRO);
    pindex[ID_ITEM_GYRO].size = sizeof(item_gyro_t)  \
        + sizeof(data_gyro_t) * pattr->num_raw_gyro,
    pindex[ID_ITEM_GYRO].offset = offset;
    offset += pindex[ID_ITEM_GYRO].size;

    pindex[ID_ITEM_MPU].magic = ITEM_ID_TO_MAGIC(ID_ITEM_MPU);
    pindex[ID_ITEM_MPU].size = sizeof(item_mpu_t)        \
        + sizeof(data_mpu_t) * pattr->num_raw_mpu,
    pindex[ID_ITEM_MPU].offset = offset;
    offset += pindex[ID_ITEM_MPU].size;

    pindex[ID_ITEM_MAGN].magic = ITEM_ID_TO_MAGIC(ID_ITEM_MAGN);
    pindex[ID_ITEM_MAGN].size = sizeof(item_magn_t)  \
        + sizeof(data_magn_t) * pattr->num_raw_magn,
    pindex[ID_ITEM_MAGN].offset = offset;
    offset += pindex[ID_ITEM_MAGN].size;

    pindex[ID_ITEM_BARO].magic = ITEM_ID_TO_MAGIC(ID_ITEM_BARO);
    pindex[ID_ITEM_BARO].size = sizeof(item_baro_t)  \
        + sizeof(data_baro_t) * pattr->num_raw_baro,
    pindex[ID_ITEM_BARO].offset = offset;
    offset += pindex[ID_ITEM_BARO].size;

    pindex[ID_ITEM_GPS].magic = ITEM_ID_TO_MAGIC(ID_ITEM_GPS);
    pindex[ID_ITEM_GPS].size = sizeof(item_gps_t)        \
        + sizeof(data_gps_t) * pattr->num_raw_gps,
    pindex[ID_ITEM_GPS].offset = offset;
    offset += pindex[ID_ITEM_GPS].size;

    pindex[ID_ITEM_ATTITUDE].magic = ITEM_ID_TO_MAGIC(ID_ITEM_ATTITUDE);
    pindex[ID_ITEM_ATTITUDE].size = sizeof(item_attitude_t)  \
        + sizeof(data_attitude_t) * pattr->num_raw_attitude,
    pindex[ID_ITEM_ATTITUDE].offset = offset;
    offset += pindex[ID_ITEM_ATTITUDE].size;

    /**
     *  @brief End flag
     */
    ((u32*)((u32)ppacket + offset))[0] = 0xBFBFBFBF;
    ((u32*)((u32)ppacket + offset))[1] = 0xBFBFBFBF;
    ((u32*)((u32)ppacket + offset))[2] = 0xBFBFBFBF;
    ((u32*)((u32)ppacket + offset))[3] = PACKET_EOP_MAGIC;
    offset += sizeof(u32) * 4;
    //packet total size
    ppacket->size = offset;


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

    if (pacc) {
        pacc->num_raw_samples   = pattr->num_raw_acce;
        pacc->timestamp         = 0;
        pacc->data_acce         = (data_acce_t *)((u32)pacc + sizeof(item_acce_t));
    }

    if (pgyro) {
        pgyro->num_raw_samples  = pattr->num_raw_gyro;
        pgyro->timestamp        = 0;
        pgyro->data_gyro        = (data_gyro_t *)((u32)pgyro + sizeof(item_gyro_t));
    }

    if (pmpu) {
        pmpu->num_raw_samples   = pattr->num_raw_mpu;
        pmpu->timestamp         = 0;
        pmpu->data_mpu          = (data_mpu_t *)((u32)pmpu + sizeof(item_mpu_t));
    }

    if (pmagn) {
        pmagn->num_raw_samples  = pattr->num_raw_magn;
        pmagn->timestamp        = 0;
        pmagn->data_magn        = (data_magn_t *)((u32)pmagn + sizeof(item_magn_t));
    }

    if (pbaro) {
        pbaro->num_raw_samples  = pattr->num_raw_gyro;
        pbaro->timestamp        = 0;
        pbaro->data_baro        = (data_baro_t *)((u32)pbaro + sizeof(item_baro_t));
    }

    if (pgps) {
        pgps->num_raw_samples   = pattr->num_raw_gps;
        pgps->timestamp         = 0;
        pgps->data_gps          = (data_gps_t *)((u32)pgps + sizeof(item_gps_t));
    }

    if (pattitude) {
        pattitude->num_raw_samples  = pattr->num_raw_attitude;
        pattitude->timestamp        = 0;
        pattitude->data_attitude    = (data_attitude_t *)((u32)pattitude + sizeof(item_attitude_t));
    }


    ppacket->checksum     = packet::calc_checksum(ppacket);

    return true;
}

u32 packet::calc_checksum(const packet_header_t* ppacket)
{
	u32 checksum = 0;

	for (u32 i = 0; i < sizeof(*ppacket) / sizeof(u32); i++) {
		checksum += ((u32 *)ppacket)[i];
	}

	return checksum;
}

}
/***********************************************************************
** End of file
***********************************************************************/
