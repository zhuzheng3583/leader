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
    BOOL b = FALSE;
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
    if (b == FALSE)
        return FALSE;

	return TRUE;
}

BOOL packet::attach(void *pmemory, const packet_attribute_t* pattr)
{
    BOOL b = FALSE;
    u32  packet_size = 0;

    if (pattr == NULL)
        return FALSE;

    packet_size = packet::calculate_total_size(pattr);
    _pheader = (packet_header_t *)pmemory;
    b = packet::init_packet(pattr, _pheader, packet_size);
    if (b == FALSE)
        return FALSE;

    return TRUE;
}

u32 packet::get_total_size(void)
{
	return _pheader ? _pheader->size : 0;
}

u32 packet::get_item_count(void)
{
    u32 i = 0;
	u32 count = 0;
	item_index_t *pindex = _pheader->index_table;

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
    pindex = _pheader->index_table;

    if (item_id < _pheader->item_count) {
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

        return (((char*)_pheader) + pindex[item_id].offset);
    }

    return NULL;
}

u32 packet::get_item_size(u32 item_id) const
{
    u32 item_magic = 0;
    item_index_t *pindex = NULL;

    item_magic = ITEM_ID_TO_MAGIC(item_id);
    pindex = _pheader->index_table;

    if (item_id < _pheader->item_count) {
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



u32 packet::init_item_index_table(
    const packet_attribute_t*   pattr,
    item_index_t                item_index_table[PACKET_DEF_ITEM_COUNT],
    u32                         item_count
    )
{
    u32 i = 0;
    u32 size = 0;
    u32 offset = 0;

    if (pattr == NULL)
        return 0;

    if (item_count != packet::calculate_item_count(pattr) ||
        item_count == 0                                   ||
        item_count > PACKET_DEF_ITEM_COUNT)
        return 0;

    memset(item_index_table, 0 , sizeof(item_index_t) * item_count);

	/**
	 * @brief 初始化item index
	 * @note  offset 按顺序偏置
	 */
	offset = sizeof(packet_header_t);
	item_index_table[ID_ITEM_ACCE].magic = ITEM_ID_TO_MAGIC(ID_ITEM_ACCE);
	item_index_table[ID_ITEM_ACCE].size = sizeof(item_acce_t) 	\
		+ sizeof(data_acce_t) * pattr->num_raw_acce;	
	item_index_table[ID_ITEM_ACCE].offset = offset;
	offset += item_index_table[ID_ITEM_ACCE].size;

	item_index_table[ID_ITEM_GYRO].magic = ITEM_ID_TO_MAGIC(ID_ITEM_GYRO);
	item_index_table[ID_ITEM_GYRO].size = sizeof(item_gyro_t) 	\
		+ sizeof(data_gyro_t) * pattr->num_raw_gyro,
	item_index_table[ID_ITEM_GYRO].offset = offset;
	offset += item_index_table[ID_ITEM_GYRO].size;

	item_index_table[ID_ITEM_MPU].magic = ITEM_ID_TO_MAGIC(ID_ITEM_MPU);
	item_index_table[ID_ITEM_MPU].size = sizeof(item_mpu_t) 		\
		+ sizeof(data_mpu_t) * pattr->num_raw_mpu,
	item_index_table[ID_ITEM_MPU].offset = offset;
	offset += item_index_table[ID_ITEM_MPU].size;

	item_index_table[ID_ITEM_MAGN].magic = ITEM_ID_TO_MAGIC(ID_ITEM_MAGN);
	item_index_table[ID_ITEM_MAGN].size = sizeof(item_magn_t) 	\
		+ sizeof(data_magn_t) * pattr->num_raw_magn,
	item_index_table[ID_ITEM_MAGN].offset = offset;
	offset += item_index_table[ID_ITEM_MAGN].size;
	
	item_index_table[ID_ITEM_BARO].magic = ITEM_ID_TO_MAGIC(ID_ITEM_BARO);
	item_index_table[ID_ITEM_BARO].size = sizeof(item_baro_t) 	\
		+ sizeof(data_baro_t) * pattr->num_raw_baro,
	item_index_table[ID_ITEM_BARO].offset = offset;
	offset += item_index_table[ID_ITEM_BARO].size;

	item_index_table[ID_ITEM_GPS].magic = ITEM_ID_TO_MAGIC(ID_ITEM_GPS);
	item_index_table[ID_ITEM_GPS].size = sizeof(item_gps_t) 		\
		+ sizeof(data_gps_t) * pattr->num_raw_gps,
	item_index_table[ID_ITEM_GPS].offset = offset;
	offset += item_index_table[ID_ITEM_GPS].size;
	
	item_index_table[ID_ITEM_ATTITUDE].magic = ITEM_ID_TO_MAGIC(ID_ITEM_ATTITUDE);
	item_index_table[ID_ITEM_ATTITUDE].size = sizeof(item_attitude_t) 	\
		+ sizeof(data_attitude_t) * pattr->num_raw_attitude,
	item_index_table[ID_ITEM_ATTITUDE].offset = offset;
	offset += item_index_table[ID_ITEM_ATTITUDE].size;


    /**
     *  @brief calculate packet header, All items and data, End flag total size.
     */
    for (i = 0; i < item_count; i++) {
        size += item_index_table[i].size;
    }
	size += sizeof(packet_header_t);
    size += sizeof(u32) * 4;

    return size;
}

BOOL packet::init_packet(
    const packet_attribute_t*   pattr,
    packet_header_t*            pheader,
    u32                         packet_size
    )
{
    u32 size = 0;
    u32 offset = 0;
    u32 item_count = 0;

    if (pattr == NULL)
        return FALSE;

    size = packet::calculate_total_size(pattr);
    if (size != packet_size)
        return FALSE;
    item_count = packet::calculate_item_count(pattr);
    if (item_count == 0 || item_count > PACKET_DEF_ITEM_COUNT)
        return FALSE;

	memset(pheader, 0, sizeof(packet_header_t));

    pheader->magic        = PACKET_SOP_MAGIC;
    pheader->revision     = pattr->revision;
    pheader->size         = size;
    pheader->checksum     = 0;
    pheader->packettag    = 0;
    pheader->configkey    = 0;
    pheader->timestamp    = 0;
    pheader->item_count   = item_count;

    /**
     *  @brief init item_index_table
     */
    packet::init_item_index_table(pattr, pheader->index_table, item_count);

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

    item_index_t *pindex = pheader->index_table;
    offset = pindex[item_count - 1].offset + pindex[item_count - 1].size;
    ((u32*)((u32)pheader + offset))[0] = 0xBFBFBFBF;
    ((u32*)((u32)pheader + offset))[1] = 0xBFBFBFBF;
    ((u32*)((u32)pheader + offset))[2] = 0xBFBFBFBF;
    ((u32*)((u32)pheader + offset))[3] = PACKET_EOP_MAGIC;

    offset += sizeof(u32) * 4;
    ASSERT(offset == packet_size);

    pheader->checksum = 0x00000000u - packet::calculate_checksum(pheader);

    return TRUE;
}



u32 packet::calculate_checksum(const packet_header_t* pheader)
{
	u32 checksum = 0;

	for (u32 i = 0; i < sizeof(*pheader) / sizeof(u32); i++) {
		checksum += ((u32 *)pheader)[i];
	}

	return checksum;
}

u32 packet::calculate_item_count(const packet_attribute_t* pattr)
{
	u32 i = 0;
	u32 item_count = 0;
	if (pattr == NULL) return 0;

	for (i = 0; i < 32; i++) {
		if (pattr->item_mask_enable & (1u << i)) {
			item_count++;
		}
	}

	if (item_count > PACKET_DEF_ITEM_COUNT) {
		ERR("Failed to item_count = %d > %d.\n", item_count, PACKET_DEF_ITEM_COUNT);
		return 0;
	}

	return item_count;
}

u32 packet::calculate_total_size(const packet_attribute_t* pattr)
{
	u32 size = 0;
	item_index_t table[PACKET_DEF_ITEM_COUNT];

	u32 item_count = calculate_item_count(pattr);
	size = packet::init_item_index_table(pattr, table, item_count);

	return size;
}

}
/***********************************************************************
** End of file
***********************************************************************/
