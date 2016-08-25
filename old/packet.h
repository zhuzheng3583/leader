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
#pragma once
#include "leader_type.h"
#include "leader_misc.h"

/**
 *  @enum  packet_error
 *  @brief ��У��Ĵ�����
 */
enum packet_error
{
    PACKET_OK           = 0x00000000,   // ����
    NULL_PACKET         = 0x00000001,   // �հ������߰���δ�����ڴ�

    BAD_CHECK_SUM       = 0x00000004,   // ��ͷУ��ʹ��󣬰����ݿ����ѱ���Ԥ���޸�
    INDEX_OUT_OF_BOUND  = 0x00000008,   // ��ͷ����Խ�磬ǿ�Ʒ��ʿ��ܵõ���������
    UNSUPPORT_REVISION  = 0x00000010,   // ��֧�ֵİ��汾
    SIZE_TOO_LARGE      = 0x00000020,   // ���������Ĭ��������256KB��

    BAD_SOP_MAGIC       = 0x00000100,   // ͷ��MAGIC���󣬿��ܲ������ݰ�
    BAD_EOP_MAGIC       = 0x00000200,   // β��MAGIC���󣬿����ǲ����������ݰ�

    UNKNOWN_ERROR       = 0x80000000    // δ֪����
};

/**
 *  enum    item_id
 *  @brief  ��������ö��ֵ�����ڿ��ٷ������ݿ�
 *  @note	���ֵΪ(PACKET_DEF_ITEM_COUNT - 1)
 */
enum item_id
{
	ID_ITEM_ACCE = 0,
	ID_ITEM_GYRO,
	ID_ITEM_MPU,
	ID_ITEM_MAGN,
	ID_ITEM_BARO,
	ID_ITEM_GPS,
	ID_ITEM_ATTITUDE,
	ID_ITEM_RC,
	ITEM_COUNT,
};

/**
 *  @struct data_acce
 *  @brief	accelerate:���ٶ�
 */
typedef struct data_acce
{
	s16 x;
	s16 y;
	s16 z;
} data_acce_t;

/**
 *  @struct data_gyro
 *  @brief	gyroscope:������
 */
typedef struct data_gyro
{
	s16 x;
	s16 y;
	s16 z;
} data_gyro_t;

/**
 *  @struct data_magn
 *  @brief
 */
typedef struct data_magn
{
	s16 x;
	s16 y;
	s16 z;
} data_magn_t;

/**
 *  @struct data_mpu
 *  @brief
 */
typedef struct data_mpu
{
    data_acce_t acce;
    data_gyro_t gyro;
} data_mpu_t;




/**
 *  @struct data_baro
 *  @brief
 */
typedef struct data_baro
{
	f32 altitude;
	s32 pressure;
    s32 temperature;
} data_baro_t;

/**
 *  @struct data_gps
 *  @brief
 */
typedef struct data_gps
{
	u32 speed;
	u32 position;
} data_gps_t;

/**
 *  @struct data_attitude
 *  @brief	ŷ����/��̬��:����������ת�����ת�Ƕȣ�
 *			������������ϵ���������������Ĳο�����ϵ
 *  @note	�������ҡ�ŷ���ǡ���Ԫ��
 */
typedef struct data_attitude
{
	f32 roll;							// roll: ���� ��Y����ת�ĽǶ�
	f32 pitch;							// pitch:���� ��X����ת�ĽǶ�
	f32 yaw;								// yaw:  ƫ�� ��Z����ת�ĽǶ�
	f32 altitude;                           // altitude �������߶�
} data_attitude_t;

typedef struct data_rc{
    s16 throttle;
    s16 roll;
    s16 pitch;
    s16 yaw;
    s16 aux1;
    s16 aux2;
    s16 aux3;
    s16 aux4;
    s16 aux5;
    s16 aux6;
}data_rc_t;


/**
 *  @struct item_acce
 *  @brief
 */
typedef struct item_acce
{
	u32 num;                    // ԭʼ���������
	u32 timestamp;

	data_acce_t *data_acce;
} item_acce_t;

/**
 *  @struct item_gyro
 *  @brief
 */
typedef struct item_gyro
{
	u32 num;                    // ԭʼ���������
	u32 timestamp;

	data_gyro_t *data_gyro;
} item_gyro_t;

/**
 *  @struct item_mpu
 *  @brief
 */
typedef struct item_mpu
{
	u32 num;                    // ԭʼ���������
	u32 timestamp;

	f32 temperature;
	data_mpu_t *data_mpu;
} item_mpu_t;

/**
 *  @struct item_magn
 *  @brief
 */
typedef struct item_magn
{
	u32 num;                    // ԭʼ���������
	u32 timestamp;

	data_magn_t *data_magn;
} item_magn_t;

/**
 *  @struct item_baro
 *  @brief
 */
typedef struct item_baro
{
	u32 num;                    // ԭʼ���������
	u32 timestamp;

	data_baro_t *data_baro;
} item_baro_t;

/**
 *  @struct item_gps
 *  @brief
 */
typedef struct item_gps
{
	u32 num;                    // ԭʼ���������
	u32 timestamp;

	data_gps_t *data_gps;
} item_gps_t;

/**
 *  @struct item_attitude
 *  @brief
 */
typedef struct item_attitude
{
	u32 num;                    // ԭʼ���������
	u32 timestamp;

	data_attitude_t *data_attitude;
} item_attitude_t;

typedef struct item_rc
{
	u32 num;                    // ԭʼ���������
	u32 timestamp;

	data_rc_t *data_rc;
} item_rc_t;



/**
 *  @struct item_index
 *  @brief  ���ݿ������������ݰ��ڸ��������xxx_item��������
 */
typedef struct item_index
{
    u32 magic;                        		// ���ݿ��magic
    u32 size;                          		// ���ݿ�Ĵ�С��data_sizeΪ0��ʾ���ݰ��в�������������
    u32 offset;                        		// ���ݿ������ݰ��е�ƫ������������ͷ���ڵ���ƫ������
	u32 reserved;
} item_index_t;

/**
 *  @struct packet_attribute
 *  @brief
 */
typedef struct packet_attribute
{
    u32 revision;            				// ���汾 PACKET_REVISION
    u32 num_acce;            			// ���ٶ�ԭʼ���������
    u32 num_gyro;
    u32 num_mpu;
    u32 num_magn;
    u32 num_baro;
    u32 num_gps;
    u32 num_attitude;
    u32 num_rc;

    u32 reserved[6];                   		// �����ֶΣ�������չ
} packet_attribute_t;


/**
 *  @struct packet_header
 *  @brief ���ݰ��İ�ͷ
 */
typedef struct packet_header
{
	u32 magic;                              // ���ݰ�ͷ��magic(SOP)���̶�ֵΪ PACKET_SOP_MAGIC
	u32 revision;                           // ���ݰ��汾����ǰ�汾 PACKET_REVISION
	u32 size;                               // ����ȫ�����������ڵ��������ݰ���С����λ���ֽ�
	u32 checksum;                           // ���ݰ�ͷ����У��ͣ���֤������ͷ32λ������͵�����

	u32 packettag;                          // ����ǣ�ÿһ���µ����ݰ�����ʱ��ż�1�����ڿ�������ͬ��
	u32 configkey;                          // �����룬ÿһ����������ʱ������ż�1�����ڿ�������ͬ��
	u32 timestamp;                          // ��ʱ����������ֶ�

	u32 item_count;                         // �������ܸ����������������magicȷ���Ƿ���Ч��
	u32 reserved[4];                        // �����ֶΣ�������չ

	item_index_t item_index_table[ITEM_COUNT];
    item_index_t reserved_index[16 - ITEM_COUNT];
                                            // ��������������������չ
} packet_header_t;


typedef struct packet_tail
{
    u32 flag[4];
} packet_tail_t;


/**
 *  @def   PACKET_SOP_MAGIC, PACKET_EOP_MAGIC
 *  @brief ���ݰ��İ�ͷ�Ͱ�βmagic�������������İ�������У��
 */
#define PACKET_SOP_MAGIC              	0xAAAAAAAA                  // ���ݰ��İ�ͷmagic
#define PACKET_EOP_MAGIC              	0x55555555                  // ���ݰ��İ�βmagic
#define PACKET_REVISION               	0x01000001                  // ���ݰ��İ汾��

#define MAGIC_ITEM					(0xABCD0000 & 0xFFFF0000)
#define ITEM_MAGIC_TO_ID(magic)			((magic) & 0x0000FFFF)
#define ITEM_ID_TO_MAGIC(id)			(MAGIC_ITEM | id)

#define INIT_ITEM_INDEX(_index, _id, _size, _offset)	\
{												\
	(_index).magic = ITEM_ID_TO_MAGIC((_id));		\
	(_index).size = (_size);						\
	(_index).offset = (_offset);					\
	(_offset) += (_index).size;					\
} while(0)

namespace app {

class packet
{
public:
	packet(void);
	~packet(void);

public:
    packet_header_t* _pheader;            // ʵ�����ݰ��洢��
    packet_attribute_t _attr;

public:
    /**
     * ����ʵ�����ݰ����Ӷѷ�����Ĵ洢�ռ䣩
     * @param[in]  pattr ���ݰ������Խṹ
     * @return     �����ɹ�����TRUE
     */
    BOOL create(const packet_attribute_t* pattr);

    /**
     * ����ǰ���ݰ�����attach��ʵ�����ݰ��洢�ռ䣬attach֮�����ͨ���˶������ʵ�����ݰ�����
     * @param[in]  pmemory 	ʵ��PA���ݰ��洢�ռ����ַ
     * @param[in]  pattr 	���ݰ������Խṹ
     * @return     �����ɹ�����TRUE
     */
    BOOL attach(void *pmemory, const packet_attribute_t* pattr);

    /**
     * ����ǰ���ݰ�������ʵ��PA���ݰ��洢�ռ�Detach
     * @return    �޷���
     */
    void detach(void);

    /**
     * ����ǰ���ݰ��������٣��ͷŶ�̬����Ĵ洢�ռ�
     * @return    �޷���
     */
    void destroy(void);

    /**
     * ���ݰ�У��
     * @return  �����루enum packet_error��
     */
    u32 verify(void) const;

    /**
     * �������ݰ��ڲ��ĸ�����ָ̬�룬�������ݰ�����ʱ��ָ�����
     * @return     �޷���
     */
    BOOL update(void);

    /**
     * ����UAV���ݰ��ĵ�ǰ��ͷ���ݼ����ͷ����У���
     * @param[in]  pheader ���ݰ��ĵ�ǰ��ͷ
     * @return     ��ͷ����У���
     * @note       �˷����������¸ð�ͷ��������°�ͷ���ֶ�Ϊ��ͷ��checksum�ֶθ�ֵ
     */
    static u32 calc_checksum(const packet_header_t* ppacket);

    /**
     * ����UAV���ݰ��ĵ�ǰ��ͷ���ݼ����ͷ����У���
     * @param[in]  pattr ���ݰ��ĵ�ǰ��ͷ
     * @return     ��ͷ����У���
     */
	u32 calc_item_count(const packet_attribute_t* pattr);

    /**
     * ����UAV���ݰ������Խṹ�������ʵ���ܴ�С�����Ը��ݴ�ֵ������ڴ沢Attach
     * @param[in]  pattr ���ݰ������Խṹ
     * @return     ����ʵ���ܴ�С
     */
    u32 calc_total_size(const packet_attribute_t* pattr);


    /**
     *  @brief ����API������create/attach֮�����
     */

    packet_attribute_t *get_attribute(void);

    /**
     * ��ȡ��ǰ���ݰ��ܴ�С����λ���ֽ�
     * @return    ��ǰ���ݰ��ܴ�С
     */
    u32 get_total_size(void);

    /**
     * ��ȡ��ǰ���ݰ��е���Ч���������
     * @return    ��ǰ���ݰ��е���Ч���������
     */
    u32 get_item_count(void);

    /**
     * ��ȡ��ǰ���ݰ���ͬ������ֶΣ�
     * packettag��configkey��timestamp
     *
     * @return    ͬ���ֶε�ֵ
     */
    u32 get_packet_tag(void);

    /**
     * ��ȡ��ǰ���ݰ���ͬ������ֶΣ�
     * packettag��configkey��timestamp
     *
     * @return    ͬ���ֶε�ֵ
     */
    u32 get_config_key(void);

    /**
     * ��ȡ��ǰ���ݰ���ͬ������ֶΣ�
     * packettag��configkey��timestamp
     *
     * @return    ͬ���ֶε�ֵ
     */
    u32 get_timestamp(void);

    /**
     *  ��ȡ�����ݵĻ���ַ�����ڽ���ֻ������
     *
     *  @return �����ݵĻ���ַ
     */
    const packet_header_t* get_packet_data(void) const;

    /**
     * ��ȡ������ĵ�ַ
     * @param[in] ������item_id
     * @return    ������ĵ�ַ����������������򷵻�NULL
     */
    void *get_item_data(u32 item_id) const;

    /**
     * ��ȡ������Ĵ�С
     * @param[in] ������item_id
     * @return    ������Ĵ�С����������������򷵻�0
     */
    u32 get_item_size(u32 item_id) const;

protected:
    u32 packet::init_item_index(const packet_attribute_t* pattr, item_index_t pindex[ITEM_COUNT]);

    BOOL packet::init(const packet_attribute_t *pattr, packet_header_t *ppacket);


};

}
/***********************************************************************
** End of file
***********************************************************************/

