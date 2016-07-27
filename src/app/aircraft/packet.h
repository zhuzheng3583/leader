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
 *  @brief 包校验的错误码
 */
enum packet_error
{
    PACKET_OK           = 0x00000000,   // 正常
    NULL_PACKET         = 0x00000001,   // 空包，或者包并未分配内存

    BAD_CHECK_SUM       = 0x00000004,   // 包头校验和错误，包数据可能已被非预期修改
    INDEX_OUT_OF_BOUND  = 0x00000008,   // 包头索引越界，强制访问可能得到错误数据
    UNSUPPORT_REVISION  = 0x00000010,   // 不支持的包版本
    SIZE_TOO_LARGE      = 0x00000020,   // 包体积过大（默认最大体积256KB）

    BAD_SOP_MAGIC       = 0x00000100,   // 头部MAGIC错误，可能不是数据包
    BAD_EOP_MAGIC       = 0x00000200,   // 尾部MAGIC错误，可能是不完整的数据包

    UNKNOWN_ERROR       = 0x80000000    // 未知错误
};

/**
 *  enum    item_id
 *  @brief  数据类型枚举值，用于快速访问数据块
 *  @note	最大值为(PACKET_DEF_ITEM_COUNT - 1)
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
 *  @brief	accelerate:加速度
 */
typedef struct data_acce
{
	s16 x;
	s16 y;
	s16 z;
} data_acce_t;

/**
 *  @struct data_gyro
 *  @brief	gyroscope:陀螺仪
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
	s16 pressure;
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
 *  @brief	欧拉角/姿态角:刚体绕三个转轴的旋转角度，
 *			且以载体坐标系，而不依赖于外界的参考坐标系
 *  @note	方向余弦、欧拉角、四元数
 */
typedef struct data_attitude
{
	f32 roll;							// roll: 翻滚 绕Y轴旋转的角度
	f32 pitch;							// pitch:俯仰 绕X轴旋转的角度
	f32 yaw;								// yaw:  偏航 绕Z轴旋转的角度
	f32 altitude;                           // altitude 距离地面高度
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
	u32 num;                    // 原始采样点个数
	u32 timestamp;

	data_acce_t *data_acce;
} item_acce_t;

/**
 *  @struct item_gyro
 *  @brief
 */
typedef struct item_gyro
{
	u32 num;                    // 原始采样点个数
	u32 timestamp;

	data_gyro_t *data_gyro;
} item_gyro_t;

/**
 *  @struct item_mpu
 *  @brief
 */
typedef struct item_mpu
{
	u32 num;                    // 原始采样点个数
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
	u32 num;                    // 原始采样点个数
	u32 timestamp;

	data_magn_t *data_magn;
} item_magn_t;

/**
 *  @struct item_baro
 *  @brief
 */
typedef struct item_baro
{
	u32 num;                    // 原始采样点个数
	u32 timestamp;

	data_baro_t *data_baro;
} item_baro_t;

/**
 *  @struct item_gps
 *  @brief
 */
typedef struct item_gps
{
	u32 num;                    // 原始采样点个数
	u32 timestamp;

	data_gps_t *data_gps;
} item_gps_t;

/**
 *  @struct item_attitude
 *  @brief
 */
typedef struct item_attitude
{
	u32 num;                    // 原始采样点个数
	u32 timestamp;

	data_attitude_t *data_attitude;
} item_attitude_t;

typedef struct item_rc
{
	u32 num;                    // 原始采样点个数
	u32 timestamp;

	data_rc_t *data_rc;
} item_rc_t;



/**
 *  @struct item_index
 *  @brief  数据块索引，即数据包内各个数据项（xxx_item）的索引
 */
typedef struct item_index
{
    u32 magic;                        		// 数据块的magic
    u32 size;                          		// 数据块的大小，data_size为0表示数据包中不包含此项数据
    u32 offset;                        		// 数据块在数据包中的偏移量（包括包头在内的总偏移量）
	u32 reserved;
} item_index_t;

/**
 *  @struct packet_attribute
 *  @brief
 */
typedef struct packet_attribute
{
    u32 revision;            				// 包版本 PACKET_REVISION
    u32 num_acce;            			// 加速度原始采样点个数
    u32 num_gyro;
    u32 num_mpu;
    u32 num_magn;
    u32 num_baro;
    u32 num_gps;
    u32 num_attitude;
    u32 num_rc;

    u32 reserved[6];                   		// 保留字段，用于扩展
} packet_attribute_t;


/**
 *  @struct packet_header
 *  @brief 数据包的包头
 */
typedef struct packet_header
{
	u32 magic;                              // 数据包头部magic(SOP)，固定值为 PACKET_SOP_MAGIC
	u32 revision;                           // 数据包版本，当前版本 PACKET_REVISION
	u32 size;                               // 包含全部包数据在内的完整数据包大小，单位：字节
	u32 checksum;                           // 数据包头部的校验和，保证整个包头32位对齐求和等于零

	u32 packettag;                          // 包标记，每一个新的数据包生成时序号加1，用于卡间数据同步
	u32 configkey;                          // 配置码，每一次重新配置时配置序号加1，用于卡内配置同步
	u32 timestamp;                          // 包时间戳，保留字段

	u32 item_count;                         // 索引项总个数（根据索引项的magic确定是否有效）
	u32 reserved[4];                        // 保留字段，用于扩展

	item_index_t item_index_table[ITEM_COUNT];
    item_index_t reserved_index[16 - ITEM_COUNT];
                                            // 保留数据索引，用于扩展
} packet_header_t;


typedef struct packet_tail
{
    u32 flag[4];
} packet_tail_t;


/**
 *  @def   PACKET_SOP_MAGIC, PACKET_EOP_MAGIC
 *  @brief 数据包的包头和包尾magic，用于轻量级的包完整性校验
 */
#define PACKET_SOP_MAGIC              	0xAAAAAAAA                  // 数据包的包头magic
#define PACKET_EOP_MAGIC              	0x55555555                  // 数据包的包尾magic
#define PACKET_REVISION               	0x01000001                  // 数据包的版本号

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
    packet_header_t* _pheader;            // 实际数据包存储区
    packet_attribute_t _attr;

public:
    /**
     * 创建实际数据包（从堆分配包的存储空间）
     * @param[in]  pattr 数据包的属性结构
     * @return     操作成功返回TRUE
     */
    BOOL create(const packet_attribute_t* pattr);

    /**
     * 将当前数据包对象attach到实际数据包存储空间，attach之后可以通过此对象操作实际数据包内容
     * @param[in]  pmemory 	实际PA数据包存储空间基地址
     * @param[in]  pattr 	数据包的属性结构
     * @return     操作成功返回TRUE
     */
    BOOL attach(void *pmemory, const packet_attribute_t* pattr);

    /**
     * 将当前数据包对象与实际PA数据包存储空间Detach
     * @return    无返回
     */
    void detach(void);

    /**
     * 将当前数据包对象销毁，释放动态分配的存储空间
     * @return    无返回
     */
    void destroy(void);

    /**
     * 数据包校验
     * @return  错误码（enum packet_error）
     */
    u32 verify(void) const;

    /**
     * 更新数据包内部的各个动态指针，用于数据包复制时的指针纠正
     * @return     无返回
     */
    BOOL update(void);

    /**
     * 根据UAV数据包的当前包头内容计算包头的新校验和
     * @param[in]  pheader 数据包的当前包头
     * @return     包头的新校验和
     * @note       此方法并不更新该包头，若需更新包头请手动为包头的checksum字段赋值
     */
    static u32 calc_checksum(const packet_header_t* ppacket);

    /**
     * 根据UAV数据包的当前包头内容计算包头的新校验和
     * @param[in]  pattr 数据包的当前包头
     * @return     包头的新校验和
     */
	u32 calc_item_count(const packet_attribute_t* pattr);

    /**
     * 根据UAV数据包的属性结构计算包的实际总大小，可以根据此值分配包内存并Attach
     * @param[in]  pattr 数据包的属性结构
     * @return     包的实际总大小
     */
    u32 calc_total_size(const packet_attribute_t* pattr);


    /**
     *  @brief 以下API必须在create/attach之后调用
     */

    packet_attribute_t *get_attribute(void);

    /**
     * 获取当前数据包总大小，单位：字节
     * @return    当前数据包总大小
     */
    u32 get_total_size(void);

    /**
     * 获取当前数据包中的有效数据项个数
     * @return    当前数据包中的有效数据项个数
     */
    u32 get_item_count(void);

    /**
     * 获取当前数据包的同步相关字段：
     * packettag，configkey，timestamp
     *
     * @return    同步字段的值
     */
    u32 get_packet_tag(void);

    /**
     * 获取当前数据包的同步相关字段：
     * packettag，configkey，timestamp
     *
     * @return    同步字段的值
     */
    u32 get_config_key(void);

    /**
     * 获取当前数据包的同步相关字段：
     * packettag，configkey，timestamp
     *
     * @return    同步字段的值
     */
    u32 get_timestamp(void);

    /**
     *  获取包数据的基地址，用于进行只读访问
     *
     *  @return 包数据的基地址
     */
    const packet_header_t* get_packet_data(void) const;

    /**
     * 获取数据项的地址
     * @param[in] 数据项item_id
     * @return    数据项的地址，若该数据项不存在则返回NULL
     */
    void *get_item_data(u32 item_id) const;

    /**
     * 获取数据项的大小
     * @param[in] 数据项item_id
     * @return    数据项的大小，若该数据项不存在则返回0
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

