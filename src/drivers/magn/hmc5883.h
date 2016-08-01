/*******************************Copyright (c)***************************
**
** Porject name:	leader
** Created by:		zhuzheng<happyzhull@163.com>
** Created date:	2016/07/31
** Modified by:
** Modified date:
** Descriptions:
**
***********************************************************************/
#pragma once
#include "leader_type.h"
#include "leader_misc.h"

#include "core.h"
#include "interrupt.h"
#include "device.h"
#include "dma.h"
#include "i2c.h"

#include "packet.h"

//bit0-bit1 xyz是否使用偏压,默认为0正常配置
//bit2-bit4 数据输出速率, 110为最大75HZ 100为15HZ 最小000 0.75HZ
//bit5-bit5每次采样平均数 11为8次 00为一次
#define ADDR_CONFIG_A 	0x00 	// r/w

//bit7-bit5磁场增益 数据越大,增益越小 默认001
#define ADDR_CONFIG_B 	0x01 	// r/w

//bit0-bit1 模式设置 00为连续测量 01为单一测量
#define ADDR_MODE 		0x02 	// r/w
#define ADDR_DATA_X_MSB 	0x03	// r
#define ADDR_DATA_X_LSB 	0x04	// r
#define ADDR_DATA_Z_MSB 	0x05	// r
#define ADDR_DATA_Z_LSB 	0x06	// r
#define ADDR_DATA_Y_MSB 	0x07	// r
#define ADDR_DATA_Y_LSB 	0x08	// r

//bit1 数据更新时该位自动锁存,等待用户读取,读取到一半的时候防止数据改变
//bit0 数据已经准备好等待读取了,DRDY引脚也能用
#define ADDR_STATUS 		0x09	// r

//三个识别寄存器,用于检测芯片完整性
#define ADDR_ID_A	0x0A	// r
#define ADDR_ID_B	0x0B	// r
#define ADDR_ID_C	0x0C	// r

//三个识别寄存器的默认值
#define ID_A_WHO_AM_I			'H' //0x48
#define ID_B_WHO_AM_I			'4' //0x34
#define ID_C_WHO_AM_I			'3' //0x33


#define HMC5883_SLAVE_ADDRESS 0x3C //写地址,读地址+1


//HMC5883 初始化宏定义
#define HMC_DEFAULT_CONFIGA_VALUE       0x78     //75hz 8倍采样 正常配置
#define HMC_DEFAULT_CONFIGB_VALUE       0x00     //+-0.88GA增益
#define HMC_DEFAULT_MODE_VALUE          0x00     //连续测量模式


namespace driver {

class hmc5883 : public device, public interrupt
{
public:
    hmc5883(PCSTR name, s32 id);
    ~hmc5883(void);

public:
	u8 _slave_addr;
	i2c *_i2c;

public:
    s32 probe(i2c *pi2c, u8 slave_addr);
    s32 remove(void);


public:
	s32 init(void);
	s32 read_raw(void);

	s32 read_reg8(u8 reg);
	s32 write_reg8(u8 reg, u8 data);
	s32 read_reg(u8 reg, u8 *buf, u8 len);
	s32 write_reg(u8 reg, u8 *buf, u8 len);
};
}

