/*******************************Copyright (c)***************************
**
**  							   SIMULATE
**
**------------------File Info-------------------------------------------
** File name:            	gps.h
** Latest Version:      	V1.0.0
** Latest modified Date:	2015/06/05
** Modified by:
** Descriptions:
**
**----------------------------------------------------------------------
** Created by:           	Zhu Zheng <happyzhull@163.com>
** Created date:         	2015/06/05
** Descriptions:
**
***********************************************************************/
#pragma once
#include "type.h"
#include "misc_usr.h"
#include "device.h"


#if 0
//GPS NMEA-0183协议重要参数结构体定义
//卫星信息
__packed typedef struct
{
 	U8  num;		//卫星编号
	U8  eledeg;	//卫星仰角
	U16 azideg;	//卫星方位角
	U8  sn;		//信噪比
}nmea_slmsg;
//UTC时间信息
__packed typedef struct
{
 	U16 year;	//年份
	U8  month;	//月份
	U8  date;	//日期
	U8  hour; 	//小时
	U8  min; 	//分钟
	U8  sec; 	//秒钟
}nmea_utc_time;
//NMEA 0183 协议解析后数据存放结构体
__packed typedef struct
{
 	U8 svnum;					//可见卫星数
	nmea_slmsg slmsg[12];		//最多12颗卫星
	nmea_utc_time utc;			//UTC时间
	u32 latitude;				//纬度 分扩大100000倍,实际要除以100000
	U8 nshemi;					//北纬/南纬,N:北纬;S:南纬
	u32 longitude;			    //经度 分扩大100000倍,实际要除以100000
	U8 ewhemi;					//东经/西经,E:东经;W:西经
	U8 gpssta;					//GPS状态:0,未定位;1,非差分定位;2,差分定位;6,正在估算.
 	U8 posslnum;				//用于定位的卫星数,0~12.
 	U8 possl[12];				//用于定位的卫星编号
	U8 fixmode;					//定位类型:1,没有定位;2,2D定位;3,3D定位
	U16 pdop;					//位置精度因子 0~500,对应实际值0~50.0
	U16 hdop;					//水平精度因子 0~500,对应实际值0~50.0
	U16 vdop;					//垂直精度因子 0~500,对应实际值0~50.0

	int altitude;			 	//海拔高度,放大了10倍,实际除以10.单位:0.1m
	U16 speed;					//地面速率,放大了1000倍,实际除以10.单位:0.001公里/小时
}nmea_msg;
////////////////////////////////////////////////////////////////////////////////////////////////////
//UBLOX NEO-6M 配置(清除,保存,加载等)结构体
__packed typedef struct
{
 	U16 header;					//cfg header,固定为0X62B5(小端模式)
	U16 id;						//CFG CFG ID:0X0906 (小端模式)
	U16 dlength;				//数据长度 12/13
	u32 clearmask;				//子区域清除掩码(1有效)
	u32 savemask;				//子区域保存掩码
	u32 loadmask;				//子区域加载掩码
	U8  devicemask; 		  	//目标器件选择掩码	b0:BK RAM;b1:FLASH;b2,EEPROM;b4,SPI FLASH
	U8  cka;		 			//校验CK_A
	U8  ckb;			 		//校验CK_B
}_ublox_cfg_cfg;

//UBLOX NEO-6M 消息设置结构体
__packed typedef struct
{
 	U16 header;					//cfg header,固定为0X62B5(小端模式)
	U16 id;						//CFG MSG ID:0X0106 (小端模式)
	U16 dlength;				//数据长度 8
	U8  msgclass;				//消息类型(F0 代表NMEA消息格式)
	U8  msgid;					//消息 ID
								//00,GPGGA;01,GPGLL;02,GPGSA;
								//03,GPGSV;04,GPRMC;05,GPVTG;
								//06,GPGRS;07,GPGST;08,GPZDA;
								//09,GPGBS;0A,GPDTM;0D,GPGNS;
	U8  iicset;					//IIC消输出设置    0,关闭;1,使能.
	U8  uart1set;				//UART1输出设置	   0,关闭;1,使能.
	U8  uart2set;				//UART2输出设置	   0,关闭;1,使能.
	U8  usbset;					//USB输出设置	   0,关闭;1,使能.
	U8  spiset;					//SPI输出设置	   0,关闭;1,使能.
	U8  ncset;					//未知输出设置	   默认为1即可.
 	U8  cka;			 		//校验CK_A
	U8  ckb;			    	//校验CK_B
}_ublox_cfg_msg;

//UBLOX NEO-6M UART端口设置结构体
__packed typedef struct
{
 	U16 header;					//cfg header,固定为0X62B5(小端模式)
	U16 id;						//CFG PRT ID:0X0006 (小端模式)
	U16 dlength;				//数据长度 20
	U8  portid;					//端口号,0=IIC;1=UART1;2=UART2;3=USB;4=SPI;
	U8  reserved;				//保留,设置为0
	U16 txready;				//TX Ready引脚设置,默认为0
	u32 mode;					//串口工作模式设置,奇偶校验,停止位,字节长度等的设置.
 	u32 baudrate;				//波特率设置
 	U16 inprotomask;		 	//输入协议激活屏蔽位  默认设置为0X07 0X00即可.
 	U16 outprotomask;		 	//输出协议激活屏蔽位  默认设置为0X07 0X00即可.
 	U16 reserved4; 				//保留,设置为0
 	U16 reserved5; 				//保留,设置为0
 	U8  cka;			 		//校验CK_A
	U8  ckb;			    	//校验CK_B
}_ublox_cfg_prt;

//UBLOX NEO-6M 时钟脉冲配置结构体
__packed typedef struct
{
 	U16 header;					//cfg header,固定为0X62B5(小端模式)
	U16 id;						//CFG TP ID:0X0706 (小端模式)
	U16 dlength;				//数据长度
	u32 interval;				//时钟脉冲间隔,单位为us
	u32 length;				 	//脉冲宽度,单位为us
	signed char status;			//时钟脉冲配置:1,高电平有效;0,关闭;-1,低电平有效.
	U8 timeref;			   		//参考时间:0,UTC时间;1,GPS时间;2,当地时间.
	U8 flags;					//时间脉冲设置标志
	U8 reserved;				//保留
 	signed short antdelay;	 	//天线延时
 	signed short rfdelay;		//RF延时
	signed int userdelay; 	 	//用户延时
	U8 cka;						//校验CK_A
	U8 ckb;						//校验CK_B
}_ublox_cfg_tp;

//UBLOX NEO-6M 刷新速率配置结构体
__packed typedef struct
{
 	U16 header;					//cfg header,固定为0X62B5(小端模式)
	U16 id;						//CFG RATE ID:0X0806 (小端模式)
	U16 dlength;				//数据长度
	U16 measrate;				//测量时间间隔，单位为ms，最少不能小于200ms（5Hz）
	U16 navrate;				//导航速率（周期），固定为1
	U16 timeref;				//参考时间：0=UTC Time；1=GPS Time；
 	U8  cka;					//校验CK_A
	U8  ckb;					//校验CK_B
}_ublox_cfg_rate;

int NMEA_Str2num(U8 *buf,U8*dx);
void GPS_Analysis(nmea_msg *gpsx,U8 *buf);
void NMEA_GPGSV_Analysis(nmea_msg *gpsx,U8 *buf);
void NMEA_GPGGA_Analysis(nmea_msg *gpsx,U8 *buf);
void NMEA_GPGSA_Analysis(nmea_msg *gpsx,U8 *buf);
void NMEA_GPGSA_Analysis(nmea_msg *gpsx,U8 *buf);
void NMEA_GPRMC_Analysis(nmea_msg *gpsx,U8 *buf);
void NMEA_GPVTG_Analysis(nmea_msg *gpsx,U8 *buf);
U8 Ublox_Cfg_Cfg_Save(void);
U8 Ublox_Cfg_Msg(U8 msgid,U8 uart1set);
U8 Ublox_Cfg_Prt(u32 baudrate);
U8 Ublox_Cfg_Tp(u32 interval,u32 length,signed char status);
U8 Ublox_Cfg_Rate(U16 measrate,U8 reftime);
void Ublox_Send_Date(U8* dbuf,U16 len);
#endif



struct gps_device {
	U32 gps_uart_banud;
	struct uart_device *uart_dev;

	struct device dev;
};

struct gps_platform {
	PCSTR name;
	S32	id;
	U32 gps_uart_banud;
	struct uart_device *uart_dev;
};


struct gps_device *gps_dev_constructe(struct gps_platform *plat);
S32 gps_dev_destroy(struct gps_device *gps_dev);

/***********************************************************************
** End of file
***********************************************************************/


