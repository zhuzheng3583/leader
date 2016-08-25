/*******************************Copyright (c)***************************
**
** Porject name:	leader
** Created by:	zhuzheng<happyzhull@163.com>
** Created date:	2016/08/22
** Modified by:
** Modified date:
** Descriptions:
**
***********************************************************************/
#include "sbus.h"
#include "core.h"

//一、协议说明：
//串口配置为波特率100kbps，8位数据，偶校验(even)，2位停止位，无流控。
//链接https://mbed.org/users/Digixx/notebook/futaba-s-bus-controlled-by-mbed/说明了S-bus帧格式。
//每帧25个字节，按照如下顺序排列：
//[startbyte] [data1] [data2] .... [data22] [flags][endbyte]
//起始字节startbyte = 11110000b (0xF0)，但实际上用STM32（据说ARM核）收到的是0x0F。
//中间22个字节就是16个通道的数据了，为什么是16个通道？因为22x8=11x16，每个通道用11bit表示，范围是0-2047。不信看波形图：

namespace driver {

sbus::sbus(PCSTR devname, s32 devid) :
	device(devname, devid)
{

}

sbus::~sbus(void)
{

}

s32 sbus::probe(void)
{
#if 0
    _rx = new gpio("sbus_rx[gpio-39]", 38);
    _rx->probe();
    _rx->set_direction_input();
    s8 c1 = 0;
    s8 c2 = 0;
    while (1) {
        c1 = sbus::read_byte();
        c2 = sbus::read_byte();
        core::mdelay(200);
    }
#else
    _tx = new gpio("sbus_tx[gpio-39]", 38);
    _tx->probe();
    _tx->set_direction_output();
    while (1) {
        sbus::write_byte(0xA5);
        core::mdelay(500);
    }
#endif
	return 0;
}


s32 sbus::remove(void)
{
	return 0;
}


s32 sbus::init(void)
{

}

s32 sbus::reset(void)
{

}

void sbus::measure(void)
{

}

void sbus::write_byte(s8 c)
{
    timestamp_t timestamp_end = 0;
    timestamp_t inc = core::convert_us_to_timestamp(DEFAULT_BAUD_DELAY_US) - 12*26;

    //发送启始位
    timestamp_end = core::get_timestamp() + inc;
    _tx->set_value(VLOW);
    while (time_after(timestamp_end, core::get_timestamp()));

    //发送8位数据位
#if 0
    u8 i = 0;
loop:
    timestamp_end = core::get_timestamp() + inc;
    _tx->set_value((c>>i) & 0x01);//先传低位
    if (i < 8) {
        i++;
        while (time_after(timestamp_end, core::get_timestamp()));
        goto loop;
    }
#elif 0
    for (u8 i = 0; i < 8; i++) {
        timestamp_end = core::get_timestamp() + inc;
        _tx->set_value((c>>i) & 0x01);//先传低位
        while (time_after(timestamp_end, core::get_timestamp()));
    }
#else
    timestamp_end = core::get_timestamp() + inc;
    _tx->set_value((c>>0) & 0x01);//第0位
    while (time_after(timestamp_end, core::get_timestamp()));
    timestamp_end = core::get_timestamp() + inc;
    _tx->set_value((c>>1) & 0x01);//第1位
    while (time_after(timestamp_end, core::get_timestamp()));
    timestamp_end = core::get_timestamp() + inc;
    _tx->set_value((c>>2) & 0x01);//第2位
    while (time_after(timestamp_end, core::get_timestamp()));
    timestamp_end = core::get_timestamp() + inc;
    _tx->set_value((c>>3) & 0x01);//第3位
    while (time_after(timestamp_end, core::get_timestamp()));
    timestamp_end = core::get_timestamp() + inc;
    _tx->set_value((c>>4) & 0x01);//第4位
    while (time_after(timestamp_end, core::get_timestamp()));
    timestamp_end = core::get_timestamp() + inc;
    _tx->set_value((c>>5) & 0x01);//第5位
    while (time_after(timestamp_end, core::get_timestamp()));
    timestamp_end = core::get_timestamp() + inc;
    _tx->set_value((c>>6) & 0x01);//第6位
    while (time_after(timestamp_end, core::get_timestamp()));
    timestamp_end = core::get_timestamp() + inc;
    _tx->set_value((c>>7) & 0x01);//第7位
    while (time_after(timestamp_end, core::get_timestamp()));
#endif

    //发送校验位(无)

    //发送结束位
    timestamp_end = core::get_timestamp() + inc;
    _tx->set_value(VHIGH);
    while (time_after(timestamp_end, core::get_timestamp()));
}

s8 sbus::read_byte(void)
{
    u8 c = 0;
    u8 tries = 3;
    timestamp_t timestamp_end = 0;
    timestamp_t inc = core::convert_us_to_timestamp(DEFAULT_BAUD_DELAY_US) - 12*26;

    while (_rx->get_value());      //等待开始位
    timestamp_end = core::get_timestamp() + inc*1.5;
    while (time_after(timestamp_end, core::get_timestamp()));

    timestamp_end = core::get_timestamp() + inc;
    if (_rx->get_value()) { c |= (0x01<<0); } //第0位
    while (time_after(timestamp_end, core::get_timestamp()));
    timestamp_end = core::get_timestamp() + inc;
    if (_rx->get_value()) { c |= (0x01<<1); } //第1位
    while (time_after(timestamp_end, core::get_timestamp()));
    timestamp_end = core::get_timestamp() + inc;
    if (_rx->get_value()) { c |= (0x01<<2); } //第2位
    while (time_after(timestamp_end, core::get_timestamp()));
    timestamp_end = core::get_timestamp() + inc;
    if (_rx->get_value()) { c |= (0x01<<3); } //第3位
    while (time_after(timestamp_end, core::get_timestamp()));
    timestamp_end = core::get_timestamp() + inc;
    if (_rx->get_value()) { c |= (0x01<<4); } //第4位
    while (time_after(timestamp_end, core::get_timestamp()));
    timestamp_end = core::get_timestamp() + inc;
    if (_rx->get_value()) { c |= (0x01<<5); } //第5位
    while (time_after(timestamp_end, core::get_timestamp()));
     timestamp_end = core::get_timestamp() + inc;
    if (_rx->get_value()) { c |= (0x01<<6); } //第6位
    while (time_after(timestamp_end, core::get_timestamp()));
    timestamp_end = core::get_timestamp() + inc;
    if (_rx->get_value()) { c |= (0x01<<7); } //第7位
    while (time_after(timestamp_end, core::get_timestamp()));

    //在指定的时间内搜寻结束位。
    while (--tries != 0) {
        timestamp_end = core::get_timestamp() + inc/3;
        if (_rx->get_value()) { break;  }       //收到结束位便退出
        while (time_after(timestamp_end, core::get_timestamp()));
    }

     return c;
}

}

/***********************************************************************
** End of file
***********************************************************************/
