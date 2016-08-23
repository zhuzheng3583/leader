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
    //_tx = new gpio("sbus_tx[gpio-39]", 39);
    //_tx->probe();
    //_tx->set_direction_output();

    _rx = new gpio("sbus_rx[gpio-39]", 39);
    _rx->probe();
    _rx->set_direction_input();
    
    s8 c1 = 0;
    s8 c2 = 0;
    while (1) {
        c1 = sbus::read_byte();
        c2 = sbus::read_byte();
        core::mdelay(200);
    }
    
    while (1) {
        sbus::write_byte(0xA5);
        core::mdelay(500);
    }

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
    timestamp_t timeout = 0;
    timestamp_t inc = DEFAULT_BAUD_DELAY_US * core::s_freq_mhz - 12*26;
    
    //发送启始位
    timeout = core::get_timestamp() + inc;
    _tx->set_value(VLOW);
    while (time_after(timeout, core::get_timestamp()));

    //发送8位数据位
#if 0
    u8 i = 0;
loop:
    timeout = core::get_timestamp() + inc;
    _tx->set_value((c>>i) & 0x01);//先传低位
    if (i < 8) {
        i++;
        while (time_after(timeout, core::get_timestamp()));
        goto loop;
    }
#elif 0
    for (u8 i = 0; i < 8; i++) {
        timeout = core::get_timestamp() + inc;
        _tx->set_value((c>>i) & 0x01);//先传低位
        while (time_after(timeout, core::get_timestamp()));
    }
#else
    timeout = core::get_timestamp() + inc;
    _tx->set_value((c>>0) & 0x01);//第0位
    while (time_after(timeout, core::get_timestamp()));
    timeout = core::get_timestamp() + inc;
    _tx->set_value((c>>1) & 0x01);//第1位
    while (time_after(timeout, core::get_timestamp()));
    timeout = core::get_timestamp() + inc;
    _tx->set_value((c>>2) & 0x01);//第2位
    while (time_after(timeout, core::get_timestamp()));
    timeout = core::get_timestamp() + inc;
    _tx->set_value((c>>3) & 0x01);//第3位
    while (time_after(timeout, core::get_timestamp()));
    timeout = core::get_timestamp() + inc;
    _tx->set_value((c>>4) & 0x01);//第4位
    while (time_after(timeout, core::get_timestamp()));
    timeout = core::get_timestamp() + inc;
    _tx->set_value((c>>5) & 0x01);//第5位
    while (time_after(timeout, core::get_timestamp()));
    timeout = core::get_timestamp() + inc;
    _tx->set_value((c>>6) & 0x01);//第6位
    while (time_after(timeout, core::get_timestamp()));
    timeout = core::get_timestamp() + inc;
    _tx->set_value((c>>7) & 0x01);//第7位
    while (time_after(timeout, core::get_timestamp()));
#endif

    //发送校验位(无)

    //发送结束位
    timeout = core::get_timestamp() + inc;
    _tx->set_value(VHIGH);
    while (time_after(timeout, core::get_timestamp()));
}

s8 sbus::read_byte(void)
{
    u8 c = 0;
    u8 tries = 3;
    timestamp_t timeout = 0;
    timestamp_t inc = DEFAULT_BAUD_DELAY_US * core::s_freq_mhz - 12*26;
   
    while (_rx->get_value());      //等待开始位
    timeout = core::get_timestamp() + inc*1.5;
    while (time_after(timeout, core::get_timestamp()));

    timeout = core::get_timestamp() + inc;
    if (_rx->get_value()) { c |= (0x01<<0); } //第0位
    while (time_after(timeout, core::get_timestamp()));
    timeout = core::get_timestamp() + inc;
    if (_rx->get_value()) { c |= (0x01<<1); } //第1位
    while (time_after(timeout, core::get_timestamp()));
    timeout = core::get_timestamp() + inc;
    if (_rx->get_value()) { c |= (0x01<<2); } //第2位
    while (time_after(timeout, core::get_timestamp()));
    timeout = core::get_timestamp() + inc;
    if (_rx->get_value()) { c |= (0x01<<3); } //第3位
    while (time_after(timeout, core::get_timestamp()));   
    timeout = core::get_timestamp() + inc;
    if (_rx->get_value()) { c |= (0x01<<4); } //第4位
    while (time_after(timeout, core::get_timestamp()));
    timeout = core::get_timestamp() + inc;
    if (_rx->get_value()) { c |= (0x01<<5); } //第5位
    while (time_after(timeout, core::get_timestamp()));
     timeout = core::get_timestamp() + inc;
    if (_rx->get_value()) { c |= (0x01<<6); } //第6位
    while (time_after(timeout, core::get_timestamp()));
    timeout = core::get_timestamp() + inc;
    if (_rx->get_value()) { c |= (0x01<<7); } //第7位
    while (time_after(timeout, core::get_timestamp()));
    
    //在指定的时间内搜寻结束位。
    while (--tries != 0) {
        timeout = core::get_timestamp() + inc/3;
        if (_rx->get_value()) { break;  }       //收到结束位便退出
        while (time_after(timeout, core::get_timestamp()));
    }
    
     return c;
}

}

/***********************************************************************
** End of file
***********************************************************************/
