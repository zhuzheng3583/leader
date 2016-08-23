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
    _tx = new gpio("sbus_rx[gpio-38]", 38);
    _tx->probe();
    _tx->set_direction_output();

    while (1) {
        sbus::write_byte(0xA5);
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
    //发送启始位
    _tx->set_value(VLOW);
    core::udelay(50);

    //发送8位数据位
    for (u8 i = 0; i < 8; i++) {
        _tx->set_value(c & (0x01>>i));//先传低位
        core::udelay(50);
    }

    //发送校验位(无)

    //发送结束位
    _tx->set_value(VHIGH);
    core::udelay(50);
}

#if 0
//从串口读一个字节
uchar RByte(void)

{

     uchar Output=0;

     uchar i=8;

     uchar temp=RDDYN;

     //发送8位数据位

Delay2cp(RDDYN*1.5);          //此处注意，等过起始位

     while(i--)

     {

         Output >>=1;

         if(RXD) Output   |=0x80;      //先收低位

         Delay2cp(35);              //(96-26)/2，循环共占用26个指令周期

     }

     while(--temp)                     //在指定的时间内搜寻结束位。

     {

         Delay2cp(1);

         if(RXD)break;              //收到结束位便退出

     }

     return Output;

}


void sbus::read_byte(void)
{

}
#endif
}

/***********************************************************************
** End of file
***********************************************************************/
