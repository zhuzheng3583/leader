/*******************************Copyright (c)***************************
**
**  							   SIMULATE
**
**------------------File Info-------------------------------------------
** File name:            	magnetometer.c
** Latest Version:      	V1.0.0
** Latest modified Date:	2015/06/07
** Modified by:
** Descriptions:
**
**----------------------------------------------------------------------
** Created by:           	Zhu Zheng <happyzhull@163.com>
** Created date:         	2015/06/07
** Descriptions:
**
***********************************************************************/
#include "misc_usr.h"
#include "magnetometer.h"
#include "i2c.h"

#define device_to_mag_device(device) 		container_of(device, struct mag_device, dev)


#if 0
#define SlaveAddress 0x3c   //定义器件在IIC总线中的从地址

/*单字节写HMC5833*/
void Single_Write_HMC5883(uchar Address,uchar Dat)

{
    IIC_Start();
    HMC5883_Send_Byte(SlaveAddress);
    HMC5883_Send_Byte(Address);
    HMC5883_Send_Byte(Dat);
    IIC_Stop();

}
/*单字节读HMC5833*/
/*uchar Single_Read_HMC5883(uchar Addr)
{
    uchar Value;
    IIC_Start();
    HMC5883_Send_Byte(SlaveAddress);
    HMC5883_Send_Byte(Addr);
    IIC_Start();
    HMC5883_Send_Byte(SlaveAddress+1);
    Value=HMC5883_Rec_Byte();
    IIC_SendAck(1);
    IIC_Stop();
    return Value;
}*/

/*多字节读HMC5833*/

void Multiple_Read_HMC5883(void)
{
    uchar i;  //连续读出HMC5883内部角度数据，地址范围0x3~0x5
    IIC_Start();
    HMC5883_Send_Byte(SlaveAddress);
    HMC5883_Send_Byte(0x03);//发送存储单元地址，从0x03开始
    IIC_Start();
    HMC5883_Send_Byte(SlaveAddress+1);
    for(i=0;i<6;i++) //连续读取6个地址数据，存储在Rec_Data
    {
        Rec_Data[i]=HMC5883_Rec_Byte();
        if(i==5)
            IIC_SendAck(1); //最后一个数据需要回NOACK
        else
            IIC_SendAck(0); //回应ACK
    }
    IIC_Stop();
    Delay(100);
}

//初始化HMC5883，根据需要请参考pdf进行修改****

void HMC5883_Init(void)
{
     Single_Write_HMC5883(0x02,0x00);
}

/*主函数*/
void main(void)

{
    int X,Y,Z;
    double Angle;     uint Acr;
    LCD_Init();//LCD12232液晶初始化
    Dis_str(0x80,"3 轴数字罗盘");
    HMC5883_Init();//HMC5883初始化
    do
    {
        Multiple_Read_HMC5883();//连续读出数据，存储在Rec_Data[]中
        X=Rec_Data[0]<<8 | Rec_Data[1];//Combine MSB and LSB of X Data output register
        Z=Rec_Data[2]<<8 | Rec_Data[3];//Combine MSB and LSB of Z Data output register
        Y=Rec_Data[4]<<8 | Rec_Data[5];//Combine MSB and LSB of Y Data output register
        Angle= atan2((double)Y,(double)X)*(180/3.14159265)+180;//单位：角度 (0~360)
        Angle*=10;
        Acr=(uint)Angle;
        Send_DATA(0x92,0);
        Send_DATA(Acr%10000/1000+0x30,1);
        Send_DATA(Acr%1000/100+0x30,1);
        Send_DATA(Acr%100/10+0x30,1);
        Send_DATA('.',1);
        Send_DATA(Acr%10+0x30,1);
        Delay(50000);
    }
    while(1);
}

#endif

static struct device *dev = NULL;


S8 mag_i2c_write(U8 reg, U8 len, U8 *buf)
{
	dev->begintransfer(dev, Write, 0);
	dev->write(dev, &reg, sizeof(reg));
	if (dev->write(dev, buf, len) != len) {
		return -1;
	}
	dev->endtransfer(dev, Write, 0);
	return 0;
}

S8 mag_i2c_read(U8 reg, U8 len, U8 *buf)
{
	dev->begintransfer(dev, Write, 0);
	dev->write(dev, &reg, sizeof(reg));
	dev->begintransfer(dev, Read, 0);
	if (dev->read(dev, buf, len) != len) {
		return -1;
	}
	dev->endtransfer(dev, Read, 0);
	return 0;
}


static inline void mag_i2c_write_byte(U8 reg, U8 data)
{
	mag_i2c_write(reg, sizeof(data), &data);
}

static inline U8 mag_i2c_read_byte(U8 reg)
{
	U8 data = 0;
	mag_i2c_read(reg, sizeof(data), &data);
	return data;
}



static S32 mag_open(struct device *this, S32 flags)
{
	INF("%s[%d]: %s.\n", this->name, this->id, __FUNCTION__);
	struct mag_device *mag_dev = device_to_mag_device(this);
	S32 ret = 0;

	dev = &(mag_dev->i2c_dev->dev);
	dev->open(dev, NULL);
	dev->ioctl(dev, CmdI2cSlaveAddr, mag_dev->magslaveaddr);// 0x1e

	U8 a = 0;
	U8 b = 0;
	U8 c = 0;
	mag_i2c_write_byte(HMC_MODE, 0x00);	 //初始化HMC5883
	//mag_i2c_write_byte(HMC_CONFIG_A, 0xC0);
	//a = mag_i2c_read_byte(HMC_CONFIG_A);
	//a = mag_i2c_read_byte(HMC_IDENTE_A);
	//b = mag_i2c_read_byte(HMC_IDENTE_B);
	//c = mag_i2c_read_byte(HMC_IDENTE_C);
	return 0;
}

static S32 mag_read(struct device *this, S8 *buf, U32 count)
{
	struct mag_device *mag_dev = device_to_mag_device(this);
	U32 readcnt = 0;
	S32 timeout = 0;

	if (count % 6 != 0)
	{
		return -1;
	}

	F64 angle;
	U32 acr;
    //连续读出HMC5883内部角度数据，地址范围0x3~0x5
	U8 tmp[12];
	S16 *buf16 = (S16 *)buf;
	for (readcnt = 0; readcnt < count; readcnt += 6)
	{
		buf16 = (S16 *)&buf[readcnt];
		if (mag_i2c_read(0x03, 6, tmp))
			return -1;
		buf16[0] = tmp[0]<<8 | tmp[1];//Combine MSB and LSB of X Data output register
		buf16[1] = tmp[2]<<8 | tmp[3];//Combine MSB and LSB of Z Data output register
		buf16[2] = tmp[4]<<8 | tmp[5];//Combine MSB and LSB of Y Data output register
		angle = atan2((F64)buf16[2],(F64)buf16[0])*(180/3.14159265)+180;//单位：角度 (0~360)
		angle *= 10;
		acr = (U32)angle;
		//INF("%f, %d.\n", angle, acr);
	}
	return readcnt;
}

static S32 mag_write(struct device *this, PCSTR buf, U32 count)
{
	struct mag_device *mag_dev = device_to_mag_device(this);
	INF("%s[%d]: %s.\n", this->name, this->id, __FUNCTION__);

	U32 writencnt = count;
	return writencnt;
}


static S32 mag_close(struct device *this)
{
	INF("%s[%d]: %s.\n", this->name, this->id, __FUNCTION__);

	return 0;
}


static S32 mag_ioctl(struct device *this, IoctlCmd cmd, U32 arg)
{
	struct mag_device *mag_dev = device_to_mag_device(this);

	switch (cmd)
	{
	default:
		ERR("error: invalid cmpass cmd.\n");
		return -1;
	}

	return 0;
}




struct mag_device *mag_dev_constructe(struct mag_platform *plat)
{
	struct device *dev = NULL;
	struct mag_device *mag_dev = NULL;

	mag_dev = (struct mag_device *)malloc(sizeof(struct mag_device));
	if (mag_dev == NULL)
	{
		WIN_DBG("error: %s[%d]: malloc cmpass_device failed.\n", plat->name, plat->id);
		goto fail0;
	}
	memset(mag_dev, 0x00, sizeof(struct mag_device));
	mag_dev->magslaveaddr = plat->magslaveaddr;
	mag_dev->i2c_dev = plat->i2c_dev;

	dev = &(mag_dev->dev);
	set_default_device_method(dev);
	dev->fd 	= -1;
	dev->name  	= plat->name;
	dev->id  	= plat->id;

	dev->open  = mag_open;
	dev->read  = mag_read;
	dev->write = mag_write;
	dev->close = mag_close;
	dev->ioctl = mag_ioctl;
	dev->isrhandler = NULL;
	register_device(dev);

	INF("%s[%d] is constructed.\n", dev->name, dev->id);

	return mag_dev;

fail0:
	return NULL;
}


S32 mag_dev_destroy(struct mag_device *mag_dev)
{
	if (mag_dev == NULL)	return -1;

	struct device *dev = &(mag_dev->dev);
	INF("%s[%d] is destroyed.\n", dev->name, dev->id);
	unregister_device(dev);

	free(mag_dev);

	return 0;

}


/***********************************************************************
** End of file
***********************************************************************/


