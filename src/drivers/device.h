/*******************************Copyright (c)***************************
** 
** Porject name:	leader
** Created by:		zhuzheng<happyzhull@163.com>
** Created date:	2016/04/05
** Modified by:
** Modified date:
** Descriptions:
**
***********************************************************************/
#pragma once
#include "leader_type.h"
#include "leader_misc.h"

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_conf.h"

namespace driver {

/**
 *  @enum  SeekMode
 *  @brief 文件寻址模式，对应C运行时库的三种寻址模式
 */
enum seek_mode
{
    SEEK_SET_M          = 1,                // SEEK_SET
    SEEK_CUR_M          = 2,                // SEEK_CUR
    SEEK_END_M          = 3,                // SEEK_END
};

enum ioctl_cmd
{
    CMD_UART_BANUD      = 0x0000A010,	// 设置UART波特率，参数：u32 banud(如9600，115200)
								
    CMD_SPI_CLK         = 0x0000A020,	// 设置SPI时钟线频率，参数：u32 spiclk(如5000000)
								
    CMD_I2C_CLK         = 0x0000A030,	// 设置I2C时钟线频率，		参数：u32 i2cclk(如100000)
    CMD_I2C_SlAVE_ADDR  = 0x0000A031,	// 设置I2C丛机设备地址，	参数：u32 slaveaddr(7bit)
								
    CMD_MPU_TEMPERTURE  = 0x0000A052,
    CMD_MPU_FIFO_RATE   = 0x0000A053,
								
    CMD_GPS_UARTBANUD   = 0x0000A060,	// 设置GPS所依赖的设备UART波特率，  参数：u32 banud(如9600，115200)
								
    CMD_GPIO_SETDIR     = 0x0000A070,
    CMD_GPIO_OUT        = 0x0000A071,
    CMD_GPIO_IN         = 0x0000A072,
								
    CMD_LED_ON          = 0x0000A080,
    CMD_LED_OFF         = 0x0000A081,
								
    CMD_TIMER_FUNC      = 0x0000A090,
    CMD_TIMER_FUNC_ARG  = 0x0000A091,
    CMD_TIMER_OUT       = 0x0000A092,
								
    CMD_PWM_DUTY_CYCLE  = 0x0000A0A0,
								
    CMD_WDOG_TIMEOUTMS  = 0x0000A0B0, 
};

enum dir_rw
{
    DIR_READ            = 0x0000B000,
    DIR_WRITE           = 0x0000B001,
};

enum wait_mode
{
    WAIT_FOREVER = ~(0),
    DO_NOT_WAIT = 0,
};

class device
{
public:
    device(void);
    device(PCSTR name, s32 id);
    
    ~device(void);

public:
    PCSTR _name;
    s32 _id;
    s32 _irq;
    u32 _handle;
    s32 _probed;
    s32 _opened;

public:
    s32 probe(void);
    s32 is_probed(void);
    s32 remove(void);

public:
    virtual s32 open(s32 flags);
    virtual s32 read(u8 *buf, u32 count);
    virtual s32 write(u8 *buf, u32 count);
    virtual s32 close(void);
    
    /* TODO 阻塞非阻塞接口 */
    /* TODO 实现Linux的poll函数，pendio类似于poll */
    //virtual s32 poll(enum dir_rw dir, s32 timeout); // 时间单位由子类决定
    virtual s32 ioctl(enum ioctl_cmd cmd, u32 arg);

    virtual s32 seek(s32 offset, enum seek_mode mode);
    virtual s32 tell(void);

    virtual s32 flush(void);
};

#define EPERM   1   /* Operation not permitted */
#define ENOENT  2   /* No such file or directory */
#define ESRCH   3   /* No such process */
#define EINTR   4   /* Interrupted system call */
#define EIO     5   /* I/O error */
#define ENXIO   6   /* No such device or address */
#define E2BIG   7   /* Argument list too long */
#define ENOEXEC 8   /* Exec format error */
#define EBADF   9   /* Bad file number */
#define ECHILD  10  /* No child processes */
#define EAGAIN  11  /* Try again */
#define ENOMEM  12  /* Out of memory */
#define EACCES  13  /* Permission denied */
#define EFAULT  14  /* Bad address */
#define ENOTBLK 15  /* Block device required */
#define EBUSY   16  /* Device or resource busy */
#define EEXIST  17  /* File exists */
#define EXDEV   18  /* Cross-device link */
#define ENODEV  19  /* No such device */
#define ENOTDIR 20  /* Not a directory */
#define EISDIR  21  /* Is a directory */
#define EINVAL  22  /* Invalid argument */
#define ENFILE  23  /* File table overflow */
#define EMFILE  24  /* Too many open files */
#define ENOTTY  25  /* Not a typewriter */
#define ETXTBSY 26  /* Text file busy */
#define EFBIG   27  /* File too large */
#define ENOSPC  28  /* No space left on device */
#define ESPIPE  29  /* Illegal seek */
#define EROFS   30  /* Read-only file system */
#define EMLINK  31  /* Too many links */
#define EPIPE   32  /* Broken pipe */
#define EDOM    33  /* Math argument out of domain of func */
#define ERANGE  34  /* Math result not representable */

struct errormap {
	char *name;
	int val;
};

/* FixMe - reduce to a reasonable size */
static struct errormap errmap[] = {
	{"Operation not permitted", EPERM},
	{"wstat prohibited", EPERM},
	{"No such file or directory", ENOENT},
	{"directory entry not found", ENOENT},
	{"file not found", ENOENT},
	{"Interrupted system call", EINTR},
	{"Input/output error", EIO},
	{"No such device or address", ENXIO},
	{"Argument list too long", E2BIG},
	{"Bad file descriptor", EBADF},
	{"Resource temporarily unavailable", EAGAIN},
	{"Cannot allocate memory", ENOMEM},
	{"Permission denied", EACCES},
	{"Bad address", EFAULT},
	{"Block device required", ENOTBLK},
	{"Device or resource busy", EBUSY},
	{"File exists", EEXIST},
	{"Invalid cross-device link", EXDEV},
	{"No such device", ENODEV},
	{"Not a directory", ENOTDIR},
	{"Is a directory", EISDIR},
	{"Invalid argument", EINVAL},
	{"Too many open files in system", ENFILE},
	{"Too many open files", EMFILE},
	{"Text file busy", ETXTBSY},
	{"File too large", EFBIG},
	{"No space left on device", ENOSPC},
	{"Illegal seek", ESPIPE},
	{"Read-only file system", EROFS},
	{"Too many links", EMLINK},
	{"Broken pipe", EPIPE},
	{"Numerical argument out of domain", EDOM},
	{"Numerical result out of range", ERANGE},
	{NULL, -1}
};

}
/***********************************************************************
** End of file
***********************************************************************/
