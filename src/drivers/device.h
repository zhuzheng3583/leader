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

#include "cmsis_os.h"

#define DEVICE_ERR(fmt, ...) ERR(("%s: " fmt), _devname, ##__VA_ARGS__)
#define DEVICE_WRN(fmt, ...) WRN(("%s: " fmt), _devname, ##__VA_ARGS__)
#define DEVICE_INF(fmt, ...) INF(("%s: " fmt), _devname, ##__VA_ARGS__)
#define DEVICE_DBG(fmt, ...) DBG(("%s: " fmt), _devname, ##__VA_ARGS__)

struct file {
	struct file *next;
	struct file *parent;
	char *name;
	int lineno;
	int flags;
};


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

	PCSTR _devname;
	s32 _devid;
	u32 _devhandle;


public:
    s32 probe(void);
    s32 is_probed(void);
    s32 remove(void);

public:
    virtual s32 open(s32 flags);
    virtual s32 read(u8 *buf, u32 size);
    virtual s32 write(u8 *buf, u32 size);
    virtual s32 close(void);

    /* TODO 阻塞非阻塞接口 */
    /* TODO 实现Linux的poll函数，pendio类似于poll */
    //virtual s32 poll(enum dir_rw dir, s32 timeout); // 时间单位由子类决定
    virtual s32 ioctl(enum ioctl_cmd cmd, u32 arg);

    virtual s32 seek(s32 offset, enum seek_mode mode);
    virtual s32 tell(void);

    virtual s32 flush(void);


	virtual int open(struct file *filp);
	virtual int close(struct file *filp);
	virtual ssize_t read(struct file *filp, char *buffer, size_t buflen);
	virtual ssize_t write(struct file *filp, const char *buffer, size_t buflen);
	virtual off_t	seek(struct file *filp, off_t offset, int whence);
	virtual int ioctl(struct file *filp, int cmd, unsigned long arg);
	virtual int poll(struct file *filp, struct pollfd *fds, bool setup);
	bool is_open() { return _open_count > 0; }




public:
    static s32 irq_init(void);
    static s32 request_irq(s32 irq, device *owner);
    static void free_irq(s32 irq);
    static void enable_irq(s32 irq);
    static void disable_irq(s32 irq);
    static void enable_all_irq(void);
    static void disable_all_irq(void);
    virtual void isr(void);

protected:
	virtual pollevent_t poll_state(file_t *filp);
	virtual void	poll_notify(pollevent_t events);
	virtual void	poll_notify_one(px4_pollfd_struct_t *fds, pollevent_t events);
	virtual int	open_first(file_t *filp);
	virtual int	close_last(file_t *filp);
	const char	*get_devname() { return _devname; }

	bool	_pub_blocked;		/**< true if publishing should be blocked */

private:
	static const unsigned _max_pollwaiters = 8;

	const char	*_devname;		/**< device node name */
	bool		_registered;		/**< true if device name was registered */
	unsigned	_open_count;		/**< number of successful opens */

	struct pollfd	*_pollset[_max_pollwaiters];
	int		store_poll_waiter(px4_pollfd_struct_t *fds);
	int		remove_poll_waiter(struct pollfd *fds);
};

}
/***********************************************************************
** End of file
***********************************************************************/
