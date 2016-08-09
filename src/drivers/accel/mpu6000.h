/*******************************Copyright (c)***************************
**
** Porject name:	LeaderUAV-Plus
** Created by:		zhuzheng<happyzhull@163.com>
** Created date:	2015/08/28
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
#include "spi.h"

#include "packet.h"
#include "ringbuffer.h"
#include "accel.h"
#include "gyro.h"


#include "os/thread.h"

#define REG_SAMPLE_RATE_DIV     0x19
#define REG_CONFIG              0x1A

#define REG_GYRO_CONFIG         0x1B
#define BITS_SELF_TEST_EN       0xE0
#define GYRO_CONFIG_FSR_SHIFT   3

#define REG_ACCEL_CONFIG        0x1C
#define REG_ACCEL_MOT_THR       0x1F
#define REG_ACCEL_MOT_DUR       0x20
#define ACCL_CONFIG_FSR_SHIFT   3

#define REG_ACCELMOT_THR        0x1F

#define REG_ACCEL_MOT_DUR       0x20

#define REG_FIFO_EN             0x23
#define FIFO_DISABLE_ALL        0x00
#define BIT_ACCEL_OUT           0x08
#define BITS_GYRO_OUT           0x70

#define REG_INT_PIN_CFG         0x37
#define BIT_INT_ACTIVE_LOW      0x80
#define BIT_INT_OPEN_DRAIN      0x40
#define BIT_INT_LATCH_EN        0x20
#define BIT_INT_RD_CLR          0x10
#define BIT_I2C_BYPASS_EN       0x02
#define BIT_INT_CFG_DEFAULT     (BIT_INT_LATCH_EN | BIT_INT_RD_CLR)

#define REG_INT_ENABLE          0x38
#define BIT_DATA_RDY_EN         0x01
#define BIT_DMP_INT_EN          0x02
#define BIT_FIFO_OVERFLOW       0x10
#define BIT_ZMOT_EN             0x20
#define BIT_MOT_EN              0x40
#define BIT_6500_WOM_EN         0x40

#define REG_DMP_INT_STATUS      0x39

#define REG_INT_STATUS          0x3A
#define BIT_DATA_RDY_INT        0x01
#define BIT_DMP_INT_INT         0x02
#define BIT_FIFO_OVERFLOW       0x10
#define BIT_ZMOT_INT            0x20
#define BIT_MOT_INT             0x40
#define BIT_6500_WOM_INT        0x40

#define REG_RAW_ACCEL           0x3B
#define REG_TEMPERATURE         0x41
#define REG_RAW_GYRO            0x43
#define REG_EXT_SENS_DATA_00    0x49

#define BIT_FIFO_RST            0x04
#define BIT_DMP_RST             0x08
#define BIT_I2C_MST_EN          0x20
#define BIT_FIFO_EN             0x40
#define BIT_DMP_EN              0x80
#define BIT_ACCEL_FIFO          0x08
#define BIT_GYRO_FIFO           0x70

#define REG_DETECT_CTRL         0x69
#define MOT_DET_DELAY_SHIFT     4

#define REG_USER_CTRL           0x6A
#define BIT_FIFO_EN             0x40
#define BIT_FIFO_RESET          0x04

#define REG_PWR_MGMT_1          0x6B
#define BIT_H_RESET             0x80
#define BIT_SLEEP               0x40
#define BIT_CYCLE               0x20
#define BIT_CLK_MASK            0x07
#define BIT_RESET_ALL           0xCF
#define BIT_WAKEUP_AFTER_RESET  0x00

#define REG_PWR_MGMT_2          0x6C
#define BIT_PWR_ACCEL_STBY_MASK 0x38
#define BIT_PWR_GYRO_STBY_MASK  0x07
#define BIT_LPA_FREQ_MASK       0xC0
#define BITS_PWR_ALL_AXIS_STBY  (BIT_PWR_ACCEL_STBY_MASK | \
                                    BIT_PWR_GYRO_STBY_MASK)

#define REG_FIFO_COUNT_H        0x72
#define REG_FIFO_R_W            0x74
#define REG_WHOAMI              0x75

#define SAMPLE_DIV_MAX          0xFF
#define ODR_DLPF_DIS            8000
#define ODR_DLPF_ENA            1000

/* Min delay = MSEC_PER_SEC/ODR_DLPF_ENA */
/* Max delay = MSEC_PER_SEC/(ODR_DLPF_ENA/SAMPLE_DIV_MAX+1) */
#define DELAY_MS_MIN_DLPF   1
#define DELAY_MS_MAX_DLPF   256

/* Min delay = MSEC_PER_SEC/ODR_DLPF_DIS and round up to 1*/
/* Max delay = MSEC_PER_SEC/(ODR_DLPF_DIS/SAMPLE_DIV_MAX+1) */
#define DELAY_MS_MIN_NODLPF 1
#define DELAY_MS_MAX_NODLPF 32

/* device bootup time in millisecond */
#define POWER_UP_TIME_MS    100
/* delay to wait gyro engine stable in millisecond */
#define SENSOR_UP_TIME_MS   30
/* delay between power operation in millisecond */
#define POWER_EN_DELAY_US   10

#define MPU6050_LPA_5HZ     0x40

/* initial configure */
#define INIT_FIFO_RATE      200

#define DEFAULT_MOT_THR		1
#define DEFAULT_MOT_DET_DUR	1
#define DEFAULT_MOT_DET_DELAY	0

/* chip reset wait */
#define MPU6050_RESET_RETRY_CNT	10
#define MPU6050_RESET_WAIT_MS	20

/* FIFO related constant */
#define MPU6050_FIFO_SIZE_BYTE	1024
#define	MPU6050_FIFO_CNT_SIZE	2

enum mpu_device_id {
	MPU6050_ID = 0x68,
	MPU6500_ID = 0x70,
};

enum mpu_gyro_fsr {
	GYRO_FSR_250DPS = 0,
	GYRO_FSR_500DPS,
	GYRO_FSR_1000DPS,
	GYRO_FSR_2000DPS,
	NUM_GYRO_FSR
};

enum mpu_accl_fsr {
	ACCEL_FSR_2G = 0,
	ACCEL_FSR_4G,
	ACCEL_FSR_8G,
	ACCEL_FSR_16G,
	NUM_ACCL_FSR
};


enum mpu_filter {
	MPU_DLPF_256HZ_NOLPF2 = 0,
	MPU_DLPF_188HZ,
	MPU_DLPF_98HZ,
	MPU_DLPF_42HZ,
	MPU_DLPF_20HZ,
	MPU_DLPF_10HZ,
	MPU_DLPF_5HZ,
	MPU_DLPF_RESERVED,
	NUM_FILTER
};

enum mpu_clock_source {
	MPU_CLK_INTERNAL = 0,
	MPU_CLK_PLL_X,
	NUM_CLK
};



/* Sensitivity Scale Factor */
/* Sensor HAL will take 1024 LSB/g */
enum mpu_accel_fs_shift {
	ACCEL_SCALE_SHIFT_02G = 0,
	ACCEL_SCALE_SHIFT_04G = 1,
	ACCEL_SCALE_SHIFT_08G = 2,
	ACCEL_SCALE_SHIFT_16G = 3
};

enum mpu_gyro_fs_shift {
	GYRO_SCALE_SHIFT_FS0 = 3,
	GYRO_SCALE_SHIFT_FS1 = 2,
	GYRO_SCALE_SHIFT_FS2 = 1,
	GYRO_SCALE_SHIFT_FS3 = 0
};

/*device enum */
enum inv_devices {
	INV_MPU6050,
	INV_MPU6500,
	INV_MPU6XXX,
	INV_NUM_PARTS
};

/**
 *  struct mpu_reg_map_s - Notable slave registers.
 *  @sample_rate_div:	Divider applied to gyro output rate.
 *  @lpf:		Configures internal LPF.
 *  @fifo_en:	Determines which data will appear in FIFO.
 *  @gyro_config:	gyro config register.
 *  @accel_config:	accel config register
 *  @mot_thr:	Motion detection threshold.
 *  @fifo_count_h:	Upper byte of FIFO count.
 *  @fifo_r_w:	FIFO register.
 *  @raw_gyro:	Address of first gyro register.
 *  @raw_accl:	Address of first accel register.
 *  @temperature:	temperature register.
 *  @int_pin_cfg:	Interrupt pin and I2C bypass configuration.
 *  @int_enable:	Interrupt enable register.
 *  @int_status:	Interrupt flags.
 *  @user_ctrl:	User control.
 *  @pwr_mgmt_1:	Controls chip's power state and clock source.
 *  @pwr_mgmt_2:	Controls power state of individual sensors.
 */
 #if 0
 struct mpu_reg_map {
	u8 who_am_i;
	u8 sample_rate_div;	// Divider applied to gyro output rate.
	u8 lpf;				// Configures internal LPF.
	u8 prod_id;			//
	u8 user_ctrl;			// User control.
	u8 fifo_en;			// Determines which data will appear in FIFO.
	u8 gyro_cfg;			// gyro config register.
	u8 accel_cfg;			// accel config register
	u8 accel_cfg2;		//
	u8 lp_accel_odr;		//
	u8 motion_thr;		// Motion detection threshold.
	u8 motion_dur;		//
	u8 fifo_count_h;		// Upper byte of FIFO count.
	u8 fifo_r_w;			// FIFO register.
	u8 raw_gyro;			// Address of first gyro register.
	u8 raw_accel;			// Address of first accel register.
	u8 temperature;		// temperature register.
	u8 int_pin_cfg;		// Interrupt pin and I2C bypass configuration.
	u8 int_enable;		// Interrupt enable register.
	u8 int_status;		// Interrupt flags.
	u8 dmp_int_status;	//
	u8 accel_intel;		//
	u8 pwr_mgmt_1;		// Controls chip's power state and clock source.
	u8 pwr_mgmt_2;		// Controls power state of individual sensors.
	u8 mem_r_w;			//
	u8 accel_offs;		//
	u8 i2c_mst;			//
	u8 bank_sel;			//
	u8 mem_start_addr;	//
	u8 prgm_start_h;		//
};
#endif
struct mpu_reg_map {
	u8 sample_rate_div;
	u8 lpf;
	u8 fifo_en;
	u8 gyro_config;
	u8 accel_config;
	u8 fifo_count_h;
	u8 mot_thr;
	u8 mot_dur;
	u8 fifo_r_w;
	u8 raw_gyro;
	u8 raw_accel;
	u8 temperature;
	u8 int_pin_cfg;
	u8 int_enable;
	u8 int_status;
	u8 user_ctrl;
	u8 pwr_mgmt_1;
	u8 pwr_mgmt_2;
};

/**
 *  struct mpu_chip_config - Cached chip configuration data.
 *  @fsr:		Full scale range.
 *  @lpf:		Digital low pass filter frequency.
 *  @accl_fs:		accel full scale range.
 *  @enable:		master enable to enable output
 *  @accel_enable:		enable accel functionality
 *  @accel_fifo_enable:	enable accel data output
 *  @gyro_enable:		enable gyro functionality
 *  @gyro_fifo_enable:	enable gyro data output
 *  @is_asleep:		1 if chip is powered down.
 *  @lpa_mode:		low power mode.
 *  @tap_on:		tap on/off.
 *  @flick_int_on:		flick interrupt on/off.
 *  @int_enabled:		interrupt is enabled.
 *  @mot_det_on:		motion detection wakeup enabled.
 *  @cfg_fifo_en:		FIFO R/W is enabled in USER_CTRL register.
 *  @int_pin_cfg:		interrupt pin configuration.
 *  @lpa_freq:		frequency of low power accelerometer.
 *  @rate_div:		Sampling rate divider.
 */
struct mpu_chip_config {
	u32 fsr:2;
	u32 lpf:3;
	u32 accel_fs:2;
	u32 enable:1;
	u32 accel_enable:1;
	u32 accel_fifo_enable:1;
	u32 gyro_enable:1;
	u32 gyro_fifo_enable:1;
	u32 is_asleep:1;
	u32 lpa_mode:1;
	u32 tap_on:1;
	u32 flick_int_on:1;
	u32 int_enabled:1;
	u32 mot_det_on:1;
	u32 cfg_fifo_en:1;
	u8 int_pin_cfg;
	u16 lpa_freq;
	u16 rate_div;
};


/**
 *  struct mpu6050_platform_data - device platform dependent data.
 *  @gpio_en:		enable GPIO.
 *  @gpio_int:		interrupt GPIO.
 *  @int_flags:		interrupt pin control flags.
 *  @use_int:		use interrupt mode instead of polling data.
 *  @place:			sensor place number.
 */
struct mpu6050_platform_data {
	int gpio_en;
	int gpio_int;
	u32 int_flags;
	bool use_int;
	u8 place;
};


#define MPUREG_PRODUCT_ID		0x0C
#define MPUREG_SELF_TESTX		0X0D		// 自检寄存器X
#define MPUREG_SELF_TESTY		0X0E		// 自检寄存器Y
#define MPUREG_SELF_TESTZ		0X0F		// 自检寄存器Z
#define MPUREG_SELF_TESTA		0X10		// 自检寄存器A

#define MPUREG_SAMPLE_RATE_DIV	0X19		// 采样频率分频器
#define MPUREG_CONFIG			0X1A		// 配置寄存器

#define MPUREG_GYRO_CONFIG		0X1B		// 陀螺仪配置寄存器
#define MPUREG_ACCEL_CONFIG		0X1C		// 加速度计配置寄存器
#define MPUREG_ACCEL_MOT_THR	0X1F		// 运动检测阀值设置寄存器
#define MPUREG_ACCEL_MOT_DUR	0X20
#define MPUREG_FIFO_ENABLE		0X23		// FIFO使能寄存器

#define MPUREG_INT_PIN_CFG		0X37		// 中断/旁路设置寄存器
#define MPUREG_INT_ENABLE		0X38		// 中断使能寄存器
#define MPUREG_INT_STATUS		0X3A		// 中断状态寄存器

#define MPUREG_RAW_ACCEL		0X3B
#define MPUREG_ACCEL_XOUTH		0X3B		// 加速度值,X轴高8位寄存器
#define MPUREG_ACCEL_XOUTL		0X3C		// 加速度值,X轴低8位寄存器
#define MPUREG_ACCEL_YOUTH		0X3D		// 加速度值,Y轴高8位寄存器
#define MPUREG_ACCEL_YOUTL		0X3E		// 加速度值,Y轴低8位寄存器
#define MPUREG_ACCEL_ZOUTH		0X3F		// 加速度值,Z轴高8位寄存器
#define MPUREG_ACCEL_ZOUTL		0X40		// 加速度值,Z轴低8位寄存器

#define MPUREG_TEMPERATURE		0X41
#define MPUREG_TEMP_OUTH		0X41		// 温度值高八位寄存器
#define MPUREG_TEMP_OUTL		0X42		// 温度值低8位寄存器

#define MPUREG_RAW_GYRO			0X43
#define MPUREG_GYRO_XOUTH		0X43		// 陀螺仪值,X轴高8位寄存器
#define MPUREG_GYRO_XOUTL		0X44		// 陀螺仪值,X轴低8位寄存器
#define MPUREG_GYRO_YOUTH		0X45		// 陀螺仪值,Y轴高8位寄存器
#define MPUREG_GYRO_YOUTL		0X46		// 陀螺仪值,Y轴低8位寄存器
#define MPUREG_GYRO_ZOUTH		0X47		// 陀螺仪值,Z轴高8位寄存器
#define MPUREG_GYRO_ZOUTL		0X48		// 陀螺仪值,Z轴低8位寄存器

#define MPUREG_I2CSLV0_DO		0X63		// IIC从机0数据寄存器
#define MPUREG_I2CSLV1_DO		0X64		// IIC从机1数据寄存器
#define MPUREG_I2CSLV2_DO		0X65		// IIC从机2数据寄存器
#define MPUREG_I2CSLV3_DO		0X66		// IIC从机3数据寄存器

#define MPUREG_I2CMST_DELAY		0X67		// IIC主机延时管理寄存器
#define MPUREG_SIGPATH_RST		0X68		// 信号通道复位寄存器
#define MPUREG_MDETECT_CTRL		0X69		// 运动检测控制寄存器
#define MPUREG_USER_CTRL		0X6A		// 用户控制寄存器
#define MPUREG_PWR_MGMT1		0X6B		// 电源管理寄存器1
#define MPUREG_PWR_MGMT2		0X6C		// 电源管理寄存器2
#define MPUREG_FIFO_COUNT_H		0X72		// FIFO计数寄存器高八位
#define MPUREG_FIFO_COUNT_L		0X73		// FIFO计数寄存器低八位
#define MPUREG_FIFO_RW			0X74		// FIFO读写寄存器
#define MPUREG_DEVICE_ID		0X75		// 器件ID寄存器

#define MPU_I2C_SLAVE_ADDR		0X68		// MPU I2C ADDR
#define MPU_OUT_HZ  			200			// 100Hz [4, 1k]Hz DMP固定为200Hz
#define MPU_DATA_MAX			0x7FFF		// 32767
#define MPU_DATA_MIN			0x8000		// -32768

//q30格式,long转float时的除数.
#define q30  1073741824.0f




using namespace os;

namespace driver {

class mpu6000 : public device, public interrupt, public thread
{
public:
	mpu6000(PCSTR devname, s32 devid);
	~mpu6000(void);

protected:
	u32		_max_fifo;
	u32		_chip_select;
	spi		*_spi;
	gpio    *_gpio_cs;

	ringbuffer	*_accel_reports;
	struct accel_scale	_accel_scale;
	f32			_accel_range_scale;
	f32			_accel_range_m_s2;

	ringbuffer	*_gyro_reports;
	struct gyro_scale	_gyro_scale;
	f32			_gyro_range_scale;
	f32			_gyro_range_rad_s;

	// last temperature reading for print_info()
	f32			_last_temperature;

public:
    s32 init(void);
    s32 reset(void);
    void measure(void);
    s16 s16_from_bytes(u8 bytes[]);

public:
    s32 probe(spi *pspi, gpio *gpio_cs);
    s32 remove(void);

public:
    virtual s32 open(s32 flags);
    s32 read_accel(u8 *buf, u32 size);
    s32 read_gyro(u8 *buf, u32 size);
    virtual s32 close(void);

public:
	s32 reset_fifo(void);
	s32 read_fifo(struct mpu_fifo_packet *data, u32 packet_cnt);
	s32 configure_fifo(u8 sensors);
	s32 set_int_enable(u8 enable);

	s32 set_gyro_fsr(u16 fsr);
	s32 set_accel_fsr(u8 fsr);
	s32 set_lpf(u16 lpf);
	s32 set_sample_rate(u16 rate);

	s32 get_gyro_raw(s16 *gyro);
	s32 get_accel_raw(s16 *accel);
	s32 get_temperature(f32 *temperature);

	u16 orientation_matrix_to_scalar(s8 *mtx);
	u16 row_2_scale(s8 *row);
	s8 	dmp_get_data(f32 *pitch, f32 *roll, f32 *yaw);

private:
	inline s32 read_reg8(u8 reg);
	inline s32 write_reg8(u8 reg, u8 data);
	s32 read_reg(u8 reg, u8 *buf, u8 len);
	s32 write_reg(u8 reg, u8 *buf, u8 len);

public:
	s32 self_test(void);
	s32 chip_self_test(void);

public:
	virtual void run(void *parg);

};


}


/***********************************************************************
** End of file
***********************************************************************/

