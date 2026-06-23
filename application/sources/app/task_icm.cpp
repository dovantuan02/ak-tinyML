#include "app_dbg.h"
#include "io_cfg.h"
#include "task_icm.h"
#include "sys_ctrl.h"

#define delay_ms(x) sys_ctrl_delay_ms(x)

ICM_20948_Device_t myICM;

static ICM_20948_Status_e write_i2c(uint8_t reg, uint8_t *data, uint32_t len, void *user);
static ICM_20948_Status_e read_i2c(uint8_t reg, uint8_t *buff, uint32_t len, void *user);

const ICM_20948_Serif_t mySerif = {
    write_i2c, // write
    read_i2c,  // read
    NULL,
};

ICM_20948_Status_e write_i2c(uint8_t reg, uint8_t *data, uint32_t len, void *user) {
	int ret = 0;
	if ((ret = io_i2c1_write(ICM_20948_I2C_ADDR_AD1, reg, data, len)) < 0) {
		APP_DBG("write i2c failed, ret=%d !!\n", ret);
		return ICM_20948_Stat_Err;
	}
	return ICM_20948_Stat_Ok;
}

ICM_20948_Status_e read_i2c(uint8_t reg, uint8_t *buff, uint32_t len, void *user) {
	int ret = 0;
	if ((ret = io_i2c1_write(ICM_20948_I2C_ADDR_AD1, reg, NULL, 0)) < 0) {
		APP_DBG("read i2c: reg write failed, ret=%d !!\n", ret);
		return ICM_20948_Stat_Err;
	}
	if ((ret = io_i2c1_read_raw(ICM_20948_I2C_ADDR_AD1, buff, len)) < 0) {
		APP_DBG("read i2c: data read failed, ret=%d !!\n", ret);
		return ICM_20948_Stat_Err;
	}
	return ICM_20948_Stat_Ok;
}

void icm90248_init() {
	ICM_20948_init_struct(&myICM);
	ICM_20948_link_serif(&myICM, &mySerif);

	while (ICM_20948_check_id(&myICM) != ICM_20948_Stat_Ok) {
		APP_DBG("whoami does not match. Halting...");
		delay_ms(1000);
	}

	ICM_20948_Status_e stat = ICM_20948_Stat_Err;
	uint8_t whoami = 0x00;
	stat = ICM_20948_get_who_am_i(&myICM, &whoami);
	while ((stat != ICM_20948_Stat_Ok) || (whoami != ICM_20948_WHOAMI)) {
		stat = ICM_20948_get_who_am_i(&myICM, &whoami);
		APP_DBG("whoami does not match (0x%08x). Halting...\n", whoami);
		delay_ms(1000);
	}

	// Here we are doing a SW reset to make sure the device starts in a known state
	ICM_20948_sw_reset(&myICM);
	delay_ms(250);
	// Set Gyro and Accelerometer to a particular sample mode
	ICM_20948_set_sample_mode(&myICM, (ICM_20948_InternalSensorID_bm)(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous); // optiona: ICM_20948_Sample_Mode_Continuous. ICM_20948_Sample_Mode_Cycled
	// Set full scale ranges for both acc and gyr
	ICM_20948_fss_t myfss;
	myfss.a = gpm2;   // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
	myfss.g = dps250; // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
	ICM_20948_set_full_scale(&myICM, (ICM_20948_InternalSensorID_bm)(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myfss);
	// Set up DLPF configuration
	ICM_20948_dlpcfg_t myDLPcfg;
	myDLPcfg.a = acc_d473bw_n499bw;
	myDLPcfg.g = gyr_d361bw4_n376bw5;
	ICM_20948_set_dlpf_cfg(&myICM, (ICM_20948_InternalSensorID_bm)(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myDLPcfg);
	// Choose whether or not to use DLPF
	ICM_20948_enable_dlpf(&myICM, ICM_20948_Internal_Acc, false);
	ICM_20948_enable_dlpf(&myICM, ICM_20948_Internal_Gyr, false);
	// Now wake the sensor up
	ICM_20948_sleep(&myICM, false);
	ICM_20948_low_power(&myICM, false);
}


