/**
 ******************************************************************************
 * @author: GaoKong
 * @date:   13/08/2016
 ******************************************************************************
**/
#include <malloc.h>
#include <stdlib.h>
#include <string.h>

#include <forward_list>
#include <functional>
#include <iterator>
#include <vector>
#include <deque>
#include <queue>
#include <array>
#include <map>

/* kernel include */
#include "ak.h"
#include "message.h"
#include "timer.h"
#include "fsm.h"

/* driver include */
#include "led.h"
#include "button.h"
#include "flash.h"

/* app include */
#include "app.h"
#include "app_dbg.h"
#include "app_bsp.h"
#include "app_flash.h"
#include "app_non_clear_ram.h"

#include "task_list.h"
#include "task_shell.h"
#include "task_life.h"
#include "task_if.h"
#include "task_rf24_if.h"
#include "task_uart_if.h"
#include "task_display.h"
#include "task_zigbee.h"
#include "task_icm.h"

/* sys include */
#include "sys_boot.h"
#include "sys_irq.h"
#include "sys_io.h"
#include "sys_ctrl.h"
#include "sys_dbg.h"

/* arduino include */
#include "SPI.h"
#include "WString.h"
#include "HardwareSerial.h"
#include "ArduinoJson.h"

/* common include */
#include "screen_manager.h"

/* ----------------------- Platform includes --------------------------------*/

/* ----------------------- Modbus includes ----------------------------------*/
#if defined (TASK_MBMASTER_EN)
#include "mbport.h"
#include "mbm.h"
#include "mbtypes.h"
#include "common/mbportlayer.h"
#endif

#include "buzzer.h"
#include "mic.h"

#include "Wire.h"
#include "ICM_20948.h"
#include "random_forest_impact_detector.h"
#include "accel_ring_buffer.h"

static accel_ring_buffer_t accel_rb;
static volatile bool accel_data_ready = false;
ICM_20948_I2C myICMi2c;
static volatile bool isInited = false;

/* ----------------------- Json includes ------------------------------------*/
//#include "json.hpp"

using namespace std;

#define MBM_SERIAL_PORT           ( 2 )
#define MBM_SERIAL_BAUDRATE       ( 9600 )

#if defined(RELEASE)
const char* app_run_mode = "RELEASE";
#else
static const char* app_run_mode = "DEBUG";
#endif

const app_info_t app_info { \
	APP_MAGIC_NUMBER, \
			APP_VER, \
};

mic_pcm_t mic_pcm;
static boot_app_share_data_t boot_app_share_data;

static void app_power_on_reset();
static void app_start_timer();
static void app_init_state_machine();
static void app_task_init();

/*****************************************************************************/
/* app main function.
 */
/*****************************************************************************/
int main_app() {
	APP_PRINT("App run mode: %s, App version: %d.%d.%d.%d\n", app_run_mode, app_info.version[0] \
			, app_info.version[1]	\
			, app_info.version[2]	\
			, app_info.version[3]);

	sys_soft_reboot_counter++;

	/******************************************************************************
	* init active kernel
	*******************************************************************************/
	ENTRY_CRITICAL();
	task_init();
	task_create((task_t*)app_task_table);
	task_polling_create((task_polling_t*)app_task_polling_table);
	EXIT_CRITICAL();

	/******************************************************************************
	* init applications
	*******************************************************************************/
	/*********************
	* hardware configure *
	**********************/
	/* init watch dog timer */
	sys_ctrl_independent_watchdog_init();	/* 32s */
	sys_ctrl_soft_watchdog_init(200);		/* 20s */

	SPI.begin();

	mic_init(&mic_pcm, sys_adc_get_mic);

	/* adc peripheral configure */
	io_cfg_adc1();			/* configure adc for thermistor and CT sensor */

	/* adc configure for ct sensor */
#ifndef MIC_EN
	adc_bat_io_cfg();
#endif
	/* flash io init */
	flash_io_ctrl_init();

	/*********************
	* software configure *
	**********************/
	/* initial boot object */
	sys_boot_init();

	/* life led init */
	led_init(&led_life, led_life_init, led_life_on, led_life_off);

	ring_buffer_char_init(&ring_buffer_console_rev, buffer_console_rev, BUFFER_CONSOLE_REV_SIZE);

	/* button init */
	button_init(&btn_mode,	10,	BUTTON_MODE_ID,	io_button_mode_init,	io_button_mode_read,	btn_mode_callback);
	button_init(&btn_up,	10,	BUTTON_UP_ID,	io_button_up_init,		io_button_up_read,		btn_up_callback);
	button_init(&btn_down,	10,	BUTTON_DOWN_ID,	io_button_down_init,	io_button_down_read,	btn_down_callback);

	button_enable(&btn_mode);
	button_enable(&btn_up);
	button_enable(&btn_down);

	/* siren init */
	// BUZZER_Init();
	BUZZER_PlaySound(BUZZER_SOUND_STARTUP);

	/* get boot share data */
	flash_read(APP_FLASH_INTTERNAL_SHARE_DATA_SECTOR_1, reinterpret_cast<uint8_t*>(&boot_app_share_data), sizeof(boot_app_share_data_t));
	if (boot_app_share_data.is_power_on_reset == SYS_POWER_ON_RESET) {
		app_power_on_reset();
	}

	/* increase start time */
	fatal_log_t app_fatal_log;
	flash_read(APP_FLASH_AK_DBG_FATAL_LOG_SECTOR, reinterpret_cast<uint8_t*>(&app_fatal_log), sizeof(fatal_log_t));
	app_fatal_log.restart_times ++;
	flash_erase_sector(APP_FLASH_AK_DBG_FATAL_LOG_SECTOR);
	flash_write(APP_FLASH_AK_DBG_FATAL_LOG_SECTOR, reinterpret_cast<uint8_t*>(&app_fatal_log), sizeof(fatal_log_t));

#if defined (TASK_MBMASTER_EN)
	/* modbus rtu init*/
	uint8_t flag_init = eMBMSerialInitExt(&xMBMMaster, MB_RTU, MBM_SERIAL_PORT, MBM_SERIAL_BAUDRATE, MB_PAR_NONE, 1);
	APP_PRINT ("Init mbMaster >> ");
	switch (flag_init)
	{
	case MB_ENOERR:
		APP_PRINT ("No error\n");
		break;
	case MB_ENOREG:
		APP_PRINT ("Illegal register address\n");
		break;
	case MB_EINVAL:
		APP_PRINT ("Illegal argument\n");
		break;
	case MB_EPORTERR:
		APP_PRINT ("Porting layer error\n");
		break;
	case MB_ENORES:
		APP_PRINT ("Insufficient resources\n");
		break;
	case MB_EIO:
		APP_PRINT ("I/O error\n");
		break;
	case MB_EILLSTATE:
		APP_PRINT ("Protocol stack in illegal state\n");
		break;
	case MB_EAGAIN:
		APP_PRINT ("Retry I/O operation\n");
		break;
	case MB_ETIMEDOUT:
		APP_PRINT ("Timeout error occurred\n");
		break;
	case MB_EX_ILLEGAL_FUNCTION:
		APP_PRINT ("Illegal function exception\n");
		break;
	case MB_EX_ILLEGAL_DATA_ADDRESS:
		APP_PRINT ("Illegal data address\n");
		break;
	case MB_EX_ILLEGAL_DATA_VALUE:
		APP_PRINT ("Illegal data value\n");
		break;
	case MB_EX_SLAVE_DEVICE_FAILURE:
		APP_PRINT ("Slave device failure\n");
		break;
	case MB_EX_ACKNOWLEDGE:
		APP_PRINT ("Slave acknowledge\n");
		break;
	case MB_EX_SLAVE_BUSY:
		APP_PRINT ("Slave device busy\n");
		break;
	case MB_EX_MEMORY_PARITY_ERROR:
		APP_PRINT ("Memory parity error\n");
		break;
	case MB_EX_GATEWAY_PATH_UNAVAILABLE:
		APP_PRINT ("Gateway path unavailable\n");
		break;
	case MB_EX_GATEWAY_TARGET_FAILED:
		APP_PRINT ("Gateway target device failed to respond\n");
		break;
		
	default:
		break;
	}
#endif

#if defined (TASK_ZIGBEE_EN)
	Serial2.begin();
	Serial2.setTimeout(100);
#endif

	EXIT_CRITICAL();

#if defined (TASK_ZIGBEE_EN)
	APP_PRINT("start_coordinator(0)\n");
	if (zigbee_network.start_coordinator(0) == 0) {
		APP_PRINT("OK\n");
	}
	else {
		APP_PRINT("NG\n");
	}
#endif
	isInited = false;
	// icm90248_init();
	Wire.begin();
	// Wire.setTimeout(10000);
	myICMi2c.begin(Wire, true);
	while (myICMi2c.status != ICM_20948_Stat_Ok){
		myICMi2c.begin(Wire, true);
		APP_DBG("Sensor init: %s\n", myICMi2c.statusString());
		sys_ctrl_delay_ms(100);
	}
	APP_DBG("Sensor init: %s\n", myICMi2c.statusString());

	bool success = true; // Use success to show if the DMP configuration was successful

	Wire.setClock(50000);
  	// Initialize the DMP. initializeDMP is a weak function. You can overwrite it if you want to e.g. to change the sample rate
  	success &= (myICMi2c.initializeDMP() == ICM_20948_Stat_Ok);
	if (!success) {
		APP_DBG("Init DMP failed !\n");
	}
	// DMP sensor options are defined in ICM_20948_DMP.h
	//    INV_ICM20948_SENSOR_ACCELEROMETER               (16-bit accel)
	//    INV_ICM20948_SENSOR_GYROSCOPE                   (16-bit gyro + 32-bit calibrated gyro)
	//    INV_ICM20948_SENSOR_RAW_ACCELEROMETER           (16-bit accel)
	//    INV_ICM20948_SENSOR_RAW_GYROSCOPE               (16-bit gyro + 32-bit calibrated gyro)
	//    INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED (16-bit compass)
	//    INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED      (16-bit gyro)
	//    INV_ICM20948_SENSOR_STEP_DETECTOR               (Pedometer Step Detector)
	//    INV_ICM20948_SENSOR_STEP_COUNTER                (Pedometer Step Detector)
	//    INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR        (32-bit 6-axis quaternion)
	//    INV_ICM20948_SENSOR_ROTATION_VECTOR             (32-bit 9-axis quaternion + heading accuracy)
	//    INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR (32-bit Geomag RV + heading accuracy)
	//    INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD           (32-bit calibrated compass)
	//    INV_ICM20948_SENSOR_GRAVITY                     (32-bit 6-axis quaternion)
	//    INV_ICM20948_SENSOR_LINEAR_ACCELERATION         (16-bit accel + 32-bit 6-axis quaternion)
	//    INV_ICM20948_SENSOR_ORIENTATION                 (32-bit 9-axis quaternion + heading accuracy)

	// Enable the DMP accelerometer
	success &= (myICMi2c.enableDMPSensor(INV_ICM20948_SENSOR_ACCELEROMETER) == ICM_20948_Stat_Ok);
	if (!success) {
		APP_DBG("Enable DMP sensor failed !\n");
	}
	// Configuring DMP to output data at multiple ODRs:
	// DMP is capable of outputting multiple sensor data at different rates to FIFO.
	// Setting value can be calculated as follows:
	// Value = (DMP running rate / ODR ) - 1
	// E.g. For a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.
	success &= (myICMi2c.setDMPODRrate(DMP_ODR_Reg_Accel, 0) == ICM_20948_Stat_Ok); // Set to the maximum
	if (!success) {
		APP_DBG("set baudrate DMP failed !\n");
	}
	// Enable the FIFO
	success &= (myICMi2c.enableFIFO() == ICM_20948_Stat_Ok);
	if (!success) {
		APP_DBG("enable fifo DMP failed !\n");
	}
	// Enable the DMP
	success &= (myICMi2c.enableDMP() == ICM_20948_Stat_Ok);
	if (!success) {
		APP_DBG("enable DMP failed !\n");
	}
	// Reset DMP
	success &= (myICMi2c.resetDMP() == ICM_20948_Stat_Ok);
	if (!success) {
		APP_DBG("reset DMP failed !\n");
	}
	// Reset FIFO
	success &= (myICMi2c.resetFIFO() == ICM_20948_Stat_Ok);
	if (!success) {
		APP_DBG("reset FIFO DMP failed !\n");
	}
	// Check success
	if (success) {
		APP_DBG("DMP enabled!\n");
		// Wire.setClock(400000);
		Wire.setTimeout(500);
		isInited = true;
		accel_ring_buffer_init(&accel_rb);
	}
	else {
		APP_DBG("Enable DMP failed!\n");
	}
	/* start timer for application */
	app_init_state_machine();
	app_start_timer();

	/******************************************************************************
	* app task initial
	*******************************************************************************/
	app_task_init();

	/******************************************************************************
	* run applications
	*******************************************************************************/
#if !defined(IF_LINK_UART_EN)
	sys_ctrl_shell_sw_to_nonblock();
#endif
	return task_run();
}

/*****************************************************************************/
/* polling task can be pushed right here
 * when all ak message queue empty, task_polling_xxx() will be called.
 */
/*****************************************************************************/
void task_polling_zigbee() {
#if defined(TASK_ZIGBEE_EN)
	zigbee_network.update();
#endif
}

void task_polling_console() {
	volatile uint8_t c = 0;

	while (ring_buffer_char_is_empty(&ring_buffer_console_rev) == false) {

		ENTRY_CRITICAL();
		c = ring_buffer_char_get(&ring_buffer_console_rev);
		EXIT_CRITICAL();

#if defined (IF_LINK_UART_EN)
		if (plink_hal_rev_byte(c) == LINK_HAL_IGNORED) {
#endif
			if (shell.index < SHELL_BUFFER_LENGHT - 1) {

				if (c == '\r' || c == '\n') { /* linefeed */

					xputc('\r');
					xputc('\n');

					shell.data[shell.index] = c;
					shell.data[shell.index + 1] = 0;
					task_post_common_msg(AC_TASK_SHELL_ID, AC_SHELL_LOGIN_CMD, (uint8_t*)&shell.data[0], shell.index + 2);

					shell.index = 0;
				}
				else {

					xputc(c);

					if (c == 8 && shell.index) { /* backspace */
						shell.index--;
					}
					else {
						shell.data[shell.index++] = c;
					}
				}
			}
			else {
				LOGIN_PRINT("\nerror: cmd too long, cmd size: %d, try again !\n", SHELL_BUFFER_LENGHT);
				shell.index = 0;
			}
#if defined (IF_LINK_UART_EN)
		}
#endif
	}
}

void task_polling_icm() {
}

void task_polling_ml() {
	if (accel_data_ready) {
		int32_t result = accel_predict_impact(&accel_rb);

		ENTRY_CRITICAL();
		accel_ring_buffer_reset(&accel_rb);
		accel_data_ready = false;
		EXIT_CRITICAL();
	}
}

/*****************************************************************************/
/* app initial function.
 */
/*****************************************************************************/

/* start software timer for application
 * used for app tasks
 */
void app_start_timer() {
	/* start timer to toggle life led */
	timer_set(AC_TASK_LIFE_ID, AC_LIFE_SYSTEM_CHECK, AC_LIFE_TASK_TIMER_LED_LIFE_INTERVAL, TIMER_PERIODIC);
	timer_set(AC_TASK_FW_ID, FW_CHECKING_REQ, FW_UPDATE_REQ_INTERVAL, TIMER_ONE_SHOT);
	timer_set(AC_TASK_DISPLAY_ID, AC_DISPLAY_INITIAL, AC_DISPLAY_INITIAL_INTERVAL, TIMER_ONE_SHOT);
}

/* init state machine for tasks
 * used for app tasks
 */
void app_init_state_machine() {

}

/* send first message to trigger start tasks
 * used for app tasks
 */
void app_task_init() {
	SCREEN_CTOR(&scr_mng_app, scr_startup_handle, &scr_startup);

	task_post_pure_msg(AC_TASK_RF24_IF_ID, AC_RF24_IF_INIT_NETWORK);
	task_post_pure_msg(AC_TASK_UART_IF_ID, AC_UART_IF_INIT);
}

/*****************************************************************************/
/* app common function
 */
/*****************************************************************************/

/* hardware timer interrupt 10ms
 * used for led, button polling
 */

int8_t convert_int16_to_int8(int16_t in) {
    return (int8_t)((int32_t)in * 127 / 32768);
}

void sys_irq_timer_10ms() {
	button_timer_polling(&btn_mode);
	button_timer_polling(&btn_up);
	button_timer_polling(&btn_down);

	if (isInited == true) {
		icm_20948_DMP_data_t data;
		myICMi2c.readDMPdataFromFIFO(&data);

		if ((myICMi2c.status == ICM_20948_Stat_Ok) || (myICMi2c.status == ICM_20948_Stat_FIFOMoreDataAvail))
		{
			if ((data.header & DMP_header_bitmap_Accel) > 0)
			{
				int8_t acc_x = convert_int16_to_int8(data.Raw_Accel.Data.X);
				int8_t acc_y = convert_int16_to_int8(data.Raw_Accel.Data.Y);
				int8_t acc_z = convert_int16_to_int8(data.Raw_Accel.Data.Z);

				// xfprintf((void(*)(int))io_usart3_putc, "%d\n", acc_z);
				accel_ring_buffer_push(&accel_rb, acc_x, acc_y, acc_z);
				if (accel_ring_buffer_full(&accel_rb) && !accel_data_ready){
					accel_data_ready = true;
				}
			}
		}
	}
}

/* init non-clear RAM objects
 */
void app_power_on_reset() {
	sys_soft_reboot_counter = 0;
}

void* app_get_boot_share_data() {
	return static_cast<void*>(&boot_app_share_data);
}
