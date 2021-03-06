/**
 * @file MPU6050.h
 * @brief Class for interfacing with MPU6050 6-axis I2C IMU
 * @author Dan Oates (WPI Class of 2020)
 */
#pragma once
#include <I2CDevice.h>
#include <I2CReading.h>

/**
 * Minimum I2C Buffer Size
 */
#if I2CDEVICE_BUFFER_SIZE < 14
	#error MPU6050 requires I2CDEVICE_BUFFER_SIZE >= 14
#endif

/**
 * Calibration Sample Count
 */
#if !defined(MPU6050_CAL_SAMPLES)
	#warning MPU6050_CAL_SAMPLES not defined. Setting to 100...
	#define MPU6050_CAL_SAMPLES 100
#endif

/**
 * Class Declaration
 */
class MPU6050
{
public:

	// Accelerometer Range
	typedef enum
	{
		acc_2G,		// +/-2G
		acc_4G,		// +/-4G
		acc_8G,		// +/-8G
		acc_16G,	// +/-16G
	}
	acc_range_t;

	// Gyroscope Range
	typedef enum
	{
		gyr_250dps,		// +/-250 deg/s
		gyr_500dps,		// +/-500 deg/s
		gyr_1000dps,	// +/-1000 deg/s
		gyr_2000dps,	// +/-2000 deg/s
	}
	gyr_range_t;

	// Gyro offsets [rad/s]
	float gyr_x_cal;
	float gyr_y_cal;
	float gyr_z_cal;

	// Initialization and Basics
	MPU6050(I2CDevice::i2c_t* i2c, bool addr_sel = false);
	bool init();
	void set_acc_range(acc_range_t range);
	void set_gyr_range(gyr_range_t range);
	void update();

	// Accelerometer
	void update_acc();
	float get_acc_x();
	float get_acc_y();
	float get_acc_z();

	// Thermometer
	void update_tmp();
	float get_tmp_c();

	// Gyroscope
	void update_gyr();
	float get_gyr_x();
	float get_gyr_y();
	float get_gyr_z();

	// Calibration
	void calibrate();
	float get_acc_x_var();
	float get_acc_y_var();
	float get_acc_z_var();
	float get_gyr_x_var();
	float get_gyr_y_var();
	float get_gyr_z_var();

protected:

	// I2C Communication
	uint8_t i2c_addr;
	I2CDevice i2c;

	// Config Registers
	static const uint8_t reg_whoami_addr = 0x75;
	static const uint8_t reg_pwrmgmt_addr = 0x6B;
	static const uint8_t reg_pwrmgmt_wake = 0x00;

	// Gyroscope Config
	static const uint8_t reg_gyr_config_addr = 0x1B;
	static const uint8_t reg_gyr_config_250dps = 0x0 << 3;
	static const uint8_t reg_gyr_config_500dps = 0x1 << 3;
	static const uint8_t reg_gyr_config_1000dps = 0x2 << 3;
	static const uint8_t reg_gyr_config_2000dps = 0x3 << 3;
	float gyr_per_lsb;

	// Accelerometer Config
	static const uint8_t reg_acc_config_addr = 0x1C;
	static const uint8_t reg_acc_config_2G = 0x0 << 3;
	static const uint8_t reg_acc_config_4G = 0x1 << 3;
	static const uint8_t reg_acc_config_8G = 0x2 << 3;
	static const uint8_t reg_acc_config_16G = 0x3 << 3;
	float acc_per_lsb;

	// Accelerometer Reading
	static const uint8_t reg_acc_x_addr = 0x3B;
	static const uint8_t reg_acc_y_addr = 0x3D;
	static const uint8_t reg_acc_z_addr = 0x3F;
	I2CReading<int16_t> acc_x, acc_y, acc_z;

	// Thermometer Reading
	static const float tmp_per_lsb;
	static const float tmp_offset_C;
	static const uint8_t reg_tmp_c_addr = 0x41;
	I2CReading<int16_t> tmp_c;

	// Gyroscope Reading
	static const uint8_t reg_gyr_x_addr = 0x43;
	static const uint8_t reg_gyr_y_addr = 0x45;
	static const uint8_t reg_gyr_z_addr = 0x47;
	I2CReading<int16_t> gyr_x, gyr_y, gyr_z;

	// Calibration
	float acc_x_var, acc_y_var, acc_z_var;
	float gyr_x_var, gyr_y_var, gyr_z_var;
};
