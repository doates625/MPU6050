/**
 * @file MPU6050.h
 * @brief Class for interfacing with MPU6050 6-axis IMU
 * @author Dan Oates (WPI Class of 2020)
 * 
 * The MPU6050 is a 6-axis accelerometer and gyroscope with a built-in
 * temperature sensor. This library acts as an I2C interface with the device
 * with suppport for both the Arduino and Mbed platforms. This library does not
 * include supports for reading the temperature sensor.
 * 
 * Dependencies:
 * - I2CDevice: https://github.com/doates625/I2CDevice.git
 * - Platform: https://github.com/doates625/Platform.git
 * - Unions: https://github.com/doates625/Unions.git
 * 
 * References:
 * - Datasheet: http://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf
 */
#pragma once
#include <I2CDevice.h>

/**
 * Minimum I2C Buffer Size
 */
#if defined(PLATFORM_MBED) && I2CDEVICE_BUFFER_SIZE < 14
	#warning MPU6050 requires I2CDEVICE_BUFFER_SIZE >= 14. Setting to 14.
	#undef I2CDEVICE_BUFFER_SIZE
	#define I2CDEVICE_BUFFER_SIZE 14
#endif

/**
 * Class Declaration
 */
class MPU6050
{
public:

	// Initialization
	MPU6050(I2CDEVICE_I2C_CLASS* i2c);
	void init();

	// Calibration
	void calibrate(float num_samples = 100);
	void set_gyro_cals(float vel_x_cal, float vel_y_cal, float vel_z_cal);
	float get_vel_x_cal();
	float get_vel_y_cal();
	float get_vel_z_cal();
	float get_vel_variance();
	float get_acc_variance();

	// Reading Retrieval
	void update();
	float get_vel_x();
	float get_vel_y();
	float get_vel_z();
	float get_acc_x();
	float get_acc_y();
	float get_acc_z();

protected:

	// I2C Registers
	static const uint8_t i2c_addr = 0x68;
	static const uint8_t reg_pwrmgmt_addr = 0x6B;
	static const uint8_t reg_pwrmgmt_wake = 0x00;
	static const uint8_t reg_data_addr = 0x3B;

	// Unit Conversions
	static const float vel_per_cnt;
	static const float acc_per_cnt;

	// Class Members
	I2CDevice i2c;
	float vel_x, vel_y, vel_z;
	float acc_x, acc_y, acc_z;
	float vel_x_cal, vel_y_cal, vel_z_cal;
	float vel_var, acc_var;
};