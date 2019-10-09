/**
 * @file MPU6050.cpp
 * @author Dan Oates (WPI Class of 2020)
 */
#include "MPU6050.h"

/**
 * Static Constants
 */
const float MPU6050::vel_per_cnt = 0.00013315805;
const float MPU6050::acc_per_cnt = 0.00059875488;

/**
 * @brief Constructor for MPU-6050 interface
 * @param i2c Platform-specific I2C interface
 */
MPU6050::MPU6050(I2CDEVICE_I2C_CLASS* i2c)
{
	// I2C interface
	this->i2c = I2CDevice(i2c, i2c_addr, I2CDevice::msb_first);

	// State sata
	this->vel_x = 0.0f;
	this->vel_y = 0.0f;
	this->vel_z = 0.0f;
	this->acc_x = 0.0f;
	this->acc_y = 0.0f;
	this->acc_z = 0.0f;
	this->vel_x_cal = 0.0f;
	this->vel_y_cal = 0.0f;
	this->vel_z_cal = 0.0f;
	this->vel_var = 0.0f;
	this->acc_var = 0.0f;
}	

/**
 * @brief Wakes up MPU6050 chip
 */
void MPU6050::init()
{
	i2c.write_uint8(reg_pwrmgmt_addr, reg_pwrmgmt_wake);
}

/**
 * @brief Calibrates IMU while stationary and estmiates sensor variances
 * @param num_samples Number of velocity samples to average
 */
void MPU6050::calibrate(float num_samples)
{
	// Reset gyro calibration offsets
	vel_x_cal = 0.0f;
	vel_y_cal = 0.0f;
	vel_z_cal = 0.0f;

	// Average 6-axis readings
	float vel_x_avg = 0.0f;
	float vel_y_avg = 0.0f;
	float vel_z_avg = 0.0f;
	float acc_x_avg = 0.0f;
	float acc_y_avg = 0.0f;
	float acc_z_avg = 0.0f;
	for(uint8_t i = 0; i < num_samples; i++)
	{
		update();
		vel_x_avg += vel_x;
		vel_y_avg += vel_y;
		vel_z_avg += vel_z;
		acc_x_avg += acc_x;
		acc_y_avg += acc_y;
		acc_z_avg += acc_z;
	}
	float num_samples_inv = 1.0f / (float)num_samples;
	vel_x_avg *= num_samples_inv;
	vel_y_avg *= num_samples_inv;
	vel_z_avg *= num_samples_inv;
	acc_x_avg *= num_samples_inv;
	acc_y_avg *= num_samples_inv;
	acc_z_avg *= num_samples_inv;

	// Estimate variances
	vel_var = 0.0f;
	acc_var = 0.0f;
	for(uint8_t i = 0; i < num_samples; i++)
	{
		update();
		const float vel_x_diff = vel_x - vel_x_avg;
		const float vel_y_diff = vel_y - vel_y_avg;
		const float vel_z_diff = vel_z - vel_z_avg;
		vel_var += vel_x_diff * vel_x_diff;
		vel_var += vel_y_diff * vel_y_diff;
		vel_var += vel_z_diff * vel_z_diff;
		const float acc_x_diff = acc_x - acc_x_avg;
		const float acc_y_diff = acc_y - acc_y_avg;
		const float acc_z_diff = acc_z - acc_z_avg;
		acc_var += acc_x_diff * acc_x_diff;
		acc_var += acc_y_diff * acc_y_diff;
		acc_var += acc_z_diff * acc_z_diff;
	}
	num_samples_inv /= 3.0f;
	vel_var *= num_samples_inv;
	acc_var *= num_samples_inv;

	// Set gyro calibration offsets
	vel_x_cal = vel_x_avg;
	vel_y_cal = vel_y_avg;
	vel_z_cal = vel_z_avg;
}

/**
 * @brief Sets gyro calibration offsets
 * @param vel_x_cal Gyro x-offset [rad/s]
 * @param vel_y_cal Gyro y-offset [rad/s]
 * @param vel_z_cal Gyro z-offset [rad/s]
 */
void MPU6050::set_gyro_cals(float vel_x_cal, float vel_y_cal, float vel_z_cal)
{
	this->vel_x_cal = vel_x_cal;
	this->vel_y_cal = vel_y_cal;
	this->vel_z_cal = vel_z_cal;
}

/**
 * @brief Returns gyro calibration x-offset [rad/s]
 */
float MPU6050::get_vel_x_cal()
{
	return vel_x_cal;
}

/**
 * @brief Returns gyro calibration y-offset [rad/s]
 */
float MPU6050::get_vel_y_cal()
{
	return vel_y_cal;
}

/**
 * @brief Returns gyro calibration z-offset [rad/s]
 */
float MPU6050::get_vel_z_cal()
{
	return vel_z_cal;
}

/**
 * @brief Returns average x-y-z angular velocity variance [(rad/s)^2]
 * 
 * Note: Must call 'calibrate' function first to compute this.
 */
float MPU6050::get_vel_variance()
{
	return vel_var;
}

/**
 * @brief Returns average x-y-z acceleration variance [(m/s^2)^2]
 * 
 * Note: Must call 'calibrate' function first to compute this.
 */
float MPU6050::get_acc_variance()
{
	return acc_var;
}

/**
 * @brief Reads accelerometer and gyro over I2C
 */
void MPU6050::update()
{
	// Initial sequential data read
	i2c.read_sequence(reg_data_addr, 14);

	// Read accelerometer
	acc_x = i2c.read_int16() * acc_per_cnt;
	acc_y = i2c.read_int16() * acc_per_cnt;
	acc_z = i2c.read_int16() * acc_per_cnt;

	// Discard temperature reading
	i2c.read_uint16();

	// Read and adjust gyroscope
	vel_x = i2c.read_int16() * vel_per_cnt - vel_x_cal;
	vel_y = i2c.read_int16() * vel_per_cnt - vel_y_cal;
	vel_z = i2c.read_int16() * vel_per_cnt - vel_z_cal;
}

/**
 * @brief Returns x angular velocity [rad/s] from most recent update
 */
float MPU6050::get_vel_x()
{
	return vel_x;
}

/**
 * @brief Returns y angular velocity [rad/s] from most recent update
 */
float MPU6050::get_vel_y()
{
	return vel_y;
}

/**
 * @brief Returns z angular velocity [rad/s] from most recent update
 */
float MPU6050::get_vel_z()
{
	return vel_z;
}

/**
 * @brief Returns x acceleration [m/s^2] from most recent update
 */
float MPU6050::get_acc_x()
{
	return acc_x;
}

/**
 * @brief Returns y acceleration [m/s^2] from most recent update
 */
float MPU6050::get_acc_y()
{
	return acc_y;
}

/**
 * @brief Returns z acceleration [m/s^2] from most recent update
 */
float MPU6050::get_acc_z()
{
	return acc_z;
}