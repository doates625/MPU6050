/**
 * @file MPU6050.cpp
 * @author Dan Oates (WPI Class of 2020)
 */
#include "MPU6050.h"
#include <CppUtil.h>
using CppUtil::mean;
using CppUtil::var;

/**
 * Static Constants
 */
const float MPU6050::tmp_per_lsb = 1.0f / 340.0f;
const float MPU6050::tmp_offset_C = 36.53f;

/**
 * @brief Constructor for MPU-6050 interface
 * @param i2c Platform-specific I2C interface
 * @param add_sel Address select line (1 = high, 0 = low)
 */
MPU6050::MPU6050(I2CDevice::i2c_t* i2c, bool addr_sel) :
	i2c(i2c, addr_sel ? 0x69 : 0x68, Struct::msb_first),
	acc_x(&(this->i2c), reg_acc_x_addr),
	acc_y(&(this->i2c), reg_acc_y_addr),
	acc_z(&(this->i2c), reg_acc_z_addr),
	tmp_c(&(this->i2c), reg_tmp_c_addr),
	gyr_x(&(this->i2c), reg_gyr_x_addr),
	gyr_y(&(this->i2c), reg_gyr_y_addr),
	gyr_z(&(this->i2c), reg_gyr_z_addr)
{
	// I2C interface
	this->i2c_addr = addr_sel ? 0x69 : 0x68;

	// Calibration
	this->acc_x_var = 0.0f; this->acc_y_var = 0.0f; this->acc_z_var = 0.0f;
	this->gyr_x_var = 0.0f; this->gyr_y_var = 0.0f; this->gyr_z_var = 0.0f;
	this->gyr_x_cal = 0.0f; this->gyr_y_cal = 0.0f; this->gyr_z_cal = 0.0f;
}

/**
 * @brief Initializes MPU6050
 * @return True if I2C communication succeeded
 */
bool MPU6050::init()
{
	// Check I2C address in WHOAMI register
	uint8_t addr = i2c.get_seq(reg_whoami_addr, 1);
	if ((addr >> 1) != (i2c_addr >> 1))
	{
		return false;
	}

	// Configure accelerometer and gyroscope
	set_acc_range(acc_2G);
	set_gyr_range(gyr_250dps);

	// Wake up device (might need to put first...)
	i2c.set(reg_pwrmgmt_addr, reg_pwrmgmt_wake);
	
	// Everything succeeded
	return true;
}

/**
 * @brief Sets accelerometer full-scale range
 * @param range Full-scale range [enum]
 * 
 * Range options:
 * - acc_2G = +/-2 [G]
 * - acc_4G = +/-4 [G]
 * - acc_8G = +/-8 [G]
 * - acc_16G = +/-16 [G]
 */
void MPU6050::set_acc_range(acc_range_t range)
{
	float acc_range_g = 0.0f;
	uint8_t reg_val = 0x00;
	switch (range)
	{
		case acc_2G:
			reg_val = reg_acc_config_2G;
			acc_range_g = 2.0f;
			break;
		case acc_4G:
			reg_val = reg_acc_config_4G;
			acc_range_g = 4.0f;
			break;
		case acc_8G:
			reg_val = reg_acc_config_8G;
			acc_range_g = 8.0f;
			break;
		case acc_16G:
			reg_val = reg_acc_config_16G;
			acc_range_g = 16.0f;
			break;
	}
	i2c.set(reg_acc_config_addr, reg_val);
	acc_per_lsb = acc_range_g * 9.81f / 32768.0f;
}

/**
 * @brief Sets gyroscope full-scale range
 * @param range Full-scale range [enum]
 * 
 * Range options:
 * - gyr_250dps = +/-250 [deg/s]
 * - gyr_500dps = +/-500 [deg/s]
 * - gyr_1000dps = +/-1000 [deg/s]
 * - gyr_2000dps = +/-2000 [deg/s]
 */
void MPU6050::set_gyr_range(gyr_range_t range)
{
	float gyr_range_dps = 0.0f;
	uint8_t reg_val = 0x00;
	switch (range)
	{
		case gyr_250dps:
			reg_val = reg_gyr_config_250dps;
			gyr_range_dps = 250.0f;
			break;
		case gyr_500dps:
			reg_val = reg_gyr_config_500dps;
			gyr_range_dps = 500.0f;
			break;
		case gyr_1000dps:
			reg_val = reg_gyr_config_1000dps;
			gyr_range_dps = 1000.0f;
			break;
		case gyr_2000dps:
			reg_val = reg_gyr_config_2000dps;
			gyr_range_dps = 2000.0f;
			break;
	}
	i2c.set(reg_gyr_config_addr, reg_val);
	gyr_per_lsb = gyr_range_dps * (M_PI / 180.0f) / 32768.0f;
}

/**
 * @brief Updates all IMU readings
 */
void MPU6050::update()
{
	i2c.get_seq(reg_acc_x_addr, 14);
	acc_x.update();
	acc_y.update();
	acc_z.update();
	tmp_c.update();
	gyr_x.update();
	gyr_y.update();
	gyr_z.update();
}

/**
 * @brief Updates accelerometer readings
 */
void MPU6050::update_acc()
{
	i2c.get_seq(reg_acc_x_addr, 6);
	acc_x.update();
	acc_y.update();
	acc_z.update();
}

/**
 * @brief Returns X acceleration [m/s^2]
 */
float MPU6050::get_acc_x()
{
	return acc_x * acc_per_lsb;
}

/**
 * @brief Returns Y acceleration [m/s^2]
 */
float MPU6050::get_acc_y()
{
	return acc_y * acc_per_lsb;
}

/**
 * @brief Returns Z acceleration [m/s^2]
 */
float MPU6050::get_acc_z()
{
	return acc_z * acc_per_lsb;
}

/**
 * @brief Updates thermometer reading
 */
void MPU6050::update_tmp()
{
	i2c.get_seq(reg_tmp_c_addr, 2);
	tmp_c.update();
}

/**
 * @brief Returns temperature [deg C]
 */
float MPU6050::get_tmp_c()
{
	return tmp_c * tmp_per_lsb + tmp_offset_C;
}

/**
 * @brief Updates gyroscope readings
 */
void MPU6050::update_gyr()
{
	i2c.get_seq(reg_gyr_x_addr, 6);
	gyr_x.update();
	gyr_y.update();
	gyr_z.update();
}

/**
 * @brief Returns X angular velocity [rad/s]
 */
float MPU6050::get_gyr_x()
{
	return gyr_x * gyr_per_lsb - gyr_x_cal;
}

/**
 * @brief Returns Y angular velocity [rad/s]
 */
float MPU6050::get_gyr_y()
{
	return gyr_y * gyr_per_lsb - gyr_y_cal;
}

/**
 * @brief Returns Z angular velocity [rad/s]
 */
float MPU6050::get_gyr_z()
{
	return gyr_z * gyr_per_lsb - gyr_z_cal;
}

/**
 * @brief Calibrates gyroscope and estimates sensor variances
 * 
 * This function requires at minimum (MPU6050_CAL_SAMPLES * 4) bytes of RAM
 * to execute. The calibration parameters calculated by this method are only
 * valid if it executes while the IMU is at rest.
 */
void MPU6050::calibrate()
{
	// Sample storage
	float samples[MPU6050_CAL_SAMPLES];

	// Reset gyro offsets
	gyr_x_cal = 0.0f;
	gyr_y_cal = 0.0f;
	gyr_z_cal = 0.0f;

	// Accelerometer X
	for (uint32_t i = 0; i < MPU6050_CAL_SAMPLES; i++)
	{
		update();
		samples[i] = get_acc_x();
	}
	float acc_x_mean = mean(samples, MPU6050_CAL_SAMPLES);
	acc_x_var = var(samples, MPU6050_CAL_SAMPLES, acc_x_mean);

	// Accelerometer Y
	for (uint32_t i = 0; i < MPU6050_CAL_SAMPLES; i++)
	{
		update();
		samples[i] = get_acc_y();
	}
	float acc_y_mean = mean(samples, MPU6050_CAL_SAMPLES);
	acc_y_var = var(samples, MPU6050_CAL_SAMPLES, acc_y_mean);

	// Accelerometer Z
	for (uint32_t i = 0; i < MPU6050_CAL_SAMPLES; i++)
	{
		update();
		samples[i] = get_acc_z();
	}
	float acc_z_mean = mean(samples, MPU6050_CAL_SAMPLES);
	acc_z_var =  var(samples, MPU6050_CAL_SAMPLES, acc_z_mean);

	// Gyroscope X
	for (uint32_t i = 0; i < MPU6050_CAL_SAMPLES; i++)
	{
		update();
		samples[i] = get_gyr_x();
	}
	float gyr_x_mean = mean(samples, MPU6050_CAL_SAMPLES);
	gyr_x_var = var(samples, MPU6050_CAL_SAMPLES, gyr_x_mean);

	// Gyroscope Y
	for (uint32_t i = 0; i < MPU6050_CAL_SAMPLES; i++)
	{
		update();
		samples[i] = get_gyr_y();
	}
	float gyr_y_mean = mean(samples, MPU6050_CAL_SAMPLES);
	gyr_y_var = var(samples, MPU6050_CAL_SAMPLES, gyr_y_mean);

	// Gyroscope Z
	for (uint32_t i = 0; i < MPU6050_CAL_SAMPLES; i++)
	{
		update();
		samples[i] = get_gyr_z();
	}
	float gyr_z_mean = mean(samples, MPU6050_CAL_SAMPLES);
	gyr_z_var = var(samples, MPU6050_CAL_SAMPLES, gyr_z_mean);

	// Assign gyro offsets
	gyr_x_cal = gyr_x_mean;
	gyr_y_cal = gyr_y_mean;
	gyr_z_cal = gyr_z_mean;
}

/**
 * @brief Returns accelerometer x-axis variance [(m/s^2)^2]
 */
float MPU6050::get_acc_x_var()
{
	return acc_x_var;
}

/**
 * @brief Returns accelerometer y-axis variance [(m/s^2)^2]
 */
float MPU6050::get_acc_y_var()
{
	return acc_y_var;
}

/**
 * @brief Returns accelerometer z-axis variance [(m/s^2)^2]
 */
float MPU6050::get_acc_z_var()
{
	return acc_z_var;
}

/**
 * @brief Returns gyroscope x-axis variance [(rad/s)^2]
 */
float MPU6050::get_gyr_x_var()
{
	return gyr_x_var;
}

/**
 * @brief Returns gyroscope y-axis variance [(rad/s)^2]
 */
float MPU6050::get_gyr_y_var()
{
	return gyr_y_var;
}

/**
 * @brief Returns gyroscope z-axis variance [(rad/s)^2]
 */
float MPU6050::get_gyr_z_var()
{
	return gyr_z_var;
}