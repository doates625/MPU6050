/**
 * @file MPU6050.cpp
 * @author Dan Oates (WPI Class of 2020)
 */
#include "MPU6050.h"
#include <CppUtil.h>

/**
 * Static Constants
 */
const float MPU6050::tmp_per_lsb = 1.0f / 340.0f;
const float MPU6050::tmp_offset_c = 36.53f;

/**
 * @brief Constructor for MPU-6050 interface
 * @param i2c Platform-specific I2C interface
 * @param add_sel Address select line (1 = high, 0 = low)
 */
MPU6050::MPU6050(I2CDEVICE_I2C_CLASS* i2c, bool addr_sel)
{
	// I2C interface
	this->i2c_addr = addr_sel ? 0x69 : 0x68;
	this->i2c = I2CDevice(i2c, i2c_addr, I2CDevice::msb_first);

	// State data
	this->acc_x = 0.0f; read_acc_x = false;
	this->acc_y = 0.0f; read_acc_y = false;
	this->acc_z = 0.0f; read_acc_z = false;
	this->tmp_c = 0.0f; read_tmp_c = false;
	this->gyr_x = 0.0f; read_gyr_x = false;
	this->gyr_y = 0.0f; read_gyr_y = false;
	this->gyr_z = 0.0f; read_gyr_z = false;

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
	uint8_t addr = i2c.read_uint8(reg_whoami_addr);
	if ((addr >> 1) != (i2c_addr >> 1))
	{
		return false;
	}

	// Configure accelerometer and gyroscope
	set_acc_range(acc_2G);
	set_gyr_range(gyr_250dps);

	// Wake up device (might need to put first...)
	i2c.write_uint8(reg_pwrmgmt_addr, reg_pwrmgmt_wake);
	
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
	i2c.write_uint8(reg_acc_config_addr, reg_val);
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
	i2c.write_uint8(reg_gyr_config_addr, reg_val);
	gyr_per_lsb = gyr_range_dps * (M_PI / 180.0f) / 32768.0f;
}

/**
 * @brief Updates all IMU readings
 */
void MPU6050::update()
{
	i2c.read_sequence(reg_acc_x_addr, 14);
	read_acc();
	read_tmp();
	read_gyr();
}

/**
 * @brief Updates accelerometer readings
 */
void MPU6050::update_acc()
{
	i2c.read_sequence(reg_acc_x_addr, 6);
	read_acc();
}

/**
 * @brief Returns X acceleration [m/s^2]
 */
float MPU6050::get_acc_x()
{
	if (read_acc_x)
	{
		read_acc_x = false;
		return acc_x;
	}
	return i2c.read_int16(reg_acc_x_addr) * acc_per_lsb;
}

/**
 * @brief Returns Y acceleration [m/s^2]
 */
float MPU6050::get_acc_y()
{
	if (read_acc_y)
	{
		read_acc_y = false;
		return acc_y;
	}
	return i2c.read_int16(reg_acc_y_addr) * acc_per_lsb;
}

/**
 * @brief Returns Z acceleration [m/s^2]
 */
float MPU6050::get_acc_z()
{
	if (read_acc_z)
	{
		read_acc_z = false;
		return acc_z;
	}
	return i2c.read_int16(reg_acc_z_addr) * acc_per_lsb;
}

/**
 * @brief Updates thermometer reading
 */
void MPU6050::update_tmp()
{
	i2c.read_sequence(reg_tmp_addr, 2);
	read_tmp();
}

/**
 * @brief Returns temperature [deg C]
 */
float MPU6050::get_tmp_c()
{
	if (read_tmp_c)
	{
		read_tmp_c = false;
		return tmp_c;
	}
	return i2c.read_int16(reg_tmp_addr) * tmp_per_lsb + tmp_offset_c;
}

/**
 * @brief Updates gyroscope readings
 */
void MPU6050::update_gyr()
{
	i2c.read_sequence(reg_gyr_x_addr, 6);
	read_gyr();
}

/**
 * @brief Returns X angular velocity [rad/s]
 */
float MPU6050::get_gyr_x()
{
	if (read_gyr_x)
	{
		read_gyr_x = false;
		return gyr_x;
	}
	return i2c.read_int16(reg_gyr_x_addr) * gyr_per_lsb - gyr_x_cal;;
}

/**
 * @brief Returns Y angular velocity [rad/s]
 */
float MPU6050::get_gyr_y()
{
	if (read_gyr_y)
	{
		read_gyr_y = false;
		return gyr_y;
	}
	return i2c.read_int16(reg_gyr_y_addr) * gyr_per_lsb - gyr_y_cal;
}

/**
 * @brief Returns Z angular velocity [rad/s]
 */
float MPU6050::get_gyr_z()
{
	if (read_gyr_z)
	{
		read_gyr_z = false;
		return gyr_z;
	}
	return i2c.read_int16(reg_gyr_z_addr) * gyr_per_lsb - gyr_z_cal;
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
		samples[i] = acc_x;
	}
	float acc_x_mean = Util::mean(samples, MPU6050_CAL_SAMPLES);
	acc_x_var = Util::var(samples, MPU6050_CAL_SAMPLES, acc_x_mean);

	// Accelerometer Y
	for (uint32_t i = 0; i < MPU6050_CAL_SAMPLES; i++)
	{
		update();
		samples[i] = acc_y;
	}
	float acc_y_mean = Util::mean(samples, MPU6050_CAL_SAMPLES);
	acc_y_var = Util::var(samples, MPU6050_CAL_SAMPLES, acc_y_mean);

	// Accelerometer Z
	for (uint32_t i = 0; i < MPU6050_CAL_SAMPLES; i++)
	{
		update();
		samples[i] = acc_z;
	}
	float acc_z_mean = Util::mean(samples, MPU6050_CAL_SAMPLES);
	acc_z_var =  Util::var(samples, MPU6050_CAL_SAMPLES, acc_z_mean);

	// Gyroscope X
	for (uint32_t i = 0; i < MPU6050_CAL_SAMPLES; i++)
	{
		update();
		samples[i] = gyr_x;
	}
	float gyr_x_mean = Util::mean(samples, MPU6050_CAL_SAMPLES);
	gyr_x_var = Util::var(samples, MPU6050_CAL_SAMPLES, gyr_x_mean);

	// Gyroscope Y
	for (uint32_t i = 0; i < MPU6050_CAL_SAMPLES; i++)
	{
		update();
		samples[i] = gyr_y;
	}
	float gyr_y_mean = Util::mean(samples, MPU6050_CAL_SAMPLES);
	gyr_y_var = Util::var(samples, MPU6050_CAL_SAMPLES, gyr_y_mean);

	// Gyroscope Z
	for (uint32_t i = 0; i < MPU6050_CAL_SAMPLES; i++)
	{
		update();
		samples[i] = gyr_z;
	}
	float gyr_z_mean = Util::mean(samples, MPU6050_CAL_SAMPLES);
	gyr_z_var = Util::var(samples, MPU6050_CAL_SAMPLES, gyr_z_mean);

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

/**
 * @brief Returns gyroscope x-axis calibration offset [rad/s]
 */
float MPU6050::get_gyr_x_cal()
{
	return gyr_x_cal;
}

/**
 * @brief Returns gyroscope y-axis calibration offset [rad/s]
 */
float MPU6050::get_gyr_y_cal()
{
	return gyr_y_cal;
}

/**
 * @brief Returns gyroscope z-axis calibration offset [rad/s]
 */
float MPU6050::get_gyr_z_cal()
{
	return gyr_z_cal;
}

/**
 * @brief Sets gyroscope x-axis calibration offset
 * @param offset Calibration offset [rad/s]
 */
void MPU6050::set_gyr_x_cal(float offset)
{
	gyr_x_cal = offset;
}

/**
 * @brief Sets gyroscope y-axis calibration offset
 * @param offset Calibration offset [rad/s]
 */
void MPU6050::set_gyr_y_cal(float offset)
{
	gyr_y_cal = offset;
}

/**
 * @brief Sets gyroscope z-axis calibration offset
 * @param offset Calibration offset [rad/s]
 */
void MPU6050::set_gyr_z_cal(float offset)
{
	gyr_z_cal = offset;
}

/**
 * @brief Reads acc registers after call to I2CDevice::read_sequence()
 */
void MPU6050::read_acc()
{
	acc_x = i2c.read_int16() * acc_per_lsb; read_acc_x = true;
	acc_y = i2c.read_int16() * acc_per_lsb; read_acc_y = true;
	acc_z = i2c.read_int16() * acc_per_lsb; read_acc_z = true;
}

/**
 * @brief Reads tmp registers after call to I2CDevice::read_sequence()
 */
void MPU6050::read_tmp()
{
	tmp_c = i2c.read_uint16() * tmp_per_lsb + tmp_offset_c;
	read_tmp_c = true;
} 

/**
 * @brief Reads gyr registers after call to I2CDevice::read_sequence()
 */
void MPU6050::read_gyr()
{
	gyr_x = i2c.read_int16() * gyr_per_lsb; read_gyr_x = true;
	gyr_y = i2c.read_int16() * gyr_per_lsb; read_gyr_y = true;
	gyr_z = i2c.read_int16() * gyr_per_lsb; read_gyr_z = true;
}