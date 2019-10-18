# MPU6050
Class for interfacing with MPU6050 6-axis I2C IMU  
Written by Dan Oates (WPI Class of 2020)

### Description
The MPU6050 is a 6-axis accelerometer, gyroscope, and thermometer. This library acts as an I2C interface with the device for the Arduino and Mbed platforms. This class supports gyroscope calibration by averaging the macro MPU6050_CAL_SAMPLES number of samples. Note that this operation requires at minimum (MPU6050_CAL_SAMPLES * 4) bytes of RAM.

### Dependencies
- [CppUtil](https://github.com/doates625/CppUtil.git)
- [Unions](https://github.com/doates625/Unions.git)
- [Platform](https://github.com/doates625/Platform.git)
- [I2CDevice](https://github.com/doates625/I2CDevice.git)

### References
- [Datasheet](http://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf)
- [Registers](https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf)