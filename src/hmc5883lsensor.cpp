#include "hmc5883ldriver/hmc5883lsensor.h"

extern "C" {
#include <errno.h>
#include <fcntl.h>
#include <i2c/smbus.h>
#include <linux/i2c-dev.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>
}

#include <iostream>

#define GAUSS_PER_BIT	1090
#define GAUSS_TO_TESLA	0.0001


HMC5883Sensor::HMC5883Sensor(int bus_number)
{
  // TODO: make char append cleaner
  filename_[9] = *std::to_string(bus_number).c_str();
  std::cout << filename_ << std::endl;
  file_ = open(filename_, O_RDWR);
  if (file_ < 0) {
    std::cerr << "Failed to open file descriptor! Check your bus number! Errno: "
              << strerror(errno);
    exit(1);
  }
  if (ioctl(file_, I2C_SLAVE, HMC5883_ADDRESS_DEFAULT) < 0) {
    std::cerr << "Failed to find device address! Check device address!";
    exit(1);
  }
  // Set sensor into continuous mode
  // Set Config register A
  int result = i2c_smbus_write_byte_data(file_, 0x00, 0x74);// 8 sample averaging and 30Hz update rate
  if (result < 0) reportError(errno);
  // Continuous measurement mode
   result = i2c_smbus_write_byte_data(file_, 0x02, 0x00);
  if (result < 0) reportError(errno);

}

HMC5883Sensor::~HMC5883Sensor() { close(file_); }

void HMC5883Sensor::getMagneticAll(){
	struct i2c_smbus_ioctl_data ioctl_data;
	union i2c_smbus_data smbus_data;

	smbus_data.block[0] = 6;// Set datablock to 6 bytes

	ioctl_data.read_write = I2C_SMBUS_READ;
	ioctl_data.command = 0x06;
	ioctl_data.size    = I2C_SMBUS_I2C_BLOCK_DATA;
	ioctl_data.data    = &smbus_data;


	int result;
	result = ioctl(file_, I2C_SMBUS, &ioctl_data);
//	result = i2c_smbus_read_i2c_block_data_or_emulated(file_, 0x06, 6, &read_buffer);
	if (result < 0) reportError(errno);
	memcpy(read_buffer, smbus_data.block+1, 6);
	result = i2c_smbus_read_byte_data(file_, 0x03);// Point the internal data pointer of the HMC5883L back to register 0x03
	if (result < 0) reportError(errno);
}

double HMC5883Sensor::getTelsaX() const{
	int itemp_x;
	double dtemp_x;
	
	itemp_x = (read_buffer[1] | (read_buffer[0] << 8));// Get data from buffer and convert to double
	dtemp_x = (itemp_x  / GAUSS_PER_BIT) * GAUSS_TO_TESLA;// Convert raw value to Gauss to Tesla
	
	return dtemp_x;
}

double HMC5883Sensor::getTelsaY() const{
	int itemp_y;
	double dtemp_y;
	
	itemp_y = (read_buffer[5] | (read_buffer[4] << 8));// Get data from buffer and convert to double
	dtemp_y = (itemp_y  / GAUSS_PER_BIT) * GAUSS_TO_TESLA;// Convert raw value to Gauss to Tesla

	return dtemp_y;
}

double HMC5883Sensor::getTelsaZ() const{
	int itemp_z;
	double dtemp_z;
	
	itemp_z = (read_buffer[3] | (read_buffer[2] << 8));// Get data from buffer and convert to double
	dtemp_z = (itemp_z  / GAUSS_PER_BIT) * GAUSS_TO_TESLA;// Convert raw value to Gauss to Tesla

	return dtemp_z;	
}

double HMC5883Sensor::getMagneticX() const
{
  int16_t mag_x_msb = i2c_smbus_read_byte_data(file_, MAG_XOUT_H);
  int16_t mag_x_lsb = i2c_smbus_read_byte_data(file_, MAG_XOUT_H + 1);
  int16_t mag_x = mag_x_lsb | mag_x_msb << 8;

  return  static_cast<double>(mag_x);
}

double HMC5883Sensor::getMagneticY() const
{
  int16_t mag_y_msb = i2c_smbus_read_byte_data(file_, MAG_YOUT_H);
  int16_t mag_y_lsb = i2c_smbus_read_byte_data(file_, MAG_YOUT_H + 1);
  int16_t mag_y = mag_y_lsb | mag_y_msb << 8;

  return  static_cast<double>(mag_y);
}

double HMC5883Sensor::getMagneticZ() const
{
  int16_t mag_z_msb = i2c_smbus_read_byte_data(file_, MAG_ZOUT_H);
  int16_t mag_z_lsb = i2c_smbus_read_byte_data(file_, MAG_ZOUT_H + 1);
  int16_t mag_z = mag_z_lsb | mag_z_msb << 8;

  return  static_cast<double>(mag_z);
}

void HMC5883Sensor::reportError(int error) { std::cerr << "Error! Errno: " << strerror(error); }
