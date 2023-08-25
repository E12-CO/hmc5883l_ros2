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
  int result = i2c_smbus_write_byte_data(file_, 0x02, 0x00);// Might check for Endianness 
  if (result < 0) reportError(errno);

}

HMC5883Sensor::~HMC5883Sensor() { close(file_); }

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
