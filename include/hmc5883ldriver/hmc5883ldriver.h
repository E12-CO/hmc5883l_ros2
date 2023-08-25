#ifndef HMC5883DRIVER_H
#define HMC5883DRIVER_H

#include "hmc5883ldriver/hmc5883lsensor.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"

class HMC5883Driver : public rclcpp::Node {
 public:
  HMC5883Driver();

 private:
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr publisher_;
  std::unique_ptr<HMC5883Sensor> hmc5883_;
  size_t count_;
  rclcpp::TimerBase::SharedPtr timer_;
  void handleInput();
};

#endif  // HMC5883DRIVER_H
