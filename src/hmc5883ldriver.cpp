#include "hmc5883ldriver/hmc5883ldriver.h"

#include <chrono>
#include <memory>

using namespace std::chrono_literals;

HMC5883Driver::HMC5883Driver()
    : Node("hmc5883publisher"), hmc5883_{std::make_unique<HMC5883Sensor>()}
{
  // Create publisher
  publisher_ = this->create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", 10);
  std::chrono::duration<int64_t, std::milli> frequency =
      1000ms / 100;
  timer_ = this->create_wall_timer(frequency, std::bind(&HMC5883Driver::handleInput, this));
}

void HMC5883Driver::handleInput()
{
  auto message = sensor_msgs::msg::MagneticField();
  message.header.stamp = this->get_clock()->now();
  message.header.frame_id = "imu_link";
  
  message.magnetic_field.x = hmc5883_->getMagneticX();
  message.magnetic_field.y = hmc5883_->getMagneticY();
  message.magnetic_field.z = hmc5883_->getMagneticZ();
  
  publisher_->publish(message);
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HMC5883Driver>());
  rclcpp::shutdown();
  return 0;
}
