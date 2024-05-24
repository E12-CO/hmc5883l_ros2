#include "hmc5883ldriver/hmc5883ldriver.h"

#include <chrono>
#include <memory>

// Define read back Loop time 
#define LOOP_TIME_MIL   33 // 33ms -> 30Hz
#define LOOP_TIME_SEC	LOOP_TIME_MIL/1000 // Loop time in second

using namespace std::chrono_literals;

HMC5883Driver::HMC5883Driver()
    : Node("hmc5883publisher"), hmc5883_{std::make_unique<HMC5883Sensor>()}
{
  // Create publisher
  publisher_ = this->create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", 10);
  
  // Wall timer for sensor read back
  timer_ = this->create_wall_timer(
	std::chrono::milliseconds(LOOP_TIME_MIL), 
	std::bind(&HMC5883Driver::magCallback, this)
	);
}

void HMC5883Driver::magCallback()
{
  auto message = sensor_msgs::msg::MagneticField();
  message.header.stamp = this->get_clock()->now();
  message.header.frame_id = "imu_link";
  
  hmc5883_->getMagneticAll();
  
  message.magnetic_field.x = hmc5883_->getTelsaX();
  message.magnetic_field.y = hmc5883_->getTelsaY();
  message.magnetic_field.z = hmc5883_->getTelsaZ();
  
  publisher_->publish(message);
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HMC5883Driver>());
  rclcpp::shutdown();
  return 0;
}
