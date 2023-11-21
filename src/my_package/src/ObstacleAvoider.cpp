#include "my_package/ObstacleAvoider.hpp"

#define MAX_RANGE (0.15)

ObstacleAvoider::ObstacleAvoider() : Node("obstacle_avoider") {
  publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);

  /*left_sensor_sub_ = create_subscription<sensor_msgs::msg::Range>(
      "/left_sensor", 1,
      std::bind(&ObstacleAvoider::leftSensorCallback, this,
                std::placeholders::_1));

  right_sensor_sub_ = create_subscription<sensor_msgs::msg::Range>(
      "/right_sensor", 1,
      std::bind(&ObstacleAvoider::rightSensorCallback, this,
                std::placeholders::_1));*/
  
  lidar_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/lidar", 1,
      std::bind(&ObstacleAvoider::lidarSensorCallback, this,
                std::placeholders::_1));
}

void ObstacleAvoider::leftSensorCallback(
    const sensor_msgs::msg::Range::SharedPtr msg) {
  left_sensor_value = msg->range;
}

void ObstacleAvoider::rightSensorCallback(
    const sensor_msgs::msg::Range::SharedPtr msg) {
  right_sensor_value = msg->range;

  auto command_message = std::make_unique<geometry_msgs::msg::Twist>();

  command_message->linear.x = 0.1;

  if (left_sensor_value < 0.9 * MAX_RANGE ||
      right_sensor_value < 0.9 * MAX_RANGE) {
    command_message->angular.z = -2.0;
  }

  publisher_->publish(std::move(command_message));
}

void ObstacleAvoider::lidarSensorCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  std::cout << "lidar scan received\n";
  lidar_scan = std::move(msg->ranges);
  auto command_message = std::make_unique<geometry_msgs::msg::Twist>();

  bool detected{false};
  for (const auto& val: lidar_scan)
  {
    if (val < 0.2)
    {
      detected = true;
      command_message->angular.z = -0.5;
      break;
    }
    else
    {
      detected = false;
    }
  }

  if (!detected)
  {
    command_message->linear.x = 0.1;
  }
  publisher_->publish(std::move(command_message));
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto avoider = std::make_shared<ObstacleAvoider>();
  rclcpp::spin(avoider);
  rclcpp::shutdown();
  return 0;
}