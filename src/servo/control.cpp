#include <chrono>
#include <cstdio>
#include <iostream>
#include <memory>
#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "servo/control.hpp"
/*
 * camera position.
 * pos_x: pwm_freq(50Hz), 3% right-end.
 * pos_y: pwm_freq(50Hz), 3% look at the top.
 * max_freq: x: 12%, y: 10%
 *
 */
PosPublisher_::PosPublisher_(const rclcpp::NodeOptions & node_options)
  : Node("control", node_options)
{
  this->initialize();
}

PosPublisher_::~PosPublisher_()
{
}

void PosPublisher_::initialize()
{
  this->declare_parameter("qos_depth", 10);
  int8_t qos_depth = this->get_parameter("qos_depth").get_value<int8_t>();
  this->get_parameter("qos_depth", qos_depth);

  pos.x = X_POS_MIN;
  pos.y = Y_POS_MIN;
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PosPublisher_>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

