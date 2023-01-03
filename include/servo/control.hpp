#ifndef CONTROL_H_
#define CONTROL_H_

#include "motor_srv/srv/motor_pos.hpp"

#define MOTOR_X 23
#define MOTOR_Y 7
#define PWM_SCALE 2400 // based on M1: 12Mhz (pwm1, pwm2)
#define PWM_PERIOD 100
#define X_POS_BASE 6
#define Y_POS_BASE 5
#define DELAY 30

class PosContol_ : public rclcpp::Node
{
public:
  using MotorPosition = motor_srv::srv::MotorPos;

  explicit PosContol_(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
  virtual ~PosContol_();
  void send_request();
private:
  // functions
  void pwm_setup();
  void initialize();

  // vars
  geometry_msgs::msg::Point pos;
  std::string service_pos = "/motor/position";
  rclcpp::Client<MotorPosition>::SharedPtr
    motor_service_client_;
};
#endif
