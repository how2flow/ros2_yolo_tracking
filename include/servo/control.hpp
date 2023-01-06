#ifndef CONTROL_H_
#define CONTROL_H_

#include "motor_srv/srv/motor_pos.hpp"

// this is wiringpi pin number 23. not physical pin number.
#define MOTOR_X 23
// this is wiringpi pin number 7. not physical pin number.
// but in this case, they are same.
#define MOTOR_Y 7

// based on M1: 12Mhz (pwm1, pwm2)
// if you wna't use pwm9, set scale 4800.
// pwm9 freq is 24Mhz
#define PWM_SCALE 2400
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
