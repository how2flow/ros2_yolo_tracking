#ifndef SERVO_H_
#define SERVO_H_

#include "motor_srv/srv/motor_pos.hpp"

#define DUTY_X_MIN 3
#define DUTY_X_MAX 12
#define DUTY_Y_MIN 3
#define DUTY_Y_MAX 10
#define X_POS_MIN DUTY_X_MIN
#define X_POS_MAX DUTY_X_MAX
#define Y_POS_MIN DUTY_Y_MIN
#define Y_POS_MAX DUTY_Y_MAX

class Servo_ : public rclcpp::Node
{
public:
  using MotorPosition = motor_srv::srv::MotorPos;

  explicit Servo_(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
  virtual ~Servo_();

private:
  // functions
  void initialize();
  int dst_x_position(const int & pos, const int8_t flag);
  int dst_y_position(const int & pos, const int8_t flag);

  // vars
  geometry_msgs::msg::Point cam_pos;
  geometry_msgs::msg::Point duty;
  int motor_flag_;
  int motor_x_;
  int motor_y_;

  std::string topic_cpos = "/camera/pos";
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_cpos;
  std::string service_pos = "/motor/position";
  rclcpp::Service<MotorPosition>::SharedPtr
    motor_service_server_;
};
#endif
