#ifndef SERVO_H_
#define SERVO_H_

#define MOTOR_X 23
#define MOTOR_Y 7
#define PWM_SCALE 2400 //based on 12Mhz (pwm1, pwm2)
#define PWM_PERIOD 100
#define X_POS_BASE ((PWM_PERIOD * 6) / 100)
#define Y_POS_BASE ((PWM_PERIOD * 5) / 100)

class Sg90Subscriber_ : public rclcpp::Node
{
public:
  explicit Sg90Subscriber_(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
  virtual ~Sg90Subscriber_();

private:
  // functions
  void initialize();
  void pwm_setup();

  // vars
  geometry_msgs::msg::Point pos;
  geometry_msgs::msg::Point cam_pos;

  std::string topic_cpos = "/camera/pos";
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_cpos;
};
#endif
