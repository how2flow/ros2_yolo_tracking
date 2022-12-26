#ifndef SERVO_H_
#define SERVO_H_

#define MOTOR_X 23
#define MOTOR_Y 7
#define PWM_SCALE 2400 //based on 12Mhz (pwm1, pwm2)
#define PWM_PERIOD 100
#define X_POS_BASE ((PWM_PERIOD * 3) / 100)
#define Y_POS_BASE ((PWM_PERIOD * 3) / 100)

class Sg90Subscriber_ : public rclcpp::Node
{
public:
  explicit Sg90Subscriber_(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
  virtual ~Sg90Subscriber_();

private:
  // functions
  void initialize();
  void pwm_setup();
  void servo_write(geometry_msgs::msg::Point::SharedPtr pos);

  // vars
  geometry_msgs::msg::Point pos;
  std::string topic_pos = "/control/position";
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_pos;
};
#endif
