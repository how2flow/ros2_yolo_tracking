#ifndef SERVO_H_
#define SERVO_H_

#define PWM_SCALE 120
#define PWM_PERIOD 2000
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
  std::string topic_pos = "/control/position";
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_pos;
  geometry_msgs::msg::Point pos;
};
#endif
