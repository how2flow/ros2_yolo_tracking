#ifndef CONTROL_H_
#define CONTROL_H_

#define POS_INTERVAL 1
#define X_POS_MIN 3
#define Y_POS_MIN 3
#define X_POS_MAX 12
#define Y_POS_MAX 10


class PosPublisher_ : public rclcpp::Node
{
public:
  explicit PosPublisher_(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
  virtual ~PosPublisher_();

private:
  // functions
  void initialize();

  // vars
  geometry_msgs::msg::Point pos;
};
#endif
