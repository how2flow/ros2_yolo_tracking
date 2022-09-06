#ifndef IMG_SUB_H_
#define IMG_SUB_H_
class CamSubscriber_ : public rclcpp::Node
{
public:
    CamSubscriber_()
      : Node("show")
    {
      setvbuf(stdout, NULL, _IONBF, BUFSIZ);
      initialize();
    }

    ~CamSubscriber_()
    {
    }

private:
    //functions
    void initialize();
    int encoding2mat(const std::string& encoding);
    void process_image(const sensor_msgs::msg::Image::SharedPtr msg);
    std::string topic_ = "/camera/mat2image_image2mat";
    std::string window_name = "Live";
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};
#endif
