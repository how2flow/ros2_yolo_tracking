#ifndef IMG_PUB_H_
#define IMG_PUB_H_

using namespace std::chrono_literals;

class CamPublisher_: public rclcpp::Node
{
public:
    CamPublisher_()
      : Node("camera")
    {
      setvbuf(stdout, NULL, _IONBF, BUFSIZ);
      init_rga();
      create_network();
      initialize();
    }

    ~CamPublisher_()
    {
      cap.release();
      free(resize_buf);
    }

private:
    void initialize();
    void init_rga();
    int create_network();
    unsigned char *load_data(FILE *fp, size_t ofst, size_t sz);
    unsigned char *load_model(const char *filename, int *model_size);
    void timerCallback();
    std::string mat2encoding(int mat_type);
    void convert_and_publish(const cv::Mat& frame);
    cv::VideoCapture cap;
    cv::Mat frame;

    int status;
    char *model_name;
    unsigned char *model_data;
    int model_data_size;
    rknn_context ctx;
    size_t actual_size;
    int img_channel = 0;
    const float nms_threshold = NMS_THRESH;
    const float box_conf_threshold = BOX_THRESH;
    int ret;

    rga_buffer_t src;
    rga_buffer_t dst;
    im_rect src_rect;
    im_rect dst_rect;

    rknn_sdk_version version;
    rknn_input_output_num io_num;
    rknn_tensor_attr input_attrs[1];
    rknn_tensor_attr output_attrs[3];
    rknn_input inputs[1];

    int channel;
    int width;
    int height;

    void *resize_buf;

    std::string topic_ = "/camera/image";
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};
#endif
