#ifndef PWM_H_
#define PWM_H_

// pwm const
#define EXPORT_NUM 0
#define MAX_T 20000000
#define PWM_MODE "normal"
#define DEFAULT_SIGNAL 500000

// pwm option idx
#define EXPORT 0
#define UNEXPORT 1
#define PERIOD 2
#define POLARITY 3
#define ENABLE 4
#define DUTY_CYCLE 5

#define ON 1
#define OFF 1

class PwmSignal_ : public rclcpp::Node
{
public:
  PwmSignal_()
  : Node("pwm")
  {
    initialize();
  }

  ~PwmSignal_()
  {
  }

  static int pwm_setup(int chip);
  static int pwm_release(int chip);
  static char* pwmchip[10];
  static char* pwm_option[6];

private:
  // functions
  void initialize();
};

#endif
