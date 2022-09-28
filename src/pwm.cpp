#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <stdlib.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "servo/pwm.hpp"

void PwmSignal_::initialize()
{
}

int PwmSignal_::pwm_setup(int chip)
{
  char cmd[100];

  if (pwmchip[chip] == NULL)
    return -1;

  sprintf(cmd, "echo %d > %s/%s", EXPORT_NUM, pwmchip[chip], pwm_option[EXPORT]);
  system(cmd);
  printf("cmd = %s\n", cmd);

  sprintf(cmd, "echo %d > %s/%s", MAX_T, pwmchip[chip], pwm_option[PERIOD]);
  system(cmd);
  printf("cmd = %s\n", cmd);

  sprintf(cmd, "echo %s > %s/%s", PWM_MODE, pwmchip[chip], pwm_option[POLARITY]);
  system(cmd);
  printf("cmd = %s\n", cmd);

  sprintf(cmd, "echo %d > %s/%s", ON, pwmchip[chip], pwm_option[ENABLE]);
  system(cmd);
  printf("cmd = %s\n", cmd);

  sprintf(cmd, "echo %d > %s/%s", DEFAULT_SIGNAL, pwmchip[chip], pwm_option[DUTY_CYCLE]);
  system(cmd);
  printf("cmd = %s\n", cmd);

  return 0;
}

int PwmSignal_::pwm_release(int chip)
{
  char cmd[100];

  if (pwmchip[chip] == NULL)
    return -1;

  sprintf(cmd, "echo %d > %s/%s", OFF, pwmchip[chip], pwm_option[ENABLE]);
  system(cmd);
  printf("cmd = %s\n", cmd);

  sprintf(cmd, "echo %d > %s/%s", EXPORT_NUM, pwmchip[chip], pwm_option[UNEXPORT]);
  system(cmd);
  printf("cmd = %s\n", cmd);

  return 0;
}

char* PwmSignal_::pwmchip[10] = {
  NULL,
  "/sys/devices/platform/fdd70010.pwm/pwm/pwmchip0",
  "/sys/devices/platform/fdd70020.pwm/pwm/pwmchip1",
  NULL, NULL, NULL, NULL, NULL, NULL,
  "/sys/devices/platform/fe6f0010.pwm/pwm/pwmchip4"
};

char* PwmSignal_::pwm_option[6] = {
  "export",
  "unexport",
  "pwm0/period",
  "pwm0/polarity",
  "pwm0/enable",
  "pwm0/duty_cycle",
};
