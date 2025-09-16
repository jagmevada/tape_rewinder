#include <Arduino.h>

#pragma once

class MotorControllerWithPwmRamper {
  uint8_t led_indicator_pin;
  uint8_t& pwm_pin;
};
