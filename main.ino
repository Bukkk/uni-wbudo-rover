#include <stdint.h>

namespace pins {
constexpr uint8_t en_a = 11;  // bialy lewo 1.0
constexpr uint8_t in_1 = 10;
constexpr uint8_t in_2 = 9;
constexpr uint8_t in_3 = 6;
constexpr uint8_t in_4 = 5;
constexpr uint8_t en_b = 3;  // zolty prawo 1.0

constexpr uint8_t builtin_led = 13;
}

namespace device {
struct motor {
  const uint8_t _pin_enable;
  const uint8_t _pin_forward;
  const uint8_t _pin_backward;

  void set_power(uint8_t power) {
    analogWrite(_pin_enable, power);
  }

  void forward() {
    digitalWrite(_pin_backward, LOW);
    digitalWrite(_pin_forward, HIGH);
  }

  void backward() {
    digitalWrite(_pin_forward, LOW);
    digitalWrite(_pin_backward, HIGH);
  }

  void stop() {
    digitalWrite(_pin_forward, LOW);
    digitalWrite(_pin_backward, LOW);
  }
};
}


#include "beeper.hpp"

// void go_forward(uint16_t cm) {
//   steering::set_left(255);
//   steering::set_right(255);
//   steering::left_forward();
//   steering::right_forward();
//   delay(0.03 * cm * 1000);
//   steering::left_stop();
//   steering::right_stop();
// }

// void go_backward(uint16_t cm) {
//   steering::set_left(255);
//   steering::set_right(255);
//   steering::left_backward();
//   steering::right_backward();
//   delay(0.03 * cm * 1000);
//   steering::left_stop();
//   steering::right_stop();
// }

void setup() {
  pinMode(pins::builtin_led, OUTPUT);
  digitalWrite(pins::builtin_led, HIGH);
  delay(2000);
  digitalWrite(pins::builtin_led, LOW);
}

void loop() {
  pinMode(pins::en_a, OUTPUT);
  pinMode(pins::en_b, OUTPUT);
  pinMode(pins::in_1, OUTPUT);
  pinMode(pins::in_2, OUTPUT);
  pinMode(pins::in_3, OUTPUT);
  pinMode(pins::in_4, OUTPUT);
  pinMode(pins::builtin_led, OUTPUT);

  Timer1.initialize();

  device::motor left_motor = {
    ._pin_enable = pins::en_a,
    ._pin_forward = pins::in_1,
    ._pin_backward = pins::in_2,
  };

  device::motor right_motor = {
    ._pin_enable = pins::en_b,
    ._pin_forward = pins::in_4,
    ._pin_backward = pins::in_3,
  };

  auto& motor = left_motor;

  left_motor.set_power(255);
  right_motor.set_power(255);
  for (;;) {

    // go_forward(50);
    motor.forward();    
    delay(1000);


    // beeper::start(100000);
    // go_backward(50);
    motor.backward();
    delay(1000);

    // beeper::stop();
    motor.stop();
    delay(1000);


  }
}
