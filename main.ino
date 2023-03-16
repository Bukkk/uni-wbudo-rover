#include <stdint.h>
namespace pins {
  constexpr uint8_t en_a = 11; // bialy lewo 1.0
  constexpr uint8_t in_1 = 10;
  constexpr uint8_t in_2 = 11;
  constexpr uint8_t in_3 = 11;
  constexpr uint8_t in_4 = 11;
  constexpr uint8_t en_b = 11; // zolty prawo 1.0

  constexpr uint8_t led = 13;
}

#define SPEED 0.016
#define LTRSPEED 0.9 

namespace steering {

  void set_left(uint8_t v) {
    analogWrite(pins::en_a, v);    
  }

  void set_right(uint8_t v) {
    analogWrite(pins::en_b, v);    
  }



  void left_forward() {
    analogWrite(pins::in_1, 255);
    analogWrite(pins::in_2, 0);
  }

  void left_backward() {
    analogWrite(pins::in_1, 0);
    analogWrite(pins::in_2, 255);
  }

  void right_forward() {
    analogWrite(pins::in_4, 255);
    analogWrite(pins::in_3, 0);
  }

  void right_backward() {
    analogWrite(pins::in_4, 0);
    analogWrite(pins::in_3, 255);
  }

  void left_stop() {
    analogWrite(pins::in_1, 0);
    analogWrite(pins::in_2, 0);
  }

  void right_stop() {
    analogWrite(pins::in_4, 0);
    analogWrite(pins::in_3, 0);
  }
  
}

#include <TimerOne.h>
template <uint8_t Pin>
struct beeper {

  beeper() {
    pinMode(Pin, OUTPUT);
    Timer1.initialize();    
  }

  static void beep() {
    uint8_t state = digitalRead(Pin) ^ 1;
    digitalWrite(Pin, state);
  }

  static void stop() {
    Timer1.detachInterrupt();
    digitalWrite(Pin, 0);
  }

  static void start(uint32_t period) {
    digitalWrite(Pin, 0);
    Timer1.detachInterrupt();
    Timer1.attachInterrupt(beep, period);
  }

};

void go_forward(uint16_t cm) {
  steering::set_left(255);
  steering::set_right(255);
  steering::left_forward();
  steering::right_forward();
  delay(SPEED * cm * 1000);
  steering::left_stop();
  steering::right_stop();
}

void go_backward(uint16_t cm) {
  steering::set_left(255);
  steering::set_right(255);
  steering::left_backward();
  steering::right_backward();
  delay(SPEED * cm * 1000);
  steering::left_stop();
  steering::right_stop();
}

void setup() {  
  digitalWrite(pins::led, 255);
  delay(2000);
  digitalWrite(pins::led, 0);  
}

void loop() {

  pinMode(pins::en_a, OUTPUT);
  pinMode(pins::en_b, OUTPUT);
  pinMode(pins::in_1, OUTPUT);
  pinMode(pins::in_2, OUTPUT);
  pinMode(pins::in_3, OUTPUT);
  pinMode(pins::in_4, OUTPUT);

  analogWrite(pins::en_a, 0);
  analogWrite(pins::en_b, 0);
  analogWrite(pins::in_1, 0);
  analogWrite(pins::in_4, 0);
  analogWrite(pins::in_2, 0);
  analogWrite(pins::in_3, 0);

  auto beep = beeper<pins::led>{};

  for(;;) {
    go_forward(50);
    
    delay(1000);

    beep.start(100000);
    go_backward(30);
    beep.stop();

    delay(1000);
  }
}
