#include <stdint.h>

namespace pins {
constexpr uint8_t builtin_led = 13;

// 11 10 9 6 5 3 orginalnie ale 9 10 sa uzywane przez timer1
constexpr uint8_t en_a = 6;  // bialy lewo 1.0
constexpr uint8_t in_1 = 7;
constexpr uint8_t in_2 = 8;
constexpr uint8_t in_3 = 2;
constexpr uint8_t in_4 = 4;
constexpr uint8_t en_b = 5;  // zolty prawo 1.0

// brazowy vcc
// czerwony gnd
constexpr uint8_t counter_a0 = A0;  // pomaranczowy
// szary vcc
// fioletowy gnd
constexpr uint8_t counter_a1 = A1;  // niebieski

uint8_t with_mode(uint8_t pin, uint8_t mode) {
  pinMode(pin, mode);
  return pin;
}

}

namespace counters {
using counter_t = uint32_t;

volatile uint32_t g_cnt_a0 = {};
volatile uint32_t g_cnt_a1 = {};

void init() {
  pinMode(pins::counter_a0, INPUT);
  pinMode(pins::counter_a1, INPUT);

  PCICR = 0x02;
  PCMSK1 |= (1 << PCINT8) | (1 << PCINT9);
}

ISR(PCINT1_vect) {
  if ((PINC & (1 << PC0)))
    counters::g_cnt_a0++;

  if ((PINC & (1 << PC1)))
    counters::g_cnt_a1++;
}
}

#include <TimerOne.h>
namespace beeper {

void init() {
  Timer1.initialize();  // blokuje 9 i 10??
}

void beep() {
  auto state = digitalRead(13) ^ 1;
  digitalWrite(13, state);
}

void stop() {
  Timer1.detachInterrupt();
  digitalWrite(13, 0);
}

void start(uint32_t us) {
  digitalWrite(13, 0);
  Timer1.detachInterrupt();
  Timer1.attachInterrupt(beep, us);
}

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

struct button {
  const uint8_t _input_pin;
};

struct tick_sensor {
  static constexpr float sensor_to_cm = (2 * 40.) / 21.;

  volatile counters::counter_t& _counter;
};

}

namespace steering {

struct busy_drive {
  device::motor& l;
  device::motor& r;

  device::tick_sensor& ls;
  device::tick_sensor& rs;

  void _go(void (device::motor::*l_action)(), void (device::motor::*r_action)(), uint16_t cm) {
    ls._counter = 0;
    rs._counter = 0;

    (l.*l_action)();
    (r.*r_action)();

    uint16_t ticks = cm * device::tick_sensor::sensor_to_cm;

    bool lb = true;
    bool rb = true;
    while (lb || rb) {
      if (lb && ls._counter >= ticks) {
        l.stop();
        lb = false;
      }
      if (rb && rs._counter >= ticks) {
        r.stop();
        rb = false;
      }
    }
    ls._counter = 0;
    rs._counter = 0;
  }

  void forward(uint16_t cm) {
    _go(&device::motor::forward, &device::motor::forward, cm);
  }

  void backward(uint16_t cm) {
    _go(&device::motor::backward, &device::motor::backward, cm);
  }

  void rotate_left(uint16_t cm) {
    _go(&device::motor::backward, &device::motor::forward, cm);
  }

  void rotate_right(uint16_t cm) {
    _go(&device::motor::forward, &device::motor::backward, cm);
  }
};

}

void setup() {
  pinMode(pins::builtin_led, OUTPUT);
  digitalWrite(pins::builtin_led, HIGH);
  delay(2000);
  digitalWrite(pins::builtin_led, LOW);
}

void loop() {
  Serial.begin(9600);
  while (!Serial)
    ;

  beeper::init();

  counters::init();
  device::tick_sensor sensor_left = { counters::g_cnt_a0 };
  device::tick_sensor sensor_right = { counters::g_cnt_a1 };

  device::motor left_motor = {
    ._pin_enable = pins::with_mode(pins::en_a, OUTPUT),
    ._pin_forward = pins::with_mode(pins::in_1, OUTPUT),
    ._pin_backward = pins::with_mode(pins::in_2, OUTPUT),
  };
  device::motor right_motor = {
    ._pin_enable = pins::with_mode(pins::en_b, OUTPUT),
    ._pin_forward = pins::with_mode(pins::in_4, OUTPUT),
    ._pin_backward = pins::with_mode(pins::in_3, OUTPUT),
  };
  steering::busy_drive drive = { .l = left_motor, .r = right_motor, .ls = sensor_left, .rs = sensor_right };

  // symulator samochodzika
  // device::button buttie_left = {
  //   .input_pin = pins::with_mode(pins::sensor_left, INPUT_PULLUP),
  // };
  // device::button buttie_right = {
  //   .input_pin = pins::with_mode(pins::sensor_right, INPUT_PULLUP),
  // };
  // digitalWrite(pins::sensor_left, HIGH);
  // digitalWrite(pins::sensor_right, HIGH);

  left_motor.set_power(255);
  right_motor.set_power(255);
  for (;;) {

    // Serial.print("l: ");
    // Serial.print(sensor_left._counter);
    // Serial.print("r: ");
    // Serial.print(sensor_right._counter);
    // Serial.print("\n");
    // delay(100);

    drive.forward(40);
    delay(500);
    beeper::start(100000);
    drive.backward(40);
    beeper::stop();
    delay(500);
    drive.rotate_left(40);
    delay(500);
    drive.rotate_right(40);
    delay(500);
  }
}
