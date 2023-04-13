#include <stdint.h>

namespace pins {
constexpr uint8_t en_a = 11;  // bialy lewo 1.0
constexpr uint8_t in_1 = 10;
constexpr uint8_t in_2 = 9;
constexpr uint8_t in_3 = 6;
constexpr uint8_t in_4 = 5;
constexpr uint8_t en_b = 3;  // zolty prawo 1.0

// brazowy vcc
// czerwony gnd
constexpr uint8_t sensor_left = A0;  // pomaranczowy
// szary vcc
// fioletowy gnd
constexpr uint8_t sensor_right = A1;  // niebieski

constexpr uint8_t builtin_led = 13;

uint8_t with_mode(uint8_t pin, uint8_t mode) {
  pinMode(pin, mode);
  return pin;
}
}

#include <TimerOne.h>
namespace beeper {

void beep() {
  auto state = digitalRead(13) ^ 1;
  digitalWrite(13, state);
}

void stop() {
  Timer1.detachInterrupt();
  digitalWrite(13, 0);
}

void start(long int period) {
  digitalWrite(13, 0);
  Timer1.detachInterrupt();
  Timer1.attachInterrupt(beep, period);
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
  const uint8_t input_pin;
};

struct sensor {
  volatile uint8_t _counter = {};

  reset() {
    _counter = 0;
  }
};
}

namespace steering {
constexpr float sensor_to_cm = (2 * 40) / 21;

struct drive {
  device::motor& l;
  device::motor& r;

  device::sensor& ls;
  device::sensor& rs;

  void _go(uint16_t cm, void (device::motor::*l_direction)(), void (device::motor::*r_direction)()) {
    ls.reset();
    rs.reset();

    (l.*l_direction)();
    (r.*r_direction)();

    uint16_t ticks = cm * sensor_to_cm;

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
    ls.reset();
    rs.reset();
  }

  void go_forward(uint16_t cm) {
    _go(cm, &device::motor::forward, &device::motor::forward);
  }

  void go_backward(uint16_t cm) {
    _go(cm, &device::motor::backward, &device::motor::backward);
  }

  void go_left(uint16_t cm) {
    _go(cm, &device::motor::backward, &device::motor::forward);
  }

  void go_right(uint16_t cm) {
    _go(cm, &device::motor::forward, &device::motor::backward);
  }
};
}



void setup() {
  pinMode(pins::builtin_led, OUTPUT);
  digitalWrite(pins::builtin_led, HIGH);
  delay(2000);
  digitalWrite(pins::builtin_led, LOW);
}

device::sensor sensor_left = {};
device::sensor sensor_right = {};
volatile uint8_t cnt1 = {};
ISR(PCINT1_vect) {
  if ((PINC & (1 << PC0)))
    sensor_right._counter++;

  if ((PINC & (1 << PC1)))
    sensor_left._counter++;
}

void loop() {
  Serial.begin(9600);
  Timer1.initialize();  // blokuje 9 i 10
  while (!Serial)
    ;


  PCICR = 0x02;
  PCMSK1 |= (1 << PCINT8) | (1 << PCINT9);

  // podzielona inicjalizacja sensorleft i right przez ISR

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

  steering::drive drive = { .l = left_motor, .r = right_motor, .ls = sensor_left, .rs = sensor_right };

  for (;;) {

    // // go_forward(50);
    // motor.forward();
    // delay(1000);

    // // beeper::start(100000);
    // // go_backward(50);
    // motor.backward();
    // delay(1000);

    // // beeper::stop();
    // motor.stop();
    // delay(1000);

    // Serial.println(cnt0);

    // Serial.print("l: ");
    // Serial.print(sensor_left._counter);
    // Serial.print("r: ");
    // Serial.print(sensor_right._counter);
    // Serial.print("\n");
    // delay(100);

    drive.go_forward(40);
    delay(500);
    drive.go_backward(40);
    delay(500);
    drive.go_left(40);
    delay(500);
    drive.go_right(40);
    delay(500);
  }
}
