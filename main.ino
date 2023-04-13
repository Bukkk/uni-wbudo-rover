#include <stdint.h>

namespace pins {
//misc
constexpr uint8_t builtin_led = 13;

// silniki
// 11 10 9 6 5 3 orginalnie ale 9 10 sa uzywane przez timer1
constexpr uint8_t en_a = 6;  // bialy lewo 1.0
constexpr uint8_t in_1 = 7;
constexpr uint8_t in_2 = 8;
constexpr uint8_t in_3 = 2;
constexpr uint8_t in_4 = 4;
constexpr uint8_t en_b = 5;  // zolty prawo 1.0

// tick_sensor
// brazowy vcc
// czerwony gnd
constexpr uint8_t counter_a0 = A0;  // pomaranczowy
// szary vcc
// fioletowy gnd
constexpr uint8_t counter_a1 = A1;  // niebieski

// sonar
constexpr uint8_t sonar_trigger = A5;  // TODO color
constexpr uint8_t sonar_echo = A4;     // TODO color

//servo
constexpr uint8_t servo_control = 11;  // TODO color
}

namespace pins {
uint8_t with_mode(uint8_t pin, uint8_t mode) {
  pinMode(pin, mode);
  return pin;
}
}

namespace astd {
template<typename T1, typename T2>
class pair {
public:
  using first_type = T1;
  using second_type = T2;

  T1 first;
  T2 second;

  constexpr pair()
    : first{}, second{} {}

  constexpr pair(const T1& x, const T2& y)
    : first{ x }, second{ y } {}

  template<typename U1, typename U2>
  constexpr pair(const pair<U1, U2>& p)
    : first(p.first), second(p.second) {}

  pair& operator=(const pair& other) {
    first = other.first;
    second = other.second;
    return *this;
  }


  constexpr bool operator==(const pair& other) const {
    return (first == other.first) && (second == other.second);
  }

  constexpr bool operator!=(const pair& other) const {
    return !(*this == other);
  }
};

template<typename T1, typename T2>
constexpr pair<T1, T2> make_pair(T1 t, T2 u) {
  return pair<T1, T2>(t, u);
}

template<typename T, uint8_t N>
class array {
public:
  using value_type = T;
  using size_type = uint8_t;
  using reference = T&;
  using const_reference = const T&;
  using pointer = T*;
  using const_pointer = const T*;
  using iterator = pointer;
  using const_iterator = const_pointer;

  T data_[N];

  constexpr size_type size() const noexcept {
    return N;
  }

  constexpr bool empty() const noexcept {
    return N == 0;
  }

  // accessory
  reference operator[](size_type index) {
    return data_[index];
  }

  // its gnu++11 so only const members can be constexpred
  constexpr const_reference operator[](size_type index) const {
    return data_[index];
  }

  reference front() {
    return data_[0];
  }

  constexpr const_reference front() const {
    return data_[0];
  }

  reference back() {
    return data_[N - 1];
  }

  constexpr const_reference back() const {
    return data_[N - 1];
  }

  // iteratory
  iterator begin() noexcept {
    return &data_[0];
  }

  const_iterator begin() const noexcept {
    return &data_[0];
  }

  iterator end() noexcept {
    return &data_[N];
  }

  const_iterator end() const noexcept {
    return &data_[N];
  }
};
}

namespace counters {
using tick_t = uint32_t;

volatile uint32_t g_cnt_a0 = {};
volatile uint32_t g_cnt_a1 = {};

void init() {
  pinMode(pins::counter_a0, INPUT);
  pinMode(pins::counter_a1, INPUT);

  PCICR = 0x02;                             // set PCIE1 bit in PCICR to enable pin 14..8 interrupts
  PCMSK1 |= (1 << PCINT8) | (1 << PCINT9);  // run ISR for 9 8 pins
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
  const uint8_t pin_enable_;
  const uint8_t pin_forward_;
  const uint8_t pin_backward_;

  void set_power(uint8_t power) {
    analogWrite(pin_enable_, power);
  }

  void forward() {
    digitalWrite(pin_backward_, LOW);
    digitalWrite(pin_forward_, HIGH);
  }

  void backward() {
    digitalWrite(pin_forward_, LOW);
    digitalWrite(pin_backward_, HIGH);
  }

  void stop() {
    digitalWrite(pin_forward_, LOW);
    digitalWrite(pin_backward_, LOW);
  }
};

struct button {
  const uint8_t input_pin_;
};

struct tick_sensor {
  static constexpr float tick_to_cm = (2 * 40.) / 21.;

  volatile counters::tick_t& counter_;
};

struct sonar {
  static float constexpr _sound_speed_ms = 343.8;                      // m/s 20C
  static float constexpr _sound_speed_mmus = _sound_speed_ms * 0.001;  // mm/us
  static float constexpr delay_to_mm = _sound_speed_mmus * 0.5;        // 0.5 bo dzwiek pokonuje dwukrotnosc osleglosci w obie strony
  static float constexpr delay_to_cm = delay_to_mm * 0.1;

  const uint8_t trigger_pin_;
  const uint8_t echo_pin_;

  enum class error_reason : uint8_t {
    none,
    too_close,
    too_far
  };

  auto measure() -> astd::pair<uint16_t, error_reason> {
    using measurement_t = decltype(measure());

    digitalWrite(trigger_pin_, HIGH);
    delayMicroseconds(15);  // 10 minimum
    digitalWrite(trigger_pin_, LOW);

    uint32_t pulse = pulseIn(echo_pin_, HIGH, 35000);  // min 150us too close, max 25000 too far, 38000 no obstacle

    if (pulse < 150) {
      return measurement_t{ 0, error_reason::too_close };
    } else if (pulse > 25000) {
      return measurement_t{ 0, error_reason::too_far };
    } else {
      uint16_t meas = pulse * delay_to_cm;
      return measurement_t{ meas, error_reason::none };
    }
  }
};

}

#include <Servo.h>
namespace device {
struct servo {
  const uint8_t pin_control_;

private:
  Servo s_;
public:
  servo(uint8_t pin_control)
    : pin_control_{ pin_control } {
    s_.attach(pin_control_);
  }

  void set_angle(uint16_t deg) {
    s_.write(deg);
  }
};
}

namespace driver {

struct blocking_steering {
  device::motor& l_;
  device::motor& r_;

  device::tick_sensor& ls_;
  device::tick_sensor& rs_;

  void _go(void (device::motor::*l_action)(), void (device::motor::*r_action)(), uint16_t cm) {
    ls_.counter_ = 0;
    rs_.counter_ = 0;

    (l_.*l_action)();
    (r_.*r_action)();

    counters::tick_t ticks = cm * device::tick_sensor::tick_to_cm;

    bool lb = true;
    bool rb = true;
    while (lb || rb) {
      if (lb && ls_.counter_ >= ticks) {
        l_.stop();
        lb = false;
      }
      if (rb && rs_.counter_ >= ticks) {
        r_.stop();
        rb = false;
      }
    }
    ls_.counter_ = 0;
    rs_.counter_ = 0;
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

namespace constants {
  constexpr astd::array<uint16_t, 5> positions = { 0u, 45u, 90u, 135u, 180u };  // if i make it constexpr static member in blocking_sonar_tower compiler crashes XD
}
struct blocking_sonar_tower {
  device::sonar& sonar_;
  device::servo& servo_;

  auto measure() -> astd::array<astd::pair<uint16_t, device::sonar::error_reason>, 5> {
    decltype(measure()) results{};
    using constants::positions;

    for (uint8_t i{}; i < positions.size(); ++i) {
      uint8_t deg = positions[i];
      servo_.set_angle(deg);
      delay(100);  // aby servo sie obrocilo??
      // auto& [meas, res] = sonar_.measure();
      auto m = sonar_.measure();
      results[i] = m;
    }

    return results;
  }
};

}

// TODO przetestuj sonar
// TODO przetestuj odleglosc jechania
// TODO wymysl mnoznik na obrot
// TODO przetestuj serwo
// TODO sterownik sonaru na serwie

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
  device::tick_sensor sensor_left{ counters::g_cnt_a0 };
  device::tick_sensor sensor_right{ counters::g_cnt_a1 };

  device::motor left_motor{
    .pin_enable_ = pins::with_mode(pins::en_a, OUTPUT),
    .pin_forward_ = pins::with_mode(pins::in_1, OUTPUT),
    .pin_backward_ = pins::with_mode(pins::in_2, OUTPUT),
  };
  device::motor right_motor{
    .pin_enable_ = pins::with_mode(pins::en_b, OUTPUT),
    .pin_forward_ = pins::with_mode(pins::in_4, OUTPUT),
    .pin_backward_ = pins::with_mode(pins::in_3, OUTPUT),
  };
  driver::blocking_steering drive{
    .l_ = left_motor,
    .r_ = right_motor,
    .ls_ = sensor_left,
    .rs_ = sensor_right
  };

  device::sonar sonar{
    .trigger_pin_ = pins::with_mode(pins::sonar_trigger, OUTPUT),
    .echo_pin_ = pins::with_mode(pins::sonar_echo, INPUT)
  };
  device::servo servo{
    .pin_control = pins::with_mode(pins::servo_control, OUTPUT)
  };
  driver::blocking_sonar_tower tower{
    .sonar_ = sonar,
    .servo_ = servo
  };


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
    auto measurements = tower.measure();
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
