#include <stdint.h>
#include <stdio.h>

namespace config {
constexpr bool enable_logs = false;
}

namespace pins {
//misc
constexpr uint8_t builtin_led = 13;

// silniki
// 11 10 9 6 5 3 orginalnie ale 9 10 sa uzywane przez timer1
// pasek lewo bialy szary folet niebieski zielony zolty prawo
constexpr uint8_t en_a = 6;  // bialy lewo 1.0
constexpr uint8_t in_1 = 7;  // szary
constexpr uint8_t in_2 = 8;  // filet
constexpr uint8_t in_3 = 2;  // niebieski
constexpr uint8_t in_4 = 4;  // zielony
constexpr uint8_t en_b = 5;  // zolty prawo 1.0

// tick_sensor
// brazowy vcc
// czerwony gnd
constexpr uint8_t counter_a1 = A1;  // pomaranczowy prawy
// szary vcc
// fioletowy gnd
constexpr uint8_t counter_a0 = A0;  // niebieski lewy

// sonar
constexpr uint8_t sonar_trigger = A5;  // opisany pin
constexpr uint8_t sonar_echo = A4;     // opisany pin

//servo
// carny gnd
constexpr uint8_t servo_control = 11;  // zolty
// czerwony vcc

// wyswietlacz sda scl vcc gnd
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

  constexpr const_iterator begin() const noexcept {
    return &data_[0];
  }

  iterator end() noexcept {
    return &data_[N];
  }

  constexpr const_iterator end() const noexcept {
    return &data_[N];
  }
};

template<typename ForwardIt, typename Compare>
ForwardIt max_element(ForwardIt first, ForwardIt last, Compare less) {
  if (first == last)
    return last;

  ForwardIt largest = first;
  ++first;

  for (; first != last; ++first)
    if (less(*largest, *first))
      largest = first;

  return largest;
}
}

namespace counters {
using tick_t = uint32_t;

volatile tick_t g_cnt_a0 = {};
volatile tick_t g_cnt_a1 = {};

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

// no remove_cv -> no template
struct cache {
  tick_t cache_ = {};
  volatile tick_t& ct_;


  cache(volatile tick_t& ct)
    : ct_(ct) {}

  void reset() {
    cache_ = ct_;
  }

  tick_t current() {
    return ct_ - cache_;
  }
};
}

#include <TimerOne.h>
namespace beeper {

void init() {
  Timer1.initialize();  // is using pins 9 and 10
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

  enum class states : uint8_t {
    STOP = 0,
    FORWARD,
    BACKWARD,
  };

  uint8_t power_;
  states state_;

  void set_power(uint8_t power) {
    power_ = power;
    analogWrite(pin_enable_, power);
  }

  void forward() {
    state_ = states::FORWARD;
    digitalWrite(pin_backward_, LOW);
    digitalWrite(pin_forward_, HIGH);
  }

  void backward() {
    state_ = states::BACKWARD;
    digitalWrite(pin_forward_, LOW);
    digitalWrite(pin_backward_, HIGH);
  }

  void stop() {
    state_ = states::STOP;
    digitalWrite(pin_forward_, LOW);
    digitalWrite(pin_backward_, LOW);
  }
};

struct button {
  const uint8_t input_pin_;
};

struct tick_sensor {
  counters::cache counter_;

  void reset() {
    counter_.reset();
  }

  auto current() -> counters::tick_t {
    return counter_.current();
  }
};

struct sonar {
  static float constexpr _sound_speed_ms = 343.8;                      // m/s 20C
  static float constexpr _sound_speed_mmus = _sound_speed_ms * 0.001;  // mm/us
  static float constexpr mm_per_pulse = _sound_speed_mmus * 0.5;       // 0.5 sound travels twice the distance from sensor to obstacle
  static float constexpr cm_per_pulse = mm_per_pulse * 0.1;

  static uint16_t constexpr min_us = 150;
  static uint16_t constexpr max_us = 25000;
  static float constexpr min_cm = min_us * cm_per_pulse;
  static float constexpr max_cm = max_us * cm_per_pulse;

  const uint8_t trigger_pin_;
  const uint8_t echo_pin_;

  enum class error_reason : uint8_t {
    none,
    too_close,
    too_far
  };

  auto measure() -> astd::pair<uint16_t, error_reason> {
    digitalWrite(trigger_pin_, HIGH);
    delayMicroseconds(15);  // 10 minimum
    digitalWrite(trigger_pin_, LOW);

    uint32_t pulse = pulseIn(echo_pin_, HIGH, 30000);  // min 150us too close, max 25000 too far, 38000 no obstacle

    if (pulse < min_us) {
      return { 0, error_reason::too_close };
    } else if (pulse > max_us) {
      return { 0, error_reason::too_far };
    } else {
      uint16_t meas = pulse * cm_per_pulse;
      return { meas, error_reason::none };
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

  void reset_position() {
    s_.write(90);
  }
};
}

#include "LiquidCrystal_I2C.h"
namespace device {

struct lcd_wrapper {
  LiquidCrystal_I2C lcd_ = LiquidCrystal_I2C(0x27, 16, 2);

  void init() {
    lcd_.init();
    lcd_.backlight();
  }

  void clear() {
    lcd_.clear();
  }

  void uwu_message() {
    lcd_.setCursor(0, 0);
    lcd_.print("stiww pwepawing!");
    lcd_.setCursor(0, 1);
    lcd_.print("touwch my eaws:3");
  }
};
}

namespace driver {

struct steering_blocking {
  device::motor& l_;
  device::motor& r_;

  device::tick_sensor& ls_;
  device::tick_sensor& rs_;

  static constexpr float tick_to_cm = (40.) / 21.;
  static constexpr float deg_to_cm = 0.3;

  void _go(void (device::motor::*l_action)(), void (device::motor::*r_action)(), uint16_t cm) {
    ls_.reset();
    rs_.reset();

    (l_.*l_action)();
    (r_.*r_action)();

    counters::tick_t ticks = cm * tick_to_cm;

    bool lb = true;
    bool rb = true;
    while (lb || rb) {
      if (lb && ls_.current() >= ticks) {
        l_.stop();
        lb = false;
      }
      if (rb && rs_.current() >= ticks) {
        r_.stop();
        rb = false;
      }

      // FIXME if counter is badly connected, it loops forever
    }

    if (config::enable_logs) {
      Serial.print("l counter: ");
      Serial.print(ls_.current());
      Serial.print("r counter: ");
      Serial.print(rs_.current());
      Serial.print("\n");
    }
    // ls_.reset();
    // rs_.reset();
  }

  void forward(uint16_t cm) {
    _go(&device::motor::forward, &device::motor::forward, cm);
  }

  void backward(uint16_t cm) {
    _go(&device::motor::backward, &device::motor::backward, cm);
  }

  void rotate_left(uint16_t deg) {
    _go(&device::motor::backward, &device::motor::forward, deg * deg_to_cm);
  }

  void rotate_right(uint16_t deg) {
    _go(&device::motor::forward, &device::motor::backward, deg * deg_to_cm);
  }

  void rotate(int16_t deg_to_left) {
    if (deg_to_left >= 0) {
      rotate_left(static_cast<uint16_t>(deg_to_left));
    } else {
      rotate_right(static_cast<uint16_t>(-deg_to_left));
    }
  }
};

namespace constants {
// left to right
// if i make it constexpr static member in blocking_sonar_tower compiler crashes XD
constexpr astd::array<uint16_t, 5> _positions = { 180u, 135u, 90u, 45u, 0u };
}
struct blocking_sonar_tower {
  device::sonar& sonar_;
  device::servo& servo_;

  auto measure() -> astd::array<decltype(sonar_.measure()), constants::_positions.size()> {
    decltype(measure()) results{};

    for (uint8_t i{}; i < constants::_positions.size(); ++i) {
      uint8_t deg = constants::_positions[i];
      servo_.set_angle(deg);
      delay(300);  // time to allow servo to rotate
      auto m = sonar_.measure();
      results[i] = m;
    }

    servo_.reset_position();
    if (config::enable_logs) {
      for (uint8_t i{}; i < constants::_positions.size(); ++i) {
        Serial.print("angle=");
        Serial.print(constants::_positions[i]);
        Serial.print(",result=");
        Serial.print(results[i].first);
        Serial.print("\n");
      }
    }
    return results;
  }
};

}

namespace statistics {
template<typename T, typename Time>
double calc_estimate(T now, T last, Time t_now, Time t_last) {
  auto d = now - last;
  auto dt = t_now - t_last;
  return dt != 0 ? double(d) / dt : 0.;
}

template<typename T>
struct simple {
  T now, last;
  decltype(millis()) t_now, t_last;

  void add(T n) {
    last = now;
    now = n;
    t_last = t_now;
    t_now = millis();
  }

  auto calc() -> double {
    return calc_estimate(now, last, t_now, t_last);
  }
};
}

namespace driver {
struct velocity_counter {
  counters::cache cache_;
  device::motor& motor_;
  statistics::simple<counters::tick_t> s_;

  double current() {
    auto v = s_.calc();
    return motor_.state_ == device::motor::states::BACKWARD ? -v : v;
  }

  void probe() {
    auto ticks = cache_.current();
    s_.add(ticks);
  }
};
}


void panic() {
  beeper::start(100000);
  for (;;)
    ;
}

void setup() {
  pinMode(pins::builtin_led, OUTPUT);
  digitalWrite(pins::builtin_led, HIGH);
  delay(2000);
  digitalWrite(pins::builtin_led, LOW);
}

void loop() {
  beeper::init();

  device::lcd_wrapper lcd{};
  lcd.init();

  // beeper::start(100000);
  // lcd.uwu_message();

  if (config::enable_logs) {
    Serial.begin(9600);
    while (!Serial)
      ;
  }


  counters::init();
  device::tick_sensor sensor_left{ counters::cache{ counters::g_cnt_a0 } };
  device::tick_sensor sensor_right{ counters::cache{ counters::g_cnt_a1 } };
  device::motor motor_left{
    .pin_enable_ = pins::with_mode(pins::en_a, OUTPUT),
    .pin_forward_ = pins::with_mode(pins::in_1, OUTPUT),
    .pin_backward_ = pins::with_mode(pins::in_2, OUTPUT),
  };
  device::motor motor_right{
    .pin_enable_ = pins::with_mode(pins::en_b, OUTPUT),
    .pin_forward_ = pins::with_mode(pins::in_4, OUTPUT),
    .pin_backward_ = pins::with_mode(pins::in_3, OUTPUT),
  };
  driver::velocity_counter speedometer_left = { .cache_ = counters::cache{ counters::g_cnt_a0 }, .motor_ = motor_left };
  driver::velocity_counter speedometer_right = { .cache_ = counters::cache{ counters::g_cnt_a0 }, .motor_ = motor_right };

  driver::steering_blocking drive{
    .l_ = motor_left,
    .r_ = motor_right,
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

  // beeper::stop();
  // lcd.clear();

  // symulator samochodzika
  // device::button buttie_left = {
  //   .input_pin = pins::with_mode(pins::sensor_left, INPUT_PULLUP),
  // };
  // digitalWrite(pins::sensor_left, HIGH);
  // device::button buttie_right = {
  //   .input_pin = pins::with_mode(pins::sensor_right, INPUT_PULLUP),
  // };
  // digitalWrite(pins::sensor_right, HIGH);

  motor_left.set_power(255);
  motor_right.set_power(255);

  {
    // test connectorow
    motor_left.forward();
    motor_right.backward();
    delay(1000);
    if (sensor_left.current() == 0 && sensor_right.current() == 0) {
      panic();
    }
  }
  for (;;) {

    // program rozgladania sie
    const auto measurements = tower.measure();
    auto furthest_away = [&measurements]() -> astd::pair<int16_t, uint16_t> {
      auto _iter = astd::max_element(measurements.begin(), measurements.end(), [](decltype(measurements[0]) const& l, decltype(measurements[0]) const& r) {
        if (l.second == device::sonar::error_reason::too_far) {
          return false;
        }

        if (r.second == device::sonar::error_reason::too_far) {
          return true;
        }

        return l.first < r.first;
      });
      uint16_t _index = _iter - measurements.begin();
      uint16_t _direction = driver::constants::_positions[_index];  // left is 180 right is 0

      int16_t direction = static_cast<int16_t>(_direction) - 90;  // left is 90 right is -90
      uint16_t distance = measurements[_index].first;
      if (config::enable_logs) {
        Serial.print("decision: {");
        Serial.print(direction);
        Serial.print(", ");
        Serial.print(distance);
        Serial.print("}\n");
      }
      return { direction, distance };
    };
    auto d = furthest_away();
    auto& direction = d.first;
    auto& distance = d.second;
    // delay(500);  // TODO test czy mozna bez
    drive.rotate(direction);
    // drive.forward(distance * 0.9);
    drive.forward(10);
    // delay(500);  // allow car to stabilize its position
    speedometer_left.probe();
    speedometer_right.probe();
    {
      double l = speedometer_left.current();
      double r = speedometer_right.current();
      char text_left[16] = { 0 };
      char text_right[16] = { 0 };
      dtostrf(l, 3, 2, text_left);
      dtostrf(r, 3, 2, text_right);
      // FIXME: make wrapper
      lcd.lcd_.clear();
      lcd.lcd_.setCursor(0, 0);
      lcd.lcd_.print(text_left);
      lcd.lcd_.setCursor(0, 1);
      lcd.lcd_.print(text_right);
    }

    {
      // drive.forward(40);
      // delay(500);
      // delay(500);
      // beeper::start(100000);
      // drive.backward(40);
      // beeper::stop();
      // delay(500);
      // drive.rotate_left(45);
      // delay(500);
      // drive.rotate_right(45);
      // delay(500);

      // test synchronizacji odleglosci
      // auto res = sonar.measure();
      // uint16_t& cm = res.first;
      // delay(2000);
      // drive.forward(cm);
      // delay(2000);
      // drive.backward(cm);
      // delay(2000);

      // test na kierunki motora
      // device::motor& m = left_motor;
      // m.forward();
      // delay(3000);
      // m.backward();
      // beeper::start(100000);
      // delay(3000);
      // m.stop();
      // beeper::stop();
      // device::motor& m = right_motor;
      // m.forward();
      // delay(3000);
      // m.backward();
      // beeper::start(100000);
      // delay(3000);
      // m.stop();
      // beeper::stop();

      // test na obrot
      // drive.rotate_left(90);
      // delay(1000);
      // drive.rotate_right(90);
      // delay(1000);
    }
  }
}

// TODO kalibracja kolek??
// TODO nie blokujace jezdzenie

// przetestuj sonar - dziala odleglosc sie zgadza
// przetestuj dokladna odleglosc jechania - zatrzymuje sie bezposrednio przed przeszkoda
// przetestuj dokladny obrot i wymysl mnoznik
// przetestuj serwo - dziala idealnie
// decydowanie kierunku
// zatrzymywanie przed przeszkoda, rozgladanie sie, podejmowanie decyzji, skrecanie, kontynuuowanie
//   - rozglada sie, podejmuje decyzje, skreca, przejezdza 0.9 odl do przeszkody