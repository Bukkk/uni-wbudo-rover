#include <stdint.h>

// now it is c++20, use system avr-gcc and -std=c++20 in platform.txt

namespace config {
constexpr bool enable_logs = false;
}

namespace pins {
//misc
constexpr uint8_t builtin_led = 13;

// silniki
// 11 10 9 6 5 3 orginalnie ale 9 10 sa uzywane przez timer1
// pasek lewo bialy szary folet niebieski zielony zolty prawo
constexpr uint8_t eng_1 = 6;  // bialy lewo 1.0
constexpr uint8_t eng_2 = 8;  // szary
constexpr uint8_t eng_3 = 7;  // filet
constexpr uint8_t eng_4 = 2;  // niebieski
constexpr uint8_t eng_5 = 4;  // zielony
constexpr uint8_t eng_6 = 5;  // zolty prawo 1.0

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

// ir
constexpr uint8_t ir_recive = 12;
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
  // set PCIE1 bit in PCICR to enable pin 14..8 interrupts
  PCICR = 0x02;
  // run ISR for 9 8 pins
  PCMSK1 = PCMSK1 | ((1 << PCINT8) | (1 << PCINT9));
}

ISR(PCINT1_vect) {
  if ((PINC & (1 << PC0)))
    counters::g_cnt_a0 = counters::g_cnt_a0 + 1;

  if ((PINC & (1 << PC1)))
    counters::g_cnt_a1 = counters::g_cnt_a1 + 1;
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

struct wrapper {
  volatile tick_t& ct_;

  void reset() {
    ct_ = 0;
  }

  tick_t current() {
    return ct_;
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

void start(uint32_t us = 100000) {
  digitalWrite(13, 0);
  Timer1.detachInterrupt();
  Timer1.attachInterrupt(beep, us);
}

}


namespace device {

struct tick_sensor {
  struct pinout {
    uint8_t counter_;
  };
  const pinout pins_;
  counters::cache ct_;

  void reset() {
    ct_.reset();
  }

  auto current() -> counters::tick_t {
    return ct_.current();
  }
};

struct motor {
  struct pinout {
    uint8_t enable_;
    uint8_t forward_;
    uint8_t backward_;
  };

  const pinout pins_;

  motor(pinout pin)
    : pins_(pin) {}

  enum class states : uint8_t {
    STOP = 0,
    FORWARD,
    BACKWARD,
  };

  states state_{};
  uint8_t power_{};

  void set_power(uint8_t power) {
    power_ = power;
    analogWrite(pins_.enable_, power);
  }

  void forward() {
    state_ = states::FORWARD;
    digitalWrite(pins_.backward_, LOW);
    digitalWrite(pins_.forward_, HIGH);
  }

  void backward() {
    state_ = states::BACKWARD;
    digitalWrite(pins_.forward_, LOW);
    digitalWrite(pins_.backward_, HIGH);
  }

  void stop() {
    state_ = states::STOP;
    digitalWrite(pins_.forward_, LOW);
    digitalWrite(pins_.backward_, LOW);
  }
};

// struct button {
//   const uint8_t input_pin_;
// };

struct sonar {
  static float constexpr _sound_speed_ms = 343.8;                      // m/s 20C
  static float constexpr _sound_speed_mmus = _sound_speed_ms * 0.001;  // mm/us
  static float constexpr mm_per_pulse = _sound_speed_mmus * 0.5;       // 0.5 sound travels twice the distance from sensor to obstacle
  static float constexpr cm_per_pulse = mm_per_pulse * 0.1;

  static uint16_t constexpr min_us = 150;
  static uint16_t constexpr max_us = 25000;
  static float constexpr min_cm = min_us * cm_per_pulse;
  static float constexpr max_cm = max_us * cm_per_pulse;

  struct pinout {
    uint8_t trigger_pin_;
    uint8_t echo_pin_;
  };
  const pinout pins_;

  sonar(pinout pins)
    : pins_{ pins } {}

  enum class error_reason : uint8_t {
    NONE,
    TOO_CLOSE,
    TOO_FAR
  };

  auto measure() -> astd::pair<uint16_t, error_reason> {
    digitalWrite(pins_.trigger_pin_, HIGH);
    delayMicroseconds(15);  // 10 minimum
    digitalWrite(pins_.trigger_pin_, LOW);

    // min 150us too close, max 25000 too far, 38000 no obstacle
    uint32_t pulse = pulseIn(pins_.echo_pin_, HIGH, 30000);

    if (pulse < min_us) {
      return { 0, error_reason::TOO_CLOSE };
    } else if (pulse > max_us) {
      return { 0, error_reason::TOO_FAR };
    } else {
      uint16_t meas = pulse * cm_per_pulse;
      return { meas, error_reason::NONE };
    }
  }
};

}

#include <Servo.h>
namespace device {
struct servo {
  struct pinout {
    uint8_t control_;
  };
  const pinout pins_;

private:
  Servo s_;
public:

  servo(pinout pins)
    : pins_{ pins } {
    s_.attach(pins.control_);
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

  lcd_wrapper() {
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

#include <IRremote.hpp>
namespace device {
struct ir_reciver_wrapper {

  uint32_t update_timestamp_{};

  struct pinout {
    uint8_t ir_recive_;
  };
  const pinout pins_;

  ir_reciver_wrapper(pinout pins)
    : pins_{ pins } {
    IrReceiver.begin(pins_.ir_recive_, ENABLE_LED_FEEDBACK);
  }

  bool available() {
    if (IrReceiver.decode()) {
      update_timestamp_ = millis();
      return true;
    }
    return false;
  }

  auto get() -> uint32_t {
    return IrReceiver.decodedIRData.decodedRawData;
  }

  enum button_code : uint32_t {
    sig_continue = 0,
    num_1 = 0xBA45FF00,
    num_2 = 0xB946FF00,
    num_3 = 0xB847FF00,
    num_4 = 0xBB44FF00,
    num_5 = 0xBF40FF00,
    num_6 = 0xBC43FF00,
    num_7 = 0xF807FF00,
    num_8 = 0xEA15FF00,
    num_9 = 0xF609FF00,
    num_0 = 0xE619FF00,
    sym_star = 0xE916FF00,
    sym_hash = 0xF20DFF00,
    arr_u = 0xE718FF00,
    arr_d = 0xAD52FF00,
    arr_l = 0xF708FF00,
    arr_r = 0xA55AFF00,
    sym_ok = 0xE31CFF00,
  };
};

struct virtual_pilot {
  ir_reciver_wrapper& recv_;

  virtual_pilot(ir_reciver_wrapper& recv)
    : recv_{ recv } {
  }

  enum class command : uint8_t {
    stop = 0,
    forward,
    backward,
    turn_left,
    turn_right,
  };

  command command_{};

  void update() {
    uint32_t since_last = millis() - recv_.update_timestamp_;

    if (since_last > 110) {
      command_ = command::stop;
    } else {
      uint32_t code = recv_.get();
      using button_code = ir_reciver_wrapper::button_code;
      switch (code) {
        case button_code::arr_u:
          {
            command_ = command::forward;
          }
          break;
        case button_code::arr_d:
          {
            command_ = command::backward;
          }
          break;
        case button_code::arr_l:
          {
            command_ = command::turn_left;
          }
          break;
        case button_code::arr_r:
          {
            command_ = command::turn_right;
          }
          break;
        case button_code::sig_continue:
          {
            // command_ = command_;
          }
          break;
        default:
          {
            if constexpr (config::enable_logs) {
              Serial.println("virtual_pilot::update{unsupported button_code}");
            }
            command_ = command::stop;
          }
      }
    }
  }
};
}

namespace driver {

struct simple_steering {
  const device::motor& l_;
  const device::motor& r_;

private:
  void _go(void (device::motor::*l_action)(), void (device::motor::*r_action)()) {

  }
public:
  void forward() {
    _go(&device::motor::forward, &device::motor::forward);
  }

  void backward() {
    _go(&device::motor::backward, &device::motor::backward);
  }

  void stop() {
    _go(&device::motor::stop, &device::motor::stop);
  }

  void rotate_left() {
    _go(&device::motor::backward, &device::motor::forward);
  }

  void rotate_right() {
    _go(&device::motor::forward, &device::motor::backward);
  }

  // TODO relaxed rotate and so on left stoped right forward
};

struct controlled_steering {
  const driver::simple_steering steering_;

  const device::tick_sensor& ls_;
  const device::tick_sensor& rs_;

  static constexpr float tick_to_cm = (40.) / 21.;
  static constexpr float deg_to_cm = 0.3;

  void _go(void (driver::simple_steering::*action)(), uint16_t cm) {
    ls_.reset();
    rs_.reset();

    (steering_.*action)();

    counters::tick_t ticks = cm * tick_to_cm;

    bool lb = true;
    bool rb = true;
    while (lb || rb) {
      if (lb && ls_.current() >= ticks) {
        steering_.l_.stop();
        lb = false;
      }
      if (rb && rs_.current() >= ticks) {
        steering_.r_.stop();
        rb = false;
      }

      // FIXME if counter is badly connected, it loops forever
    }

    if constexpr (config::enable_logs) {
      Serial.print("l counter: ");
      Serial.print(ls_.current());
      Serial.print("r counter: ");
      Serial.print(rs_.current());
      Serial.print("\n");
    }
  }

  void forward(uint16_t cm) {
    _go(&driver::simple_steering::forward, cm);
  }

  void backward(uint16_t cm) {
    _go(&driver::simple_steering::backward, cm);
  }

  void rotate_left(uint16_t deg) {
    _go(&driver::simple_steering::rotate_left, deg * deg_to_cm);
  }

  void rotate_right(uint16_t deg) {
    _go(&driver::simple_steering::rotate_right, deg * deg_to_cm);
  }

  void rotate(int16_t deg_to_left) {
    if (deg_to_left >= 0) {
      rotate_left(static_cast<uint16_t>(deg_to_left));
    } else {
      rotate_right(static_cast<uint16_t>(-deg_to_left));
    }
  }
};

struct sonar_tower_blocking {
  device::sonar& sonar_;
  device::servo& servo_;

  constexpr static astd::array<uint16_t, 5> _positions = { 180u, 135u, 90u, 45u, 0u };

  auto measure() -> astd::array<decltype(sonar_.measure()), _positions.size()> {
    decltype(measure()) results{};

    for (uint8_t i{}; i < _positions.size(); ++i) {
      uint8_t deg = _positions[i];
      servo_.set_angle(deg);
      delay(300);  // time to allow servo to rotate
      auto m = sonar_.measure();
      results[i] = m;
    }

    servo_.reset_position();

    if constexpr (config::enable_logs) {
      for (uint8_t i{}; i < _positions.size(); ++i) {
        Serial.print("angle=");
        Serial.print(_positions[i]);
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
    auto cms = cache_.current() * device::sonar::cm_per_pulse;
    s_.add(cms);
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

  // beeper::start(100000);
  // lcd.uwu_message();

  if constexpr (config::enable_logs) {
    Serial.begin(9600);
    while (!Serial)
      ;
  }


  counters::init();
  device::tick_sensor sensor_left(
    { .counter_ = pins::with_mode(pins::counter_a0, INPUT) },
    counters::cache{ counters::g_cnt_a0 });
  device::tick_sensor sensor_right(
    { .counter_ = pins::with_mode(pins::counter_a1, INPUT) },
    counters::cache{ counters::g_cnt_a1 });
  device::motor motor_left({
    .enable_ = pins::with_mode(pins::eng_1, OUTPUT),
    .forward_ = pins::with_mode(pins::eng_2, OUTPUT),
    .backward_ = pins::with_mode(pins::eng_3, OUTPUT),
  });
  device::motor motor_right({
    .enable_ = pins::with_mode(pins::eng_6, OUTPUT),
    .forward_ = pins::with_mode(pins::eng_5, OUTPUT),
    .backward_ = pins::with_mode(pins::eng_4, OUTPUT),
  });
  driver::velocity_counter speedometer_left(counters::cache{ counters::g_cnt_a0 }, motor_left);
  driver::velocity_counter speedometer_right(counters::cache{ counters::g_cnt_a1 }, motor_right);

  driver::simple_steering steering{
    .l_ = motor_left,
    .r_ = motor_right,
  };
  driver::controlled_steering drive{
    .steering_ = steering,
    .ls_ = sensor_left,
    .rs_ = sensor_right,
  };

  device::sonar sonar({ .trigger_pin_ = pins::with_mode(pins::sonar_trigger, OUTPUT),
                        .echo_pin_ = pins::with_mode(pins::sonar_echo, INPUT) });
  device::servo servo({
    .control_ = pins::with_mode(pins::servo_control, OUTPUT),
  });
  driver::sonar_tower_blocking tower{
    .sonar_ = sonar,
    .servo_ = servo
  };

  device::ir_reciver_wrapper recv({ .ir_recive_ = pins::with_mode(pins::ir_recive, INPUT) });
  device::virtual_pilot pilot(recv);

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

  // {
  //   // connector test
  //   lcd.lcd_.clear();
  //   lcd.lcd_.setCursor(0, 0);
  //   lcd.lcd_.print("test czujnikow");

  //   motor_left.forward();
  //   motor_right.backward();

  //   delay(800);

  //   motor_left.stop();
  //   motor_right.stop();

  //   if (sensor_left.current() == 0 && sensor_right.current() == 0) {
  //     lcd.lcd_.setCursor(0, 1);
  //     lcd.lcd_.print("failed");
  //     panic();
  //   } else {
  //     lcd.lcd_.setCursor(0, 1);
  //     lcd.lcd_.print("ok");
  //   }

  //   delay(1000);
  //   lcd.clear();
  // }

  // {
  //   // test obrotu
  //   lcd.lcd_.clear();
  //   lcd.lcd_.setCursor(0, 0);
  //   lcd.lcd_.print("test obrotu");

  //   drive.rotate_left(90);
  //   delay(1000);
  //   drive.rotate_right(90);

  //   lcd.lcd_.setCursor(0, 1);
  //   lcd.lcd_.print("???");
  //   delay(1000);

  //   lcd.clear();
  // }

  for (;;) {

    {
      // bardzo prosty program poruszania sie za pilotem na liste 7
      using command = device::virtual_pilot::command;
      switch (pilot.command_) {
        case command::forward: {
          steering.forward();
        }break;
        case command::backward: {
          steering.backward();
        }break;
        case command::turn_left: {
          drive.rotate_left(45);
        }break;
        case command::turn_right: {
          drive.rotate_right(45);
        }break;
        case command::stop: {
          steering.stop();
        }break;
        default: {
          steering.stop();
          beeper::start();
          delay(1000);
          beeper::stop();
          if constexpr (config::enable_logs) {
            Serial.println("main::l7{not supported command}");
          }
        }
      }
    }

    // {  // program rozgladania sie na liste 4 i 5
    //   const auto measurements = tower.measure();

    //   auto furthest_away = [&measurements]() -> astd::pair<int16_t, uint16_t> {
    //     auto _iter = astd::max_element(
    //       measurements.begin(),
    //       measurements.end(),
    //       [](decltype(measurements[0]) const& l, decltype(measurements[0]) const& r) {
    //         if (l.second == device::sonar::error_reason::TOO_FAR) {
    //           return false;
    //         }

    //         if (r.second == device::sonar::error_reason::TOO_FAR) {
    //           return true;
    //         }

    //         return l.first < r.first;
    //       });

    //     uint16_t _index = _iter - measurements.begin();
    //     int16_t direction = static_cast<int16_t>(driver::sonar_tower_blocking::_positions[_index]) - 90;
    //     uint16_t distance = measurements[_index].first;

    //     if constexpr (config::enable_logs) {
    //       Serial.print("decision: {");
    //       Serial.print(direction);
    //       Serial.print(", ");
    //       Serial.print(distance);
    //       Serial.print("}\n");
    //     }

    //     return { direction, distance };
    //   };
    //   auto d = furthest_away();

    //   // delay(500);
    //   auto& direction = d.first;
    //   auto& distance = d.second;
    //   drive.rotate(direction);
    //   // drive.forward(distance * 0.9);
    //   drive.forward(10);
    //   // delay(500);

    //   speedometer_left.probe();
    //   speedometer_right.probe();
    //   {
    //     // ticks per ms to ticks per s
    //     double l = speedometer_left.current() * 1000;
    //     double r = speedometer_right.current() * 1000;

    //     // FIXME: make wrapper
    //     lcd.lcd_.clear();
    //     {
    //       char text[16] = { 0 };
    //       dtostrf(l, 3, 2, text);
    //       lcd.lcd_.setCursor(0, 0);
    //       lcd.lcd_.print(text);
    //     }
    //     {
    //       char text[16] = { 0 };
    //       dtostrf(r, 3, 2, text);
    //       lcd.lcd_.setCursor(0, 1);
    //       lcd.lcd_.print(text);
    //     }
    //     {
    //       char text[16] = { 0 };
    //       dtostrf(double(distance), 3, 2, text);
    //       lcd.lcd_.setCursor(8, 0);
    //       lcd.lcd_.print(text);
    //     }
    //     {
    //       char text[16] = { 0 };
    //       dtostrf(double(direction), 3, 2, text);
    //       lcd.lcd_.setCursor(8, 1);
    //       lcd.lcd_.print(text);
    //     }
    //   }
    // }

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
// przetestuj dokladny obrot i wymysl mnoznik - jest ok ale pewnie zalezy od samochodzika
// przetestuj serwo - dziala idealnie
// decydowanie kierunku
// zatrzymywanie przed przeszkoda, rozgladanie sie, podejmowanie decyzji, skrecanie, kontynuuowanie
//   - rozglada sie, podejmuje decyzje, skreca, przejezdza 0.9 odl do przeszkody

// przetestuj wyswietlanie speedometra na w programie z rozgladaniem
// przetestuj jezdzenie na pilocie