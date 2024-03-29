#include <stdint.h>

// now it is c++20, use system avr-gcc and -std=c++20 in platform.txt,
// and use system compiler instead of arduino ide one

namespace config {
constexpr bool enable_logs = true;
}

/**
 * @brief hardware connections configuration
 *
 */
namespace pins {

// misc
constexpr uint8_t builtin_led = 13;

// engine controller:
// ribbon cable - left to right
constexpr uint8_t eng_1 = 6;  // white leftmost
constexpr uint8_t eng_2 = 8;  // gray
constexpr uint8_t eng_3 = 7;  // violet
constexpr uint8_t eng_4 = 4;  // blue
constexpr uint8_t eng_5 = 12; // green
constexpr uint8_t eng_6 = 5;  // yellow rightmost

// tick_sensor:
// brown vcc
// red gnd
constexpr uint8_t counter_a1 = A1; // orange right
// gray vcc
// violet gnd
constexpr uint8_t counter_a0 = A0; // blue left

// sonar:
constexpr uint8_t sonar_trigger = A2; // pin is labeled trig
constexpr uint8_t sonar_echo = A3;    // pin is labeled echo

// servo:
// red vcc
// brown gnd
constexpr uint8_t servo_control = 9; // yellow

// display:
// labeled pins sda scl vcc gnd

// ir:
// vcc
// gnd
constexpr uint8_t ir_recive = 2;
} // namespace pins

namespace pins {
uint8_t with_mode(uint8_t pin, uint8_t mode) {
  pinMode(pin, mode);
  return pin;
}
} // namespace pins

/**
 * @brief custom implementations of a few C++ standard library features
 *
 */
namespace astd {

/**
 * @brief a std pair
 *
 * @tparam T1
 * @tparam T2
 */
template <typename T1, typename T2> class pair {
public:
  T1 first;
  T2 second;

  constexpr pair() : first{}, second{} {}

  constexpr pair(const T1 &x, const T2 &y) : first{x}, second{y} {}

  template <typename U1, typename U2>
  constexpr pair(const pair<U1, U2> &p) : first(p.first), second(p.second) {}

  pair &operator=(const pair &other) {
    first = other.first;
    second = other.second;
    return *this;
  }

  constexpr bool operator==(const pair &other) const {
    return (first == other.first) && (second == other.second);
  }

  constexpr bool operator!=(const pair &other) const {
    return !(*this == other);
  }
};

template <typename T1, typename T2>
constexpr pair<T1, T2> make_pair(T1 t, T2 u) {
  return pair<T1, T2>(t, u);
}

/**
 * @brief a std array knockoff
 *
 * @tparam T
 * @tparam N
 */
template <typename T, uint8_t N> class array {
public:
  using value_type = T;
  using size_type = uint8_t;
  using reference = T &;
  using const_reference = const T &;
  using pointer = T *;
  using const_pointer = const T *;
  using iterator = pointer;
  using const_iterator = const_pointer;

  T data_[N];

  constexpr size_type size() const noexcept { return N; }

  constexpr bool empty() const noexcept { return N == 0; }

  // accessory
  reference operator[](size_type index) { return data_[index]; }

  // its gnu++11 so only const members can be constexpred
  constexpr const_reference operator[](size_type index) const {
    return data_[index];
  }

  reference front() { return data_[0]; }

  constexpr const_reference front() const { return data_[0]; }

  reference back() { return data_[N - 1]; }

  constexpr const_reference back() const { return data_[N - 1]; }

  // iteratory
  iterator begin() noexcept { return &data_[0]; }

  constexpr const_iterator begin() const noexcept { return &data_[0]; }

  iterator end() noexcept { return &data_[N]; }

  constexpr const_iterator end() const noexcept { return &data_[N]; }
};

/**
 * @brief find max element in range based on the provided relation
 *
 * @tparam ForwardIt
 * @tparam Compare
 * @param first begin of range
 * @param last end of range
 * @param less relation
 * @return ForwardIt
 */
template <typename ForwardIt, typename Compare>
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
} // namespace astd

/**
 * @brief hardware counters driver
 *
 */
namespace device::counters {
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
    device::counters::g_cnt_a0 = device::counters::g_cnt_a0 + 1;

  if ((PINC & (1 << PC1)))
    device::counters::g_cnt_a1 = device::counters::g_cnt_a1 + 1;
}

/**
 * @brief counter cache wrapper
 *
 */
struct cache {
  tick_t cache_ = {};
  volatile tick_t &ct_;

  cache(volatile tick_t &ct) : ct_(ct) {}

  void reset() { cache_ = ct_; }

  tick_t current() { return ct_ - cache_; }
};

/**
 * @brief counter newtype wrapper
 *
 */
struct wrapper {
  volatile tick_t &ct_;

  void reset() { ct_ = 0; }

  tick_t current() { return ct_; }
};
} // namespace device::counters

/**
 * @brief old beeper driver, it is disabled because timer caused problems
 *
 */
// #include <TimerOne.h>
namespace device::beeper {

// void init() {
//   // Timer1.initialize();  // is using pins 9 and 10
// }

// void beep() {
//   // auto state = digitalRead(13) ^ 1;
//   // digitalWrite(13, state);
// }

// void stop() {
//   // Timer1.detachInterrupt();
//   // digitalWrite(13, 0);
// }

// void start(uint32_t us = 100000) {
//   // digitalWrite(13, 0);
//   // Timer1.detachInterrupt();
//   // Timer1.attachInterrupt(beep, us);

} // namespace device::beeper

namespace device {

/**
 * @brief tick sensor device driver, it ticks when wheels are rotating
 *
 */
struct tick_sensor {
  struct pinout {
    uint8_t counter_;
  };
  const pinout pins_;
  counters::cache ct_;

  void reset() { ct_.reset(); }

  auto current() -> counters::tick_t { return ct_.current(); }
};

/**
 * @brief L298N motor controller device driver
 *
 */
struct motor {
  struct pinout {
    uint8_t enable_;
    uint8_t forward_;
    uint8_t backward_;
  };

  const pinout pins_;

  motor(pinout pin) : pins_(pin) {}

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

/**
 * @brief HC SR-04 ultrasound distance sensor device driver
 *
 */
struct sonar {
  static float constexpr _sound_speed_ms = 343.8;                     // m/s 20C
  static float constexpr _sound_speed_mmus = _sound_speed_ms * 0.001; // mm/us
  static float constexpr mm_per_pulse =
      _sound_speed_mmus *
      0.5; // 0.5 sound travels twice the distance from sensor to obstacle
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

  sonar(pinout pins) : pins_{pins} {}

  enum class error_reason : uint8_t { NONE, TOO_CLOSE, TOO_FAR };

  // in cm
  auto measure() -> astd::pair<uint16_t, error_reason> {
    digitalWrite(pins_.trigger_pin_, HIGH);
    delayMicroseconds(15); // 10 minimum
    digitalWrite(pins_.trigger_pin_, LOW);

    // min 150us too close, max 25000 too far, 38000 no obstacle
    uint32_t pulse = pulseIn(pins_.echo_pin_, HIGH, 30000);

    if (pulse < min_us) {
      return {0, error_reason::TOO_CLOSE};
    } else if (pulse > max_us) {
      return {0, error_reason::TOO_FAR};
    } else {
      uint16_t cm = pulse * cm_per_pulse;
      return {cm, error_reason::NONE};
    }
  }
};

} // namespace device

#include <Servo.h>
namespace device {
/**
 * @brief Servo.h driver device wrapper
 *
 */
struct servo {
  struct pinout {
    uint8_t control_;
  };
  const pinout pins_;

private:
  Servo s_;

public:
  servo(pinout pins) : pins_{pins} { s_.attach(pins.control_); }

  void set_angle(uint16_t deg) {
    if constexpr (config::enable_logs) {
      Serial.print("servo::set_angle{");
      Serial.print(deg);
      Serial.println("}");
    }
    s_.write(deg);
  }

  void reset_position() { s_.write(90); }
};
} // namespace device

#include <LiquidCrystal_I2C.h>
namespace device {

/**
 * @brief LiquidCrystal_I2C.h driver device wrapper
 *
 */
struct lcd {
  LiquidCrystal_I2C lcd_ = LiquidCrystal_I2C(0x27, 16, 2);

  lcd() {
    lcd_.init();
    lcd_.backlight();
  }

  void clear() { lcd_.clear(); }

  void uwu_message() {
    lcd_.setCursor(0, 0);
    lcd_.print("stiww pwepawing!");
    lcd_.setCursor(0, 1);
    lcd_.print("touwch my eaws:3");
  }
};
} // namespace device

#include <IRremote.hpp>
namespace device {
/**
 * @brief VS1838B ir remote device driver wrapper
 *
 */
struct ir_reciver {

  unsigned long update_timestamp_{};

  struct pinout {
    uint8_t ir_recive_;
  };
  const pinout pins_;

  ir_reciver(pinout pins) : pins_{pins} {
    IrReceiver.begin(pins_.ir_recive_, ENABLE_LED_FEEDBACK);
  }

  bool available() {
    if (IrReceiver.decode()) {
      update_timestamp_ = millis();
      IrReceiver.resume();
      return true;
    }
    return false;
  }

  auto get() -> uint32_t { return IrReceiver.decodedIRData.decodedRawData; }

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

} // namespace device

namespace facade {

/**
 * @brief abstraction of a pilot that steers the car
 *
 */
struct virtual_pilot {
  device::ir_reciver &recv_;

  virtual_pilot(device::ir_reciver &recv) : recv_{recv} {}

  /**
   * @brief state machine
   *
   */
  enum class command : uint8_t {
    stop = 0,
    forward,
    backward,
    turn_left,
    turn_right,
  };

  command command_{};

  /**
   * @brief synchronize with a state of a real pilot connected by ir device
   *
   */
  void update() {
    using button_code = device::ir_reciver::button_code;

    bool recived = recv_.available();
    uint32_t since_last = millis() - recv_.update_timestamp_;

    if (since_last > 110) {
      if constexpr (config::enable_logs) {
        Serial.println("virtual_pilot::update{timeout}");
      }
      command_ = command::stop;
    }

    if (!recived) {
      return;
    }

    // if constexpr (config::enable_logs) {
    //   Serial.println("virtual_pilot::update{}");
    // }

    if constexpr (config::enable_logs) {
      Serial.println(recv_.update_timestamp_);
      Serial.println(millis());
      Serial.println(since_last);
    }

    if (since_last > 110) {
      if constexpr (config::enable_logs) {
        Serial.println("virtual_pilot::update{recived but outdated}");
      }
      command_ = command::stop;
    } else {
      uint32_t code = recv_.get();
      switch (code) {
      case button_code::arr_u: {
        if constexpr (config::enable_logs) {
          Serial.println("virtual_pilot::update{forward}");
        }
        command_ = command::forward;
      } break;
      case button_code::arr_d: {
        if constexpr (config::enable_logs) {
          Serial.println("virtual_pilot::update{backward}");
        }
        command_ = command::backward;
      } break;
      case button_code::arr_l: {
        if constexpr (config::enable_logs) {
          Serial.println("virtual_pilot::update{left}");
        }
        command_ = command::turn_left;
      } break;
      case button_code::arr_r: {
        if constexpr (config::enable_logs) {
          Serial.println("virtual_pilot::update{right}");
        }
        command_ = command::turn_right;
      } break;
      case button_code::sig_continue: {
        // command_ = command_;
      } break;
      case button_code::sym_ok: {
        if constexpr (config::enable_logs) {
          Serial.println("virtual_pilot::update{stop}");
        }
        command_ = command::stop;
      } break;
      default: {
        if constexpr (config::enable_logs) {
          Serial.println("virtual_pilot::update{unsupported button_code}");
        }
        command_ = command::stop;
      }
      }
    }
  }
};

/**
 * @brief vehicle steering facade
 *
 */
struct steering {
  device::motor &l_;
  device::motor &r_;

private:
  void _go(void (device::motor::*l_action)(),
           void (device::motor::*r_action)()) {
    (l_.*l_action)();
    (r_.*r_action)();
  }

public:
  void forward() { _go(&device::motor::forward, &device::motor::forward); }

  void backward() { _go(&device::motor::backward, &device::motor::backward); }

  void stop() { _go(&device::motor::stop, &device::motor::stop); }

  void rotate_left() { _go(&device::motor::backward, &device::motor::forward); }

  void rotate_right() {
    _go(&device::motor::forward, &device::motor::backward);
  }

  // TODO relaxed rotate and so on left stoped right forward
};

/**
 * @brief facade for precise vehicle control using real-time rotation data
 *
 */
struct precise_steering {
  facade::steering steering_;

  device::tick_sensor &ls_;
  device::tick_sensor &rs_;

  static constexpr float tick_to_cm = (40.) / 21.;
  static constexpr float deg_to_cm = 0.3;

  void _go(void (facade::steering::*action)(), uint16_t cm) {
    ls_.reset();
    rs_.reset();

    (steering_.*action)();

    device::counters::tick_t ticks = cm * tick_to_cm;

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
      if constexpr (config::enable_logs) {
        Serial.print("precise_steering::_go::while{");
        Serial.print(ls_.current());
        Serial.print(", ");
        Serial.print(rs_.current());
        Serial.println("}");
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

  void forward(uint16_t cm) { _go(&facade::steering::forward, cm); }

  void backward(uint16_t cm) { _go(&facade::steering::backward, cm); }

  void rotate_left(uint16_t deg) {
    _go(&facade::steering::rotate_left, deg * deg_to_cm);
  }

  void rotate_right(uint16_t deg) {
    _go(&facade::steering::rotate_right, deg * deg_to_cm);
  }

  void rotate(int16_t deg_to_left) {
    if (deg_to_left >= 0) {
      rotate_left(static_cast<uint16_t>(deg_to_left));
    } else {
      rotate_right(static_cast<uint16_t>(-deg_to_left));
    }
  }
};

/**
 * @brief facade that combines servo and sonar attached to it to make some kind
 * of a radar lol :p
 *
 */
struct sonar_tower_blocking {
  device::sonar &sonar_;
  device::servo &servo_;

  // left to right
  constexpr static astd::array<uint16_t, 5> _positions = {180u, 135u, 90u, 45u,
                                                          0u};

  auto measure() -> astd::array<decltype(sonar_.measure()), _positions.size()> {
    decltype(measure()) results{};

    for (uint8_t i{}; i < _positions.size(); ++i) {
      uint8_t deg = _positions[i];
      servo_.set_angle(deg);
      delay(300); // time to allow servo to rotate
      auto m = sonar_.measure();

      if constexpr (config::enable_logs) {
        Serial.print("measurement{");
        Serial.print(m.first);
        Serial.println("}");
      }
      results[i] = m;
    }

    servo_.reset_position();

    if constexpr (config::enable_logs) {
      for (uint8_t i{}; i < _positions.size(); ++i) {
        Serial.print("measured positions{angle=");
        Serial.print(_positions[i]);
        Serial.print(", result=");
        Serial.print(results[i].first);
        Serial.println("}");
      }
    }

    return results;
  }
};

} // namespace facade

/**
 * @brief statistics helper 'thingies'
 *
 */
namespace statistics {
template <typename T, typename Time>
double calc_estimate(T now, T last, Time t_now, Time t_last) {
  auto d = now - last;
  auto dt = t_now - t_last;
  return dt != 0 ? double(d) / dt : 0.;
}

template <typename T> struct simple {
  T now, last;
  decltype(millis()) t_now, t_last;

  void meas(T n) {
    last = now;
    now = n;
    t_last = t_now;
    t_now = millis();
  }

  auto appr() -> double { return calc_estimate(now, last, t_now, t_last); }
};
} // namespace statistics

namespace facade {

/**
 * @brief measure how fast a motor spins
 *
 */
struct velocity_counter {
  device::counters::cache cache_;
  device::motor &motor_;
  statistics::simple<device::counters::tick_t> stats_;

  double current() {
    auto v = stats_.appr();
    return motor_.state_ == device::motor::states::BACKWARD ? -v : v;
  }

  void probe() {
    auto cm = cache_.current() * device::sonar::cm_per_pulse;
    stats_.meas(cm);
  }
};
} // namespace facade

/**
 * @brief panic handler - it is nice to see externally that device has crashed
 * :p
 *
 */
void panic() {
  // beeper::start();
  if constexpr (config::enable_logs) {
    Serial.println("panic{}");
  }
  for (;;)
    ;
}

/**
 * @brief this function just delays start of a program
 *
 */
void setup() {
  pinMode(pins::builtin_led, OUTPUT);
  digitalWrite(pins::builtin_led, HIGH);
  delay(2000);
  digitalWrite(pins::builtin_led, LOW);
}

/**
 * @brief main program
 *
 */
void loop() {
  // beeper::init();
  // beeper::start(200000);

  // init sequence

  if constexpr (config::enable_logs) {
    Serial.begin(9600);
    while (!Serial)
      ;

    Serial.println("init{}");
  }

  device::lcd lcd{};
  lcd.uwu_message();
  if constexpr (config::enable_logs) {
    Serial.println("lcd enabled{}");
  }

  device::counters::init();
  device::tick_sensor sensor_left(
      {.counter_ = pins::with_mode(pins::counter_a0, INPUT)},
      device::counters::cache{device::counters::g_cnt_a0});
  device::tick_sensor sensor_right(
      {.counter_ = pins::with_mode(pins::counter_a1, INPUT)},
      device::counters::cache{device::counters::g_cnt_a1});
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
  facade::velocity_counter speedometer_left(
      device::counters::cache{device::counters::g_cnt_a0}, motor_left);
  facade::velocity_counter speedometer_right(
      device::counters::cache{device::counters::g_cnt_a1}, motor_right);
  facade::steering steering{
      .l_ = motor_left,
      .r_ = motor_right,
  };
  facade::precise_steering precise_steering{
      .steering_ = steering,
      .ls_ = sensor_left,
      .rs_ = sensor_right,
  };
  if constexpr (config::enable_logs) {
    Serial.println("motors enabled{}");
  }

  device::sonar sonar(
      {.trigger_pin_ = pins::with_mode(pins::sonar_trigger, OUTPUT),
       .echo_pin_ = pins::with_mode(pins::sonar_echo, INPUT)});
  device::servo servo({
      .control_ = pins::with_mode(pins::servo_control, OUTPUT),
  });
  facade::sonar_tower_blocking tower{.sonar_ = sonar, .servo_ = servo};
  if constexpr (config::enable_logs) {
    Serial.println("tower enabled{}");
  }

  device::ir_reciver recv(
      {.ir_recive_ =
           pins::ir_recive /* pins::with_mode(pins::ir_recive, INPUT) */});
  facade::virtual_pilot pilot(recv);
  if constexpr (config::enable_logs) {
    Serial.println("ir enabled{}");
  }

  // beeper::stop();
  // lcd.clear();

  // TODO: here was supposed to be a program that tunes:
  // - makes sure all devices are connected
  // - power to motor rotation speed
  // - vehicle rotation motor tick to degrees
  // but i have never written an automatic one :p
  motor_left.set_power(255);
  motor_right.set_power(255);

  // some tests

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
  //   // rotation test
  //   lcd.lcd_.clear();
  //   lcd.lcd_.setCursor(0, 0);
  //   lcd.lcd_.print("test obrotu");

  //   precise_steering.rotate_left(90);
  //   delay(1000);
  //   precise_steering.rotate_right(90);

  //   lcd.lcd_.setCursor(0, 1);
  //   lcd.lcd_.print("???");
  //   delay(1000);

  //   lcd.clear();
  // }

  // a main program
  if constexpr (config::enable_logs) {
    Serial.println("mainloop{}");
  }
  for (;;) {
    // if constexpr (config::enable_logs) {
    //   Serial.println("loop{}");
    // }

    {
      // controll a vehicle with a pilot with 'dead hand' 'do not crash into
      // wals' thingie

      auto road_clear = [&](uint16_t angle) {
        servo.set_angle(angle);
        auto [dist, err] = sonar.measure();
        return dist > 10 && err == device::sonar::error_reason::NONE ||
               err == device::sonar::error_reason::TOO_FAR;
      };

      pilot.update();
      using command = facade::virtual_pilot::command;
      switch (pilot.command_) {
      case command::forward: {
        if (road_clear(90)) {
          steering.forward();
        } else {
          steering.stop();
        }
      } break;
      case command::backward: {
        steering.backward();
      } break;
      case command::turn_left: {
        // precise_steering.rotate_left(45);
        steering.rotate_left();
      } break;
      case command::turn_right: {
        // precise_steering.rotate_right(45);
        steering.rotate_right();
      } break;
      case command::stop: {
        steering.stop();
      } break;
      default: {
        steering.stop();
        // beeper::start();
        delay(1000);
        // beeper::stop();
        if constexpr (config::enable_logs) {
          Serial.println("main::l7{not supported command}");
        }
      }
      }

      speedometer_left.probe();
      speedometer_right.probe();
      {
        // ticks per ms to ticks per s
        double l = speedometer_left.current() * 1000;
        double r = speedometer_right.current() * 1000;

        // FIXME: make wrapper
        lcd.lcd_.clear();
        {
          char text[16] = {0};
          dtostrf(l, 3, 2, text);
          lcd.lcd_.setCursor(0, 0);
          lcd.lcd_.print(text);
        }
        {
          char text[16] = {0};
          dtostrf(r, 3, 2, text);
          lcd.lcd_.setCursor(0, 1);
          lcd.lcd_.print(text);
        }
        // {
        //   char text[16] = { 0 };
        //   dtostrf(double(distance), 3, 2, text);
        //   lcd.lcd_.setCursor(8, 0);
        //   lcd.lcd_.print(text);
        // }
      }

      delay(10);
    }

    //   {  // program to go wherever is the furtherst distance to an obstacle
    //     const auto measurements = tower.measure();

    //     auto furthest_away = [&measurements]() -> astd::pair<int16_t,
    //     uint16_t> {
    //       auto _iter = astd::max_element(
    //         measurements.begin(),
    //         measurements.end(),
    //         [](decltype(measurements[0]) const& l, decltype(measurements[0])
    //         const& r) {
    //           if (l.second == device::sonar::error_reason::TOO_FAR) {
    //             return false;
    //           }

    //           if (r.second == device::sonar::error_reason::TOO_FAR) {
    //             return true;
    //           }

    //           return l.first < r.first;
    //         });

    //       uint16_t _index = _iter - measurements.begin();
    //       int16_t direction =
    //       static_cast<int16_t>(driver::sonar_tower_blocking::_positions[_index])
    //       - 90; uint16_t distance = measurements[_index].first;

    //       if constexpr (config::enable_logs) {
    //         Serial.print("decision: {");
    //         Serial.print(direction);
    //         Serial.print(", ");
    //         Serial.print(distance);
    //         Serial.print("}\n");
    //       }

    //       return { direction, distance };
    //     };
    //     auto d = furthest_away();

    //     // delay(500);
    //     auto& direction = d.first;
    //     auto& distance = d.second;
    //     precise_steering.rotate(direction);
    //     // precise_steering.forward(distance * 0.9);
    //     precise_steering.forward(10);
    //     // delay(500);

    //     speedometer_left.probe();
    //     speedometer_right.probe();
    //     {
    //       // ticks per ms to ticks per s
    //       double l = speedometer_left.current() * 1000;
    //       double r = speedometer_right.current() * 1000;

    //       // FIXME: make wrapper
    //       // lcd.lcd_.clear();
    //       // {
    //       //   char text[16] = { 0 };
    //       //   dtostrf(l, 3, 2, text);
    //       //   lcd.lcd_.setCursor(0, 0);
    //       //   lcd.lcd_.print(text);
    //       // }
    //       // {
    //       //   char text[16] = { 0 };
    //       //   dtostrf(r, 3, 2, text);
    //       //   lcd.lcd_.setCursor(0, 1);
    //       //   lcd.lcd_.print(text);
    //       // }
    //       // {
    //       //   char text[16] = { 0 };
    //       //   dtostrf(double(distance), 3, 2, text);
    //       //   lcd.lcd_.setCursor(8, 0);
    //       //   lcd.lcd_.print(text);
    //       // }
    //       // {
    //       //   char text[16] = { 0 };
    //       //   dtostrf(double(direction), 3, 2, text);
    //       //   lcd.lcd_.setCursor(8, 1);
    //       //   lcd.lcd_.print(text);
    //       // }
    //     }
    //   }

    //   {
    //     // precise_steering.forward(40);
    //     // delay(500);
    //     // delay(500);
    //     // beeper::start(100000);
    //     // precise_steering.backward(40);
    //     // beeper::stop();
    //     // delay(500);
    //     // precise_steering.rotate_left(45);
    //     // delay(500);
    //     // precise_steering.rotate_right(45);
    //     // delay(500);

    //     // test motor ticks to distance
    //     // auto res = sonar.measure();
    //     // uint16_t& cm = res.first;
    //     // delay(2000);
    //     // precise_steering.forward(cm);
    //     // delay(2000);
    //     // precise_steering.backward(cm);
    //     // delay(2000);

    //     // test motor direction
    //     // device::motor& m = left_motor;
    //     // m.forward();
    //     // delay(3000);
    //     // m.backward();
    //     // beeper::start(100000);
    //     // delay(3000);
    //     // m.stop();
    //     // beeper::stop();
    //     // device::motor& m = right_motor;
    //     // m.forward();
    //     // delay(3000);
    //     // m.backward();
    //     // beeper::start(100000);
    //     // delay(3000);
    //     // m.stop();
    //     // beeper::stop();

    //     // test rotation ticks to degrees
    //     // precise_steering.rotate_left(90);
    //     // delay(1000);
    //     // precise_steering.rotate_right(90);
    //     // delay(1000);
    //   }
    // }
  }
}