#include "Stream.h"
#pragma once

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