#pragma once

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include "avr/dtostrf.h"
// #include "binary.h"
#include "itoa.h"

// #include "wiring_analog.h"
// #include "wiring_constants.h"
// #include "wiring_digital.h"
// #include "wiring_pulse.h"
// #include "wiring_shift.h"
#include "wiring_time.h"

#ifdef __cplusplus
  #include <algorithm>
  using std::min;
  using std::max;
#else // C
  #include <stdlib.h>
  #ifndef abs
    #define abs(x) ((x)>0?(x):-(x))
  #endif // abs

  #ifndef min
    #define min(a,b) ((a)<(b)?(a):(b))
  #endif // min

  #ifndef max
    #define max(a,b) ((a)>(b)?(a):(b))
  #endif // max

#endif // __cplusplus

#ifdef __cplusplus
  // #include "HardwareTimer.h"
  // #include "Tone.h"
  // #include "WCharacter.h"
  // #include "WInterrupts.h"
  // #include "WMath.h"
  #include "WSerial.h"
  #include "WString.h"
#endif // __cplusplus


#define clockCyclesPerMicrosecond() ( SystemCoreClock / 1000000L )
#define clockCyclesToMicroseconds(a) ( ((a) * 1000L) / (SystemCoreClock / 1000L) )
#define microsecondsToClockCycles(a) ( (a) * (SystemCoreClock / 1000000L) )
