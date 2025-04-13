#pragma once

#include "wiring.h"

/* sketch */

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus
// Weak empty variant initialization function.
// May be redefined by variant files.
// extern void initVariant() __attribute__((weak));

extern void setup(void);
extern void loop(void);

// void yield(void);
#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus