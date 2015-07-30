#pragma once

#include <avr/pgmspace.h>
// https://gcc.gnu.org/onlinedocs/gcc/Named-Address-Spaces.html
// http://saaadhu.github.io/appnote.html
// https://sourceware.org/ml/binutils/2012-12/msg00151.html

extern unsigned short const ramplen;
extern uint16_t const __flash ramp[];
