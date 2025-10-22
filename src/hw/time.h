#ifndef TIME_H
#define TIME_H

#include <stdint.h>
#include "mcu.h"

void TIME_init();
uint32_t get_timer_val(void);

#endif // TIME_H