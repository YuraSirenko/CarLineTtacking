#ifndef LF_HAL_H
#define LF_HAL_H

#include <stdint.h>

void hal_init(void);
void hal_read_line_sensors(uint16_t *raw, int count);
void hal_set_wheel_speeds(float left_mps, float right_mps);
uint32_t hal_micros(void);

#endif
