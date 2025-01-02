#ifndef ENCODER_H_
#define ENCODER_H_
#include "stdint.h"


void encoder_init(uint16_t max_value);
uint16_t encoder_read(void);

#endif /* ENCODER_H_ */