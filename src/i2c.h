#ifndef I2C_H
#define I2C_H

#include <stdint.h>

void i2c_init(void);
void i2c_start(void);
void i2c_stop(void);
void i2c_out(uint8_t);
void i2c_write(uint8_t, uint8_t, uint8_t);

#endif