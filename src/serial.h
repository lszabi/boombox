#ifndef SERIAL_H
#define SERIAL_H

#include <stdint.h>

void Usart1Init(void);
void Usart1Put(uint8_t);
uint8_t Usart1Get(void);
uint8_t Usart1Read(void);

void Usart1Puts(char *);
void Usart1PutDec(uint32_t);
void Usart1PutHex(uint32_t);
void Usart1Printf(const char *, ...);

#endif