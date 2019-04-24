#include "stm32f10x_lib.h"
#include "serial.h"
#include <stdarg.h>

void Usart1Init(void) {
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	//enable bus clocks
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	
	//Set USART1 Tx (PA.09) as AF push-pull
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//Set USART1 Rx (PA.10) as input floating
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	
	//Write USART1 parameters
	USART_Init(USART1, &USART_InitStructure);
	//Enable USART1
	USART_Cmd(USART1, ENABLE);
}

void Usart1Put(uint8_t ch) {
	USART_SendData(USART1, (uint8_t) ch);
	//Loop until the end of transmission
	while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
}

uint8_t Usart1Get(void){
	while ( USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);
	return (uint8_t)USART_ReceiveData(USART1);
}

uint8_t Usart1Read(void){
	if ( USART_GetFlagStatus(USART1, USART_FLAG_RXNE) != RESET ) {
		return (uint8_t)USART_ReceiveData(USART1);
	}
	return 0;
}

void Usart1Puts(char *str) {
	while ( *str ) {
		Usart1Put((uint8_t)(*str++));
	}
}

void Usart1PutDec(uint32_t num) {
	int i = 0;
	char buf[10];
	do {
		buf[i++] = '0' + ( num % 10 );
		num /= 10;
	} while ( num );
	do {
		Usart1Put(buf[--i]);
	} while ( i );
}

void Usart1PutHex(uint32_t num) {
	int i;
	char buf[8];
	for ( i = 0; i < 8; i++ ) {
		buf[i] = num & 0x0F;
		if ( buf[i] > 9 ) {
			buf[i] += 'a' - 10;
		} else {
			buf[i] += '0';
		}
		num >>= 4;
	}
	Usart1Puts("0x");
	do {
		Usart1Put(buf[--i]);
	} while ( i );
}

void Usart1Printf(const char *str, ...) {
	va_list ap;
	va_start(ap, str);
	while ( *str ) {
		if ( *str == '%' ) {
			str++;
			if ( *str == '%' ) {
				Usart1Put('%');
			} else if ( *str == 'c' ) {
				uint8_t c = va_arg(ap, int);
				Usart1Put(c);
			} else if ( *str == 's' ) {
				char *c = va_arg(ap, char *);
				Usart1Puts(c);
			} else if ( *str == 'd' || *str == 'u' ) {
				uint32_t n = va_arg(ap, uint32_t);
				Usart1PutDec(n);
			} else if ( *str == 'x' ) {
				uint32_t n = va_arg(ap, uint32_t);
				Usart1PutHex(n);
			} else {
				Usart1Put('?');
			}
		} else {
			Usart1Put(*str);
		}
		str++;
	}
	va_end(ap);
}
