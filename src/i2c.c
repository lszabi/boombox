#include "stm32f10x_lib.h"
#include "i2c.h"
#include "utils.h"

#define SCL_HIGH() ( GPIOB->BSRR |= ( 1 << 8 ) )
#define SCL_LOW() ( GPIOB->BSRR |= ( 1 << ( 8 + 16 ) ) )
#define SDA_HIGH() ( GPIOB->BSRR |= ( 1 << 9 ) )
#define SDA_LOW() ( GPIOB->BSRR |= ( 1 << ( 9 + 16 ) ) )

void i2c_init(void) {
	// Configure PORTB as open drain, PB8 = SCL, PB9 = SDA
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	GPIO_SetBits(GPIOB, GPIO_Pin_8 | GPIO_Pin_9);
}

#define I2C_DELAY_US 1

// delay 4 unit
void i2c_start(void) {
	SDA_HIGH();
	delay_us(I2C_DELAY_US);
	SCL_HIGH();
	delay_us(I2C_DELAY_US);
	SDA_LOW();
	delay_us(I2C_DELAY_US);
	SCL_LOW();
	delay_us(I2C_DELAY_US);
}

// delays 3 unit
void i2c_stop(void) {
	SCL_LOW();
	SDA_LOW();
	delay_us(I2C_DELAY_US);
	SCL_HIGH();
	delay_us(I2C_DELAY_US);
	SDA_HIGH();
	delay_us(I2C_DELAY_US);
}

// delays 26 unit
void i2c_out(uint8_t data) {
	// 8 bits
	for ( uint8_t outBits = 0; outBits < 8; outBits++ ) {
		if ( data & 0x80 ) {
			SDA_HIGH();
		} else {
			SDA_LOW();
		}
	  	data <<= 1;
		
		// Generate clock for 8 data bits
		delay_us(I2C_DELAY_US);
		SCL_HIGH();
		delay_us(I2C_DELAY_US);
		SCL_LOW();
		delay_us(1);
	}
	
	// Generate clock for ACK
	SDA_HIGH();
	SCL_HIGH();
	delay_us(I2C_DELAY_US);
	
	// Wait for clock to go high, clock stretching
	//while(I2C_CLK);
	
	// Clock high, valid ACK
	//ackBit = I2C_DATA;
	SCL_LOW();
	delay_us(I2C_DELAY_US);
}

// delays 85 unit
void i2c_write(uint8_t addr, uint8_t reg, uint8_t cmd) {
	i2c_start();
	i2c_out(addr << 1);
	i2c_out(reg);
	i2c_out(cmd);
	i2c_stop();
}
