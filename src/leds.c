#include "stm32f10x_lib.h"
#include "leds.h"
#include "i2c.h"

// --- Beacon ---

// Setup beacon for PWM pulse
void beacon_init(void) {
	// Set timer clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	// Configure TIM3
	TIM_TimeBaseInitTypeDef timerInitStructure;
	timerInitStructure.TIM_Prescaler = 720;
	timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM_CounterMode_Up;
	timerInitStructure.TIM_Period = 256;
	timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM3, &timerInitStructure);
	
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_Cmd(TIM3, ENABLE);
	
	// Configure PB0 as alternate output push-pull for beacon
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	// Configure Channel 3
	TIM3->CCMR2 = 0x0064; // PWM1 mode
	TIM3->CCER = 0x0100; // enabled, positive polarity
	TIM3->CCR3 = 0;
}

// Step beacon state
void beacon_step(void) {
	static uint32_t beacon_duty = 0;
	static int beacon_sign = 1;
	
	beacon_duty += beacon_sign;
	if ( beacon_duty <= 0 || beacon_duty >= 256 ) {
		beacon_sign *= -1;
	}
	
	TIM3->CCR3 = beacon_duty * beacon_duty / 256;
}

// --- Heartbeat LED ---

void hbled_init(void) {
	// Configure PC13 as output push-pull for onboard LED
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIOC, GPIO_Pin_13);
}

// Toggle the state of the LED
void hbled_toggle(int decim) {
	static int state = 0;
	static int counter = 0;
	
	counter++;
	if ( counter < decim ) {
		return;
	}
	
	if ( state ) {
		GPIO_SetBits(GPIOC, GPIO_Pin_13);
	} else {
		GPIO_ResetBits(GPIOC, GPIO_Pin_13);
	}
	state = !state;
	counter = 0;
}

// --- RGB panels ---

void rgb_init(void) {
	// Set timer clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	// Configure TIM1
	TIM1_TimeBaseInitTypeDef timerInitStructure;
	timerInitStructure.TIM1_Prescaler = 720;
	timerInitStructure.TIM1_CounterMode = TIM1_CounterMode_Up; //TIM_CounterMode_Up;
	timerInitStructure.TIM1_Period = 256;
	timerInitStructure.TIM1_ClockDivision = TIM1_CKD_DIV1;
	TIM1_TimeBaseInit(&timerInitStructure);
	
	TIM1_OC1PreloadConfig(TIM1_OCPreload_Enable);
	TIM1_OC2PreloadConfig(TIM1_OCPreload_Enable);
	TIM1_OC3PreloadConfig(TIM1_OCPreload_Enable);
	TIM1_Cmd(ENABLE);
	TIM1_CtrlPWMOutputs(ENABLE);
	
	// Configure PB13..15 as alternate push-pull
	// RED: 14, GREEN: 15, BLUE: 13
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	// Configure Channel 1..3
	TIM1->CCMR1 = 0x6464; // PWM1 mode
	TIM1->CCMR2 = 0x0064;
	TIM1->CCER = 0x0444; // enabled, positive polarity on complementary outputs
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;
}

void rgb_set(uint32_t r, uint32_t g, uint32_t b) {
	b = 0.8 * b;
	g = 0.55 * g;
	TIM1->CCR1 = b * b / 256;
	TIM1->CCR2 = r * r / 256;
	TIM1->CCR3 = g * g / 256;
}

// --- LED stripe ---

static uint32_t stripe_state = 0x00;

void stripe_init(void) {
	// Init I2C separately!
	
	// Init stripe outputs
	i2c_write(0x22, 0x00, 0x00);
	i2c_write(0x22, 0x01, 0x00);
	
	i2c_write(0x23, 0x00, 0x00);
	i2c_write(0x23, 0x01, 0x00);
}

void stripe_set(uint32_t state) {
	stripe_state = state;
}

void stripe_set_level(int level) {
	stripe_state = 0;
	
	stripe_state |= ( level > 1 ) << 9;
	stripe_state |= ( level > 2 ) << 8;
	stripe_state |= ( level > 5 ) << 7;
	stripe_state |= ( level > 10 ) << 6;
	stripe_state |= ( level > 21 ) << 5;
	stripe_state |= ( level > 45 ) << 4;
	stripe_state |= ( level > 97 ) << 3;
	stripe_state |= ( level > 208 ) << 2;
	stripe_state |= ( level > 446 ) << 1;
	stripe_state |= ( level > 955 ) << 0;
	/*
	stripe_state |= ( level > 186 ) << 9;
	stripe_state |= ( level > 372 ) << 8;
	stripe_state |= ( level > 559 ) << 7;
	stripe_state |= ( level > 745) << 6;
	stripe_state |= ( level > 931 ) << 5;
	stripe_state |= ( level > 1117 ) << 4;
	stripe_state |= ( level > 1303 ) << 3;
	stripe_state |= ( level > 1490 ) << 2;
	stripe_state |= ( level > 1676 ) << 1;
	stripe_state |= ( level > 1862 ) << 0;
	*/
}

void stripe_out(void) {
	i2c_write(0x22, 0x12, stripe_state & 0xff);
	i2c_write(0x22, 0x13, ( stripe_state & 0x0700 ) >> 8);
	i2c_write(0x23, 0x12, stripe_state & 0xff);
	i2c_write(0x23, 0x13, ( stripe_state & 0x700 ) >> 8);
}

void stripe_scroll_step(void) {
	stripe_state >>= 1;
	if ( !stripe_state ) {
		stripe_state = 0x0400;
	}
}
