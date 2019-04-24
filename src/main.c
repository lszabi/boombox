#include "stm32f10x_lib.h"
#include "utils.h"
#include "leds.h"
#include "serial.h"
#include "i2c.h"
#include "dsp.h"
#include <stdint.h>
#include <stdlib.h>


int main(void) {
	// Configure the system clocks
	RCC_Configuration();
	NVIC_Configuration();
	
	// Enable GPIOC and GPIOB clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

	// Init peripherials
	hbled_init();
	beacon_init();
	rgb_init();
	
	i2c_init();
	stripe_init();
	
	amp_init();
	
	uint32_t state = 0;
	//int i2c_count = 0;
	
	while ( 1 ) {
		
		/*
		if ( i2c_count >= 40 ) {
			stripe_scroll_step();
			stripe_out();
			i2c_count = 0;
		}
		i2c_count++;
		*/
		
		stripe_out();
		
		beacon_step();
		hbled_toggle(150);
		
		uint32_t cl = state & 0xff;
		if ( state / 256 == 0 ) {
			rgb_set(255 - cl, cl, 0);
		} else if ( state / 256 == 1 ) {
			rgb_set(0, 255 - cl, cl);
		} else if ( state / 256 == 2 ) {
			rgb_set(cl, 0, 255 - cl);
		} else {
			state = 0;
		}
		state++;
		
		delay_ms(10);		
	}
	
	
}