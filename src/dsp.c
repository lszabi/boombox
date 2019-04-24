#include "dsp.h"
#include "stm32f10x_lib.h"
#include "leds.h"

#define ADC_BUFFER_SIZE 25

uint16_t adc_values[ADC_BUFFER_SIZE];
uint16_t adc_values_idx = 0;

int absval(int a ) {
	return a > 0 ? a : -a;
}

void adc_init(void) {
	// Configure PA2 as analog input
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// enable ADC1 clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	
	// ADC1 configuration
	ADC_InitTypeDef ADC_InitStructure;
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	ADC_Init(ADC1, &ADC_InitStructure);
	
	// ADC1 channel2 configuration
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_55Cycles5);
	
	// Enable ADC1
	ADC_Cmd(ADC1, ENABLE);
	
	// ADC1 reset calibration
	ADC_ResetCalibration(ADC1);
	while ( ADC_GetResetCalibrationStatus(ADC1) );
	ADC_StartCalibration(ADC1);
	while ( ADC_GetCalibrationStatus(ADC1) );
	
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

void timer_init(void) {
	// Set timer clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
	// Configure TIM3
	// Clock division 2 with prescaler 75 at 72Mhz = 48kHz
	TIM_TimeBaseInitTypeDef timerInitStructure;
	timerInitStructure.TIM_Prescaler = 75;
	timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	timerInitStructure.TIM_Period = 10;
	timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV2;
	TIM_TimeBaseInit(TIM3, &timerInitStructure);
	
	// Configure TIM3 interrupt
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM3, ENABLE);
}

void pwm_init(void) {
	// Set timer clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	// Configure TIM4
	TIM_TimeBaseInitTypeDef timerInitStructure;
	timerInitStructure.TIM_Prescaler = 1;
	timerInitStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned3; //TIM_CounterMode_Up;
	timerInitStructure.TIM_Period = 256;
	timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM4, &timerInitStructure);
	
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_Cmd(TIM4, ENABLE);
	
	// Configure PB6 and PB7 as alternate function push-pull
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	// Configure pwm out
	TIM4->CCMR1 = 0x6464;
	TIM4->CCER = 0x13; // complementer active high-low
}

void TIM3_IRQHandler(void) {
	if ( TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET ) {
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
		uint32_t adc_value = ADC_GetConversionValue(ADC1);
		
		uint8_t pwm_out = adc_value >> 4;
		TIM4->CCR1 = pwm_out;
		TIM4->CCR2 = pwm_out;
		
		stripe_set_level(absval(adc_value - (1 << 11)));
	}
}

void amp_init(void) {
	adc_init();
	timer_init();
	pwm_init();
}
