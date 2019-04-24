#include "stm32f10x_lib.h"
#include "utils.h"
#include "serial.h"
#include <stdint.h>
#include <stdlib.h>

#define PWM_BITS 8

#define ADC_BUFFER_SIZE 25

uint16_t adc_values[ADC_BUFFER_SIZE];
uint16_t adc_values_idx = 0;

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
	timerInitStructure.TIM_Period = 1 << PWM_BITS;
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
	//TIM4->CCER = 0x11; // both active high
	
}

void pwm_out(uint32_t pulse_a, uint32_t pulse_b) {
	TIM4->CCR1 = pulse_a;
	TIM4->CCR2 = pulse_b;
}

void pwm_a(uint32_t pulse) {
	// Configure PB6 as alternate function, PB7 as output push-pull
	GPIOB->CRL &= 0x00ffffff;
	GPIOB->CRL |= 0x3b000000;
	GPIOB->BSRR = 0x00800000;
	// PWM on PB6 (Timer 4 Channel 1)
	TIM4->CCR1 = pulse;
	TIM4->CCMR1 = 0x0064;
	TIM4->CCER = 0x01;
}

void pwm_b(uint32_t pulse) {
	// Configure PB7 as alternate function, PB6 as output push-pull
	GPIOB->CRL &= 0x00ffffff;
	GPIOB->CRL |= 0xb3000000;
	GPIOB->BSRR = 0x00400000;
	// PWM on PB7 (Timer 4 Channel 2)
	TIM4->CCR2 = pulse;
	TIM4->CCMR1 = 0x6400;
	TIM4->CCER = 0x10;
}

uint32_t pwm_get_duty(uint32_t b) {
	return b * b * ( 1.0f / (1 << PWM_BITS) );
}

uint16_t adc_get_average(void) {
	uint32_t avg = 0;
	for ( uint32_t i = 0; i < ADC_BUFFER_SIZE; i++ ) {
		avg += adc_values[i];
	}
	return avg / ADC_BUFFER_SIZE;
}

void TIM3_IRQHandler(void) {
	if ( TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET ) {
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
		uint16_t adc_value = ADC_GetConversionValue(ADC1);
		//adc_values[adc_values_idx++] = adc_value;
		//adc_values_idx %= ADC_BUFFER_SIZE;
		//int16_t value = adc_value - 0x07ff;
		//value /= 2;
		//float volume = 0.5f;
		//value *= volume;
		/*
		if ( value >= 0 ) {
			//Usart1Printf("%d\r\n", value);
			pwm_a(pwm_get_duty(value));
		} else {
			//Usart1Printf("-%d\r\n", -value);
			pwm_b(pwm_get_duty(-value));
		}
		*/
		//Usart1Printf("%d\r\n", adc_value);
		uint32_t val = (int)(( adc_value >> 4 ) * 1.5);
		pwm_out(val + 20, val - 20);
		//pwm_out(128, 128);
	}
}

void i2c_init()
{
    // Initialization struct
    I2C_InitTypeDef I2C_InitStruct;
    GPIO_InitTypeDef GPIO_InitStruct;

    // Step 1: Initialize I2C
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    I2C_InitStruct.I2C_ClockSpeed = 100000;
    I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStruct.I2C_OwnAddress1 = 0x00;
    I2C_InitStruct.I2C_Ack = I2C_Ack_Disable;
    I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2C1, &I2C_InitStruct);
    I2C_Cmd(I2C1, ENABLE);
 
    // Step 2: Initialize GPIO as open drain alternate function
    //RCC_APB2PeriphClockCmd(I2C_GPIO_RCC, ENABLE);
	
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void i2c_start()
{
    // Wait until I2C1 is not busy anymore
    while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));

    // Generate start condition
    I2C_GenerateSTART(I2C1, ENABLE);

    // Wait for I2C EV5. 
    // It means that the start condition has been correctly released 
    // on the I2C bus (the bus is free, no other devices is communicating))
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
}

void i2c_stop()
{
    // Generate I2C stop condition
    I2C_GenerateSTOP(I2C1, ENABLE);
    // Wait until I2C stop condition is finished
    while (I2C_GetFlagStatus(I2C1, I2C_FLAG_STOPF));
}

void i2c_address_direction(uint8_t address, uint8_t direction)
{
    // Send slave address
    I2C_Send7bitAddress(I2C1, address, direction);

    // Wait for I2C EV6
    // It means that a slave acknowledges his address
    if (direction == I2C_Direction_Transmitter) {
        while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    } else if (direction == I2C_Direction_Receiver) { 
        while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
    }
}

void i2c_transmit(uint8_t byte)
{
    // Send data byte
    I2C_SendData(I2C1, byte);
    // Wait for I2C EV8_2.
    // It means that the data has been physically shifted out and 
    // output on the bus)
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
}

uint8_t i2c_receive_ack()
{
    // Enable ACK of received data
    I2C_AcknowledgeConfig(I2C1, ENABLE);
    // Wait for I2C EV7
    // It means that the data has been received in I2C data register
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));

    // Read and return data byte from I2C data register
    return I2C_ReceiveData(I2C1);
}

uint8_t i2c_receive_nack()
{
    // Disable ACK of received data
    I2C_AcknowledgeConfig(I2C1, DISABLE);
    // Wait for I2C EV7
    // It means that the data has been received in I2C data register
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));

    // Read and return data byte from I2C data register
    return I2C_ReceiveData(I2C1);
}

void i2c_write(uint8_t address, uint8_t data)
{
    i2c_start();
    i2c_address_direction(address << 1, I2C_Direction_Transmitter);
    i2c_transmit(data);
    i2c_stop();
}

void i2c_write_w(uint8_t address, uint16_t data)
{
    i2c_start();
    i2c_address_direction(address << 1, I2C_Direction_Transmitter);
    i2c_transmit(data >> 8);
	i2c_transmit(data & 0xff);
    i2c_stop();
}

void i2c_read(uint8_t address, uint8_t* data)
{
    i2c_start();
    i2c_address_direction(address << 1, I2C_Direction_Receiver);
    *data = i2c_receive_nack();
    i2c_stop();
}

int main(void) {
	// Configure the system clocks
	RCC_Configuration();
	NVIC_Configuration();
	// Enable GPIOC and GPIOB clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

	// Configure PC.13 as Output push-pull
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// Init peripherials
	//Usart1Init();
	adc_init();
	pwm_init();
	timer_init();
	//i2c_init();
	
	//delay(0x1000);
	//i2c_write_w(0x22, 0x0000);
	
	/*
	// Test h-bridge
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	*/
	
	//GPIO_ResetBits(GPIOB, GPIO_Pin_6);
	//GPIO_ResetBits(GPIOB, GPIO_Pin_7);
	
	//GPIO_ResetBits(GPIOB, GPIO_Pin_6);
	//GPIO_ResetBits(GPIOB, GPIO_Pin_7);
	
	
	/*
	int count = 0;
	int sign = 1;
	*/
	
	GPIO_SetBits(GPIOC, GPIO_Pin_13);
	while ( 1 ) {
		/*
		count += sign;
		if ( count > 0 ) {
			pwm_a(pwm_get_duty((uint32_t)count));
		} else {
			pwm_b(pwm_get_duty((uint32_t)(-count)));
		}
		if ( count == (MAX_VOL) || count == -(MAX_VOL) ) {
			sign *= -1;
		}
		*/
		
		GPIO_ResetBits(GPIOC, GPIO_Pin_13);
		GPIO_ResetBits(GPIOB, GPIO_Pin_0);
		//GPIO_ResetBits(GPIOB, GPIO_Pin_15);
		//GPIO_ResetBits(GPIOB, GPIO_Pin_6);
		//GPIO_SetBits(GPIOB, GPIO_Pin_7);
		//i2c_write_w(0x22, 0x12ff);
		delay(0xc0000);
		GPIO_SetBits(GPIOC, GPIO_Pin_13);
		GPIO_SetBits(GPIOB, GPIO_Pin_0);
		//GPIO_SetBits(GPIOB, GPIO_Pin_15);
		//GPIO_SetBits(GPIOB, GPIO_Pin_6);
		//GPIO_ResetBits(GPIOB, GPIO_Pin_7);
		//i2c_write_w(0x22, 0x1200);
		delay(0xc0000);
		
		//Delay(0x05);
		/*
		uint16_t adc_val = ADC2Get();
		Usart1Put(adc_val >> 4);
		//Usart1Put(( adc_val >> 8 ) & 0x0f);
		*/
		/*
		GPIO_ResetBits(GPIOC, GPIO_Pin_13);
		delay(0x20000);
		GPIO_SetBits(GPIOC, GPIO_Pin_13);
		delay(0x200000);
		*/
	}
}
