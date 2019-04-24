#include "utils.h"

void RCC_Configuration(void) {
	/* RCC system reset(for debug purpose) */
	RCC_DeInit();
	/* Enable HSE */
	RCC_HSEConfig(RCC_HSE_ON);
	/* Wait till HSE is ready */
	ErrorStatus HSEStartUpStatus = RCC_WaitForHSEStartUp();

	if(HSEStartUpStatus == SUCCESS) {
		/* Enable Prefetch Buffer */
		FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

		/* Flash 2 wait state */
		FLASH_SetLatency(FLASH_Latency_2);
 	
		/* HCLK = SYSCLK */
		RCC_HCLKConfig(RCC_SYSCLK_Div1); 
	
		/* PCLK2 = HCLK */
		RCC_PCLK2Config(RCC_HCLK_Div1); 

		/* PCLK1 = HCLK/2 */
		RCC_PCLK1Config(RCC_HCLK_Div2);

		/* PLLCLK = 8MHz * 9 = 72 MHz */
		RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);

		/* Enable PLL */ 
		RCC_PLLCmd(ENABLE);

		/* Wait till PLL is ready */
		while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);

		/* Select PLL as system clock source */
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

		/* Wait till PLL is used as system clock source */
		while(RCC_GetSYSCLKSource() != 0x08);
	}
}

void NVIC_Configuration(void) {
#ifdef	VECT_TAB_RAM	
	/* Set the Vector Table base location at 0x20000000 */ 
	NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
#else	/* VECT_TAB_FLASH	*/
	/* Set the Vector Table base location at 0x08005000 */ 
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x5000);
#endif
}

void delay_ms(vu32 nCount) {
	nCount *= 1200;
	for(; nCount != 0; nCount--) {
		asm volatile ("nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; ");
	}
}

void delay_us(vu32 nCount) {
	for ( nCount *= 6; nCount != 0; nCount-- );
}
