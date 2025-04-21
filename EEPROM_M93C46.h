#ifndef SRC_EEPROM_M93_H_
	#define SRC_EEPROM_M93_H_
	#include "stm32f334x8.h"
	#include "stdlib.h"

	#define HIGH 1 // BIT MASK - HIGH STATE
	#define LOW 0 // BIT MASK - LOW STATE
	#define GPIOB_MODER_ALTFCN_B3 (0b10 << 2) // BIT MASK - ALTERNATE FUNCTION ON PB3 PIN
	#define GPIOB_AFR_TIM2CH2_B3 (1 << 4) // BIT MASK - TIMER2 CHANEL2 ON PB3 PIN
	#define TIM2_CCMR1_OC2M_PWMMODE2 0b111 << 12 // BIT MASK - PWM MODE 2 ON CHANEL 2 OF TIMER2
	#define GPIOA_MODER_ALTFCN_PA2 0x20 // BIT MASK - ALTERNATE FUNCTION ON PA2 PIN
	#define GPIOA_MODER_ALTFCN_PA3 0x80 // BIT MASK - ALTERNATE FUNCTION ON PA3 PIN
	#define GPIOA_AFR_TX_PA2 0x700 // BIT MASK - TX ON PA2
	#define GPIOA_AFR_RX_PA3 0x7000 // BIT MASK - RX ON PA3
	#define USART_BRR 0xD05 // 9600 BAUD FOR 32MHz CLOCK
	#define MemoryBytes 128 // NUMBER OF BYTES IN THE MEMORY
	#define wordLength 8 // LENGTH OF A WORD IN MEMORY

	extern const char *entryText;

	typedef struct{ // MEMORY OBJECT
		GPIO_TypeDef *GPIOAddress; // ADDRESS OF GPIO PORT USED FOR PINS
		short D; // DATA TO MEMORY PIN
		short Q; // DATA FROM MEMORY PIN
		short S; // SELECT PIN
		uint32_t frequency; // CLOCK FREQUENCY
		uint8_t Address; // ADDRESS IN THE MEMORY
	}EEPROM_M93_TypeDef;

	extern EEPROM_M93_TypeDef memory1;

	void EEPROM_M93_config(EEPROM_M93_TypeDef *Memory);
	void USART_config(void);
#endif
