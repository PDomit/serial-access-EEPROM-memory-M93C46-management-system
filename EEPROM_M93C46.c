#include <EEPROM_M93C46.h>


char receivedDataUART[MemoryBytes]; // TABLE TO CONTAING RECEIVING DATA 
char sentDataUART; // VARIABLE TO CONTAIN SENDING DATA
const char *entryText = "WELCOME ON THE BOARD !\nTELL ME WHAT TO DO:\nWR###...\nRD###...\nER\nWA#\nLEGEND:\nWR - WRITE DATA\nRD - READ DATA\nER - ERASE ALL DATA\nWA - WRITE ALL DATA\n# - BYTE OF DATA\n$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$\n"; // ENTRY TEXT

static void Write_Enable(EEPROM_M93_TypeDef *Memory);
static void Write_Data(EEPROM_M93_TypeDef *Memory,char *WriteData);
static uint8_t Read_Data (EEPROM_M93_TypeDef *Memory);
static void Erase_AllData(EEPROM_M93_TypeDef *Memory);
static void Write_AllData(EEPROM_M93_TypeDef *Memory, char Data);

/**
 * @brief
 *The function sets appropriate GPIO pins to be input/ output in to communicate with a memory.
 *It also configures Timer2 which is used to generate clock signal for a memory chip. Moreover it configures a pin to be
 *the clock signal.
 *
 * @param
 * EEPROM_M93_TypeDef *Memory is a pointer to a memory's object. It contains the informations necessary
 * to perform any operation such as which pins correspond to memory's inputs / output, clock frequency or
 * selected memory's address.
 *
 * @retvla
 * none
 */
void EEPROM_M93_config(EEPROM_M93_TypeDef *Memory){
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN; //ENABLING GPIOB'S CLOCK
	Memory->GPIOAddress->MODER |= (0x1 << (2*(Memory->D))) | (0x1 << (2*(Memory->S))); //SETTING D AND S PINS AS OUTPUTS
	Memory->GPIOAddress->MODER &= ~(0x3 << (2*(Memory->Q))); //SETTING Q PIN AS INPUT
	Memory->GPIOAddress->OTYPER &= ~( (0x1 << Memory->D) | (0x1 << Memory->S) ); // SETTING PUSH PULL MODE
	Memory->GPIOAddress->PUPDR &= ~( (0x3 << Memory->D) | (0x3 << Memory->S) ); // SETTING NO PULL UP/DOWN RESISTORS
	Memory->GPIOAddress->ODR &= ~( (0x1 << Memory->D) | (0x1 << Memory->S) ); // SETTING LOGICAL 0 ON THE OUTPUT PINS
	Memory->Address = 0;

	RCC->AHBENR |= RCC_AHBENR_GPIOAEN; //ENABLING GPIOA'S CLOCK
	GPIOA->MODER |= GPIOB_MODER_ALTFCN_B3; // SETTING ALTERNATE FUNCTION ON PORT B3
	GPIOA->AFR[0] |= GPIOB_AFR_TIM2CH2_B3; // SETTING PB3 AS TIMER2'S CHANNEL  FOR GENERATING CLOCK SIGNAL

	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // ENABLING TIMER2'S CLOCK
	TIM2->DIER |= TIM_DIER_UIE; //ENABLING TIMER2'S UPDATE INTERRUPT
	TIM2->CR1 |= TIM_CR1_ARPE; //ENABLING TIMER2'S AUTO RELOAD PRELOAD
	TIM2->CCMR1 |= TIM_CCMR1_OC2PE | TIM2_CCMR1_OC2M_PWMMODE2; //ENABLING OUTPUT COMPARE 2 PRELOAD AND SETTING PWM MODE 2
	TIM2->CCER |= TIM_CCER_CC2E; // ENABLING CAPTURE/COMAPRE OUTPUT2
	TIM2->PSC = (uint16_t) 0; // SETTING THE PRESCALLER VALUE
	TIM2->ARR = (uint32_t) 64000; // SETTING AUTO RELOAD REGISTER
	TIM2->CCR2 = (uint32_t) TIM2->ARR / 2; // SETTING CAPTURE/COMPARE 2 REGISTER VALUE
	TIM2->EGR |= TIM_EGR_UG; // ENABLING TIMER2'S UPDATE GENERATION
	TIM2->SR &= ~TIM_SR_UIF; // CLEARING TIMER2'S UPDATE INTERRUPT FLAG
}

/**
 * @brief
 * The function enables the possibility to perform the write operation. It is because of the memory
 * chip design.
 *
 * @param
 * EEPROM_M93_TypeDef *Memory is a pointer to a memory's struct. It contains the informations necessary
 * to perform any operation such as which pins correspond to memory's inputs / output, clock frequency or
 * selected memory's address.
 *
 * @retvla
 * none
 */
static void Write_Enable(EEPROM_M93_TypeDef *Memory){

	static uint16_t writeEnableData = 0x60; // THIS VARIABLE IS A DEMANDING FRAME OF BITS WHICH WILL BE SEND TO ENABLE WRITTING TO THE MEMORY.
	static uint8_t writeEnableCycles = 10; // NUMBER OF REQUIRED CLOCK CYCLES TO PERFORM THE WRITE ENABLE OPERATION
	Memory->GPIOAddress->ODR |= HIGH << Memory->S; // SETTING HIGH STATE ON S INPUT
	Memory->GPIOAddress->ODR |= HIGH << Memory->D; // SETTING HIGH STATE ON D INPUT
	TIM2->CR1 |= TIM_CR1_CEN; // STARTING THE CLOCK SIGNAL C
	for (int i = writeEnableCycles - 2; i >= 0  ; i-- ){ // COUNTING CLOCK CYCLES (FIRST CYCLE OCCURS WHILE ENABLING TIMER2 ABOVE)
		while (!(TIM2->SR & TIM_SR_UIF)){ // WAITING FOR THE END OF A CYCLE
			// WAIT
		}
		TIM2->SR &= ~TIM_SR_UIF; //CLEARING THE UPDATE INTERUPT FLAG
		if (((writeEnableData >> i) & 0x1) == 1){ // CHECKING IF CURRENTLY SENDING BIT IS IN HIGH STATE
			Memory->GPIOAddress->ODR |= (HIGH << Memory->D); // SETTING D INPUT IN HIGH STATE
		}
		else{
			Memory->GPIOAddress->ODR &= ~(HIGH << Memory->D); // SETTING D INPUT IN LOW STATE
		}
	}
	while (!(TIM2->SR & TIM_SR_UIF)){
		// WAIT
	}
	TIM2->CR1 &= ~TIM_CR1_CEN; // DISABLING TIMER2
	TIM2->SR &= ~TIM_SR_UIF; // CLEARING THE UPDATE INTERUPT FLAG
	TIM2->CNT = 0; // RESETTING CNT REGISTER VALUE
	Memory->GPIOAddress->ODR &= ~((HIGH << Memory->D) | (HIGH << Memory->S)); // SETTING S AND D INPUTS IN LOW STATE
}

/**
 * @brief
 * The function writes data into the memory
 *
 * @param
 * EEPROM_M93_TypeDef *Memory is a pointer to the struct of a memory into which the data will be loaded.
 * char *UserData is a pointer to a table of data.
 *
 * @retvla
 * none
 */
static void Write_Data( EEPROM_M93_TypeDef *Memory,char *UserData){
	Write_Enable(Memory);
	static uint8_t writeDataCycles = 18; // NUMBER OF REQUIRED CLOCK CYCLES TO PERFORM THE WRITE OPERATION
	uint32_t writeData = 0x8000 + (Memory->Address << 8) + *UserData;// THIS VARIABLE CONTAINS OPERATION'S OPCODE WITH A BYTE OF DATA.
	Memory->GPIOAddress->ODR  |= HIGH << Memory->S; // SETTING HIGH STATE ON S INPUT
	Memory->GPIOAddress->ODR  |= HIGH << Memory->D; // SETTING HIGH STATE ON D INPUT
	TIM2->CR1 |= TIM_CR1_CEN; // STARTING THE CLOCK SIGNAL C
	for (int i = writeDataCycles - 2; i >= 0  ; i--){ // COUNTING CLOCK CYCLES (FIRST CYCLE OCCURS WHILE ENABLING TIMER2 ABOVE)
		  if (((writeData >> i) & 0x1) == 1){ // CHECKING IF CURRENTLY SENDING BIT IS IN HIGH STATE
			  while (!(TIM2->SR & TIM_SR_UIF)){ // WAITING FOR THE END OF A CYCLE
				  // WAIT
			  }
			  Memory->GPIOAddress->ODR |= (HIGH << Memory->D); // SETTING D INPUT IN HIGH STATE
		  }
		  else{
			  while (!(TIM2->SR & TIM_SR_UIF)){ // WAITING FOR THE END OF A CYCLE
				  // WAIT
			  }

			  Memory->GPIOAddress->ODR &= ~(HIGH << Memory->D); // SETTING D INPUT IN LOW STATE
		  }
		  TIM2->SR &= ~TIM_SR_UIF; //CLEARING THE UPDATE INTERUPT FLAG
	}
	while (!(TIM2->SR & TIM_SR_UIF)){ // WAITING FOR THE END OF A CYCLE
		// WAIT
	}
	TIM2->CR1 &= ~TIM_CR1_CEN; // DISABLING TIMER2
	TIM2->SR &= ~TIM_SR_UIF; //CLEARING THE UPDATE INTERUPT FLAG
	TIM2->CNT = 0; // RESETTING CNT REGISTER VALUE
	Memory->GPIOAddress->ODR &= ~((HIGH << Memory->D) | (HIGH << Memory->S)); // SETTING S AND D INPUTS IN LOW STATE
}

/**
 * @brief
 * The function reads data from the memory. Reading starts from the value of the address stored in
 * the memory struct.
 *
 * @param
 * EEPROM_M93_TypeDef *Memory is a pointer to the struct of a memory into which the data will be loaded.
 *
 * @retvla
 * The function return uint8_t which is an ASCII code which has been read from the memory.
 */
static uint8_t Read_Data (EEPROM_M93_TypeDef *Memory){
	char receivedDataMemory = 0; // INTO THIS VARIABLE A VALUE FROM THE MEMORY WILL BE LOADED
	static uint8_t readDataCycles = 10; // NUMBER OF REQUIRED CLOCK CYCLES TO PERFORM THE READ OPERATION
	uint16_t readData = 0x100 + Memory->Address; // THIS VARIABLE CONTAINS OPERATION'S OPCODE AND AN ADDRESS
	Memory->GPIOAddress->ODR  |= HIGH << Memory->S; // SETTING HIGH STATE ON S INPUT
	Memory->GPIOAddress->ODR  |= HIGH << Memory->D; // SETTING HIGH STATE ON D INPUT
	TIM2->CR1 |= TIM_CR1_CEN; // STARTING THE CLOCK SIGNAL C
	for (int i = readDataCycles - 2; i >= 0  ; i--){ // COUNTING CLOCK CYCLES (FIRST CYCLE OCCURS WHILE ENABLING TIMER2 ABOVE)
	  if (((readData >> i) & 0x1) == 1){ // CHECKING IF CURRENTLY SENDING BIT IS IN HIGH STATE
		  while (!(TIM2->SR & TIM_SR_UIF)){ // WAITING FOR THE END OF A CYCLE
			  // WAIT
		  }
		  TIM2->SR &= ~TIM_SR_UIF; //CLEARING THE UPDATE INTERUPT FLAG
		  Memory->GPIOAddress->ODR |= (HIGH << Memory->D); // SETTING D INPUT IN HIGH STATE
	  }
	  else{
		  while (!(TIM2->SR & TIM_SR_UIF)){ // WAITING FOR THE END OF A CYCLE
			  // WAIT
		  }
		  TIM2->SR &= ~TIM_SR_UIF; //CLEARING THE UPDATE INTERUPT FLAG
		  Memory->GPIOAddress->ODR &= ~(HIGH << Memory->D); // SETTING D INPUT IN LOW STATE
	  }
	}
	while (!(TIM2->SR & TIM_SR_UIF)){  // WAITING FOR THE END OF A CYCLE
		// WAIT
	}
	TIM2->SR &= ~TIM_SR_UIF; //CLEARING THE UPDATE INTERUPT FLAG
	Memory->GPIOAddress->ODR &= ~(HIGH << Memory->D); // SETTING D INPUT IN LOW STATE
	for (int i = wordLength-1; i >= 0; i--){ // COUNTING CLOCK CYCLES WHILE RECEIVING BITS OF DATA FROM THE MEMORY
		while (!(TIM2->SR & TIM_SR_UIF)){ // WAITING FOR THE END OF A CYCLE
			// WAIT
		}
		TIM2->SR &= ~TIM_SR_UIF; //CLEARING THE UPDATE INTERUPT FLAG
		if (!(Memory->GPIOAddress->IDR & (0x1 << Memory->Q))){ // CHECKING IF CURRENTLY RECEIVING BIT IS IN LOW STATE
			receivedDataMemory &= ~(1 << i); // WRITTING RECEIVED LOW BIT INTO THE VARIABLE
		}
		else if ((Memory->GPIOAddress->IDR & (0x1 << Memory->Q))){ // CHECKING IF CURRENTLY RECEIVING BIT IS IN HIGH STATE
			receivedDataMemory |= (1 << i); // WRITTING RECEIVED HIGH BIT INTO THE VARIABLE
		}
	}
	TIM2->CR1 &= ~TIM_CR1_CEN;  // DISABLING TIMER2
	TIM2->SR &= ~TIM_SR_UIF; //CLEARING THE UPDATE INTERUPT FLAG
	TIM2->CNT = 0; // RESETTING CNT REGISTER VALUE
	Memory->GPIOAddress->ODR &= ~((HIGH << Memory->D) | (HIGH << Memory->S)); // SETTING S AND D INPUTS IN LOW STATE
	return receivedDataMemory; // RECEIVING DATA FROM THE MEMORY
}

/**
 * @brief
 * The function erases all data from the memory.
 *
 * @param
 * EEPROM_M93_TypeDef *Memory is a pointer to a memory's struct. It contains the informations necessary
 * to perform any operation such as which pins correspond to memory's inputs / output, clock frequency or
 * selected memory's address.
 *
 * @retvla
 * none
 */
static void Erase_AllData(EEPROM_M93_TypeDef *Memory){
	static uint16_t eraseAllData = 0x040; // THIS VARIABLE CONTAINS OPERATION'S OPCODE
	static uint8_t eraseAllDataCycles = 10; // NUMBER OF REQUIRED CLOCK CYCLES TO PERFORM THE ERASE OPERATION
	Memory->GPIOAddress->ODR |= HIGH << Memory->S; // SETTING HIGH STATE ON S INPUT
	Memory->GPIOAddress->ODR |= HIGH << Memory->D; // SETTING HIGH STATE ON D INPUT
	TIM2->CR1 |= TIM_CR1_CEN; // STARTING THE CLOCK SIGNAL C
	for (int i = eraseAllDataCycles - 2; i >= 0  ; i-- ){ // COUNTING CLOCK CYCLES (FIRST CYCLE OCCURS WHILE ENABLING TIMER2 ABOVE)
		while (!(TIM2->SR & TIM_SR_UIF)){ // WAITING FOR THE END OF A CYCLE
			// WAIT
		}
		TIM2->SR &= ~TIM_SR_UIF; //CLEARING THE UPDATE INTERUPT FLAG
		if (((eraseAllData >> i) & 0x1) == 1){ // CHECKING IF CURRENTLY SENDING BIT IS IN HIGH STATE
			Memory->GPIOAddress->ODR |= (HIGH << Memory->D); // SETTING D INPUT IN HIGH STATE
		}
		else{
			Memory->GPIOAddress->ODR &= ~(HIGH << Memory->D); // SETTING D INPUT IN LOW STATE
		}
	}
	while (!(TIM2->SR & TIM_SR_UIF)){ // WAITING FOR THE END OF A CYCLE
		// WAIT
	}
	TIM2->CR1 &= ~TIM_CR1_CEN; // STARTING THE CLOCK SIGNAL C
	TIM2->SR &= ~TIM_SR_UIF; //CLEARING THE UPDATE INTERUPT FLAG
	TIM2->CNT = 0; // RESETTING CNT REGISTER VALUE
	Memory->GPIOAddress->ODR &= ~((HIGH << Memory->D) | (HIGH << Memory->S)); // SETTING S AND D INPUTS IN LOW STATE
}

/**
 * @brief
 * The function writes all bytes in the memory with one byte.
 *
 * @param
 * EEPROM_M93_TypeDef *Memory is a pointer to a memory's struct. It contains the informations necessary
 * to perform any operation such as which pins correspond to memory's inputs / output, clock frequency or
 * selected memory's address.
 * char Data is a byte to be written into the memory
 *
 * @retvla
 * none
 */
static void Write_AllData(EEPROM_M93_TypeDef *Memory, char Data){
	uint16_t writeAllData = 0x02000 + Data; // THIS VARIABLE CONTAINS OPERATION'S OPCODE AND A BYTE OF DATA TO WRITE INTO THE MEMORY
	static uint8_t writeAllDataCycles = 18; // NUMBER OF REQUIRED CLOCK CYCLES TO PERFORM THE WRITE ALL DATA OPERATION
	Memory->GPIOAddress->ODR |= HIGH << Memory->S; // SETTING HIGH STATE ON S INPUT
	Memory->GPIOAddress->ODR |= HIGH << Memory->D; // SETTING HIGH STATE ON D INPUT
	TIM2->CR1 |= TIM_CR1_CEN; // STARTING THE CLOCK SIGNAL C
	for (int i = writeAllDataCycles - 2; i >= 0  ; i-- ){ // COUNTING CLOCK CYCLES (FIRST CYCLE OCCURS WHILE ENABLING TIMER2 ABOVE)
		while (!(TIM2->SR & TIM_SR_UIF)){ // WAITING FOR THE END OF A CYCLE
			// WAIT
		}
		TIM2->SR &= ~TIM_SR_UIF; //CLEARING THE UPDATE INTERUPT FLAG
		if (((writeAllData >> i) & 0x1) == 1){ // CHECKING IF CURRENTLY SENDING BIT IS IN HIGH STATE
			Memory->GPIOAddress->ODR |= (HIGH << Memory->D); // SETTING D INPUT IN HIGH STATE
		}
		else{
			Memory->GPIOAddress->ODR &= ~(HIGH << Memory->D); // SETTING D INPUT IN LOW STATE
		}
	}
	while (!(TIM2->SR & TIM_SR_UIF)){ // WAITING FOR THE END OF A CYCLE
		// WAIT
	}
	TIM2->CR1 &= ~TIM_CR1_CEN; // STARTING THE CLOCK SIGNAL C
	TIM2->SR &= ~TIM_SR_UIF; //CLEARING THE UPDATE INTERUPT FLAG
	TIM2->CNT = 0; // RESETTING CNT REGISTER VALUE
	Memory->GPIOAddress->ODR &= ~((HIGH << Memory->D) | (HIGH << Memory->S)); // SETTING S AND D INPUTS IN LOW STATE
}

/**
 * @brief
 * The function enables USART interrupt in NVIC and sets necessary bits in USART's registers
 * in order to prepare USART to work. It also sets USART's transmission speed, and pins PA2 and PA3
 * to communicate with PC via the cable.
 *
 * @param
 * none
 *
 * @retvla
 * none
 */
void USART_config(void){
	short i = 0; // VARIABLE CREATED IN ORDER TO MAKE THE INTERATION BELOW WHICH PRINTS entryText
	NVIC_EnableIRQ(USART2_IRQn); // ENABLING USART'S INTERRUPT
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN; //ENABLING GPIOA CLOCK
	GPIOA->MODER |= GPIOA_MODER_ALTFCN_PA2 |
			GPIOA_MODER_ALTFCN_PA3; // SETTING ALTERNATE FUNCTIONS ON PA2 AND PA3
	GPIOA->AFR[0] |= GPIOA_AFR_TX_PA2 | GPIOA_AFR_RX_PA3; // SETTING TX AND RX UART'S FUNCTIONS ON PA2 AND PA3
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN; // ENABLING UART2'S CLOCK
	USART2->BRR = USART_BRR; // SETTING BAUDE RATE USART'S CLOCK [32 MHz] / USART2->BRR [0xD05] = BAUDE RATE [9600 baud]
	USART2->CR1 |= USART_CR1_RXNEIE | USART_CR1_TE | USART_CR1_RE |
			USART_CR1_UE;  // ENABLING USART'S RECEIVING, TRANSMITTING, RXNE INTERRUPT AND TURNING ON THE USART
	while(entryText [i] != '\0'){ 	// PRINTING ALL THE ENTRY TEXT IN A TERMINAL
		USART2->TDR = entryText [i]; // LOADING CHAR INTO USART SENDING REGISTER
		while (!(USART2->ISR & USART_ISR_TXE)){ // WAITING UNITL THE USART SEND A BYTE
			//wait
		}
		i++; // INCREMENTING THE ITERATION VARIABLE
	}
}

/**
 * @brief
 * THE USART'S INTERRUPT OCCURS WHEN SOME DATA ARRIVES FROM A PC. INSIDE THE INTERRUPT RECEIVED BYTES ARE
 * WRITTING INTO receivedDataUART TABLE ONE BY ONE. SECONDLY COMES CHECKING WHICH TWO LETTER CODE HAS ARRIVED IN
 * ORDER TO SELECT DESIRED OPERATION. IF THERE ARE SOME BYTES AFTER THE TWO LETTER CODE (IN CASE OF WRITE,
 * OR WRITE ALL OPERATIONS) THEY ARE ALSO WRITTEN INTO receivedDataUART TABLE.
 *
 * @param
 * none
 *
 * @retvla
 * none
 */
void USART2_IRQHandler(void){
	static short i = 0; // VARIABLE TO MAKE ITERATION
	receivedDataUART[i] = (char) USART2->RDR; //READING DATA FROM USART'S RECEIVE REGISTER
	if (receivedDataUART[0] == 'W' && receivedDataUART[1] == 'R' && i > 1){ // CHECKING IF WRITE REQUESTING OCCURED
		if (receivedDataUART[i] == '\r'){ // CHECKING IF ENTER WAS PRESSED. IT MEANS USER WROTE ALL DATA HE WANTED TO SEND.
			for (int k = 2; k < i; k++ ){ // WRITTING DATA INTO THE MEMORY. FIRST BYTE IS ON THIRD POSITION IN THE TABLE
				Write_Data(&memory1, &receivedDataUART[k]); // USING FUNCTION TO WRITE THE DATA receivedDataUART[k] TO memory1
				TIM2->CR1 |= TIM_CR1_CEN; // ENABLING TIMER 2
				for(int i =0; i < 10; i++){ // GIVING THE MEMORY SOME TIME FOR INTERNAL OPERATIONS
					while (!(TIM2->SR & TIM_SR_UIF)){ // CHECKING TIMER2 UPDATE FLAG
						//wait
					}
					TIM2->SR &= ~TIM_SR_UIF; // CLEARING THE UPDATE INTERRUPT FLAG
				}
				TIM2->CR1 &= ~TIM_CR1_CEN; // DISABLING THE TIMER2
				TIM2->SR &= ~TIM_SR_UIF; // CLEARING THE UPDATE INTERRUPT FLAG
				memory1.Address += 1; // INCREMENTING MEMORY'S ADDRESS
			}
			for (int k = 0; k < 2; k++){ // CLEARING TWO LETTERS CODE FROM THE receivedDataUART TABLE
				receivedDataUART[k] = 0;
			}
			i = -1; // RESETTING INCREMENT VARIABLE
			memory1.Address = 0; // RESETTING MEMORY'S ADDRESS
		}
	}
	if (receivedDataUART[0] == 'R' && receivedDataUART[1] == 'D' && receivedDataUART[2] == '\r'){ //CHECKING IF READ REQUESTING OCCURED
		sentDataUART = Read_Data(&memory1); // READING DATA FROM THE MEMORY
		while (sentDataUART != 255 && memory1.Address < (MemoryBytes - 1)){ //CHECKING IF THE LAST ADDRESS HAS BEEN USED
			memory1.Address += 1; // INCREMENTING ADDRESS
			USART2->TDR = sentDataUART; // WRITTING DATA FROM MEMORY TO USART'S TRANSMIT REGISTER
			while (!(USART2->ISR & USART_ISR_TXE)){ // WAITING FOR THE END OF A TRANSMISSION
				// WAIT
			}
			TIM2->CR1 |= TIM_CR1_CEN; // ENABLING TIEMR2
			for(int K =0; K < 10; K++){ // GIVING THE MEMORY SOME TIME FOR INTERNAL OPERATIONS
				while ((TIM2->SR & 0x0001) != 1){ // CHECKING TIMER2 UPDATE FLAG
					// WAIT
				}
				TIM2->SR &= ~TIM_SR_UIF; // CLEARING THE UPDATE INTERRUPT FLAG
			}
			TIM2->CR1 &= ~TIM_CR1_CEN; // DISABLING TIMER2
			TIM2->SR &= ~TIM_SR_UIF; // CLEARING TIMER2'S UPDATE INTERRUPT FLAG
			sentDataUART = Read_Data(&memory1); // READING DATA FROM THE MEMORY
		}
		for (int k = 0; k < 2; k++){ // CLEARING TWO LETTERS CODE FROM THE receivedDataUART TABLE
			receivedDataUART[k] = 0;
		}
		i = -1; // RESETTING INCREMENT VARIABLE
		memory1.Address = 0; // RESETTING MEMORY'S ADDRESS
	}
	if (receivedDataUART[0] == 'E' && receivedDataUART[1] == 'R' && receivedDataUART[2] == '\r'){ //CHECKING IF ERASE REQUESTING OCCURED
		Erase_AllData(&memory1); // EREASE THE MEMORY
		for (int k = 0; k < 2; k++){ // CLEARING TWO LETTERS CODE FROM THE receivedDataUART TABLE
			receivedDataUART[k] = 0;
		}
		i = -1; // RESETTING INCREMENT VARIABLE
	}
	if (receivedDataUART[0] == 'W' && receivedDataUART[1] == 'A' && receivedDataUART[3] == '\r'){ //CHECKING IF WRITE ALL REQUESTING OCCURED
			Write_AllData(&memory1, receivedDataUART[2]); // WRITE ALL THE MEMORY WITH A BYTE
			for (int k = 0; k < 2; k++){ // CLEARING TWO LETTERS CODE FROM THE receivedDataUART TABLE
				receivedDataUART[k] = 0;
			}
			i = -1; // RESETTING INCREMENT VARIABLE
	}
	if( receivedDataUART[0] != 'W' && receivedDataUART[0] != 'R' && receivedDataUART[0] != 'E' ){ // CHECKING IF FIRST LETTER IS CORRECT
		i = -1; // RESETTING INCREMENT VARIABLE
	}
	if(i == 2 && receivedDataUART[1] != 'R' && receivedDataUART[1] != 'D' && receivedDataUART[1] != 'A'){ // CHECKING IF SECOND LETTER IS CORRECT
		i = -1; // RESETTING INCREMENT VARIABLE
	}
	i++; // INCREMENTING VARIABLE

}

