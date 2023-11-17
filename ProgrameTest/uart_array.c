#include "LPC17xx.h"
#include "lpc17xx_uart.h"


void UART_config(void);
void UART_Transmit(char *string);


void transmitirSeno(void);

volatile uint32_t msTicks; // Contador para el SysTick

void SysTick_Handler(void) {
    msTicks++; // Incrementa el contador cada milisegundo
}



int main() {
    // Inicializa SysTick a 1 ms
    if (SysTick_Config(SystemCoreClock / 1000)) {
        while (1); // Captura el error
    }

    UART_config();  // Inicializa la UART

    while(1) {
    	transmitirSeno();
    	 delay_ms(3000);
    }
}

void UART_config(void) {
    // Configura los pines P0.2 (TXD) y P0.3 (RXD)
	LPC_PINCON->PINSEL0 &= ~(0xF << 4); // Limpia los bits de los pines P0.2 y P0.3
	LPC_PINCON->PINSEL0 |= (1 << 4) | (1 << 6); // Configura P0.2 como TXD0 y P0.3 como RXD0


	UART_CFG_Type UARTConfigStruct;

	    // Configure UART settings
	    UARTConfigStruct.Baud_rate = 9600;
	    UARTConfigStruct.Parity = UART_PARITY_NONE;
	    UARTConfigStruct.Databits = UART_DATABIT_8;
	    UARTConfigStruct.Stopbits = UART_STOPBIT_1;

	    // Initialize UART0 peripheral with the specified parameters
	    UART_Init(LPC_UART0, &UARTConfigStruct);

	    // Optional: Configure FIFO
	    UART_FIFO_CFG_Type FIFOCfg;
	    FIFOCfg.FIFO_DMAMode = DISABLE;
	    FIFOCfg.FIFO_Level = UART_FIFO_TRGLEV0;
	    FIFOCfg.FIFO_ResetRxBuf = ENABLE;
	    FIFOCfg.FIFO_ResetTxBuf = ENABLE;
	    UART_FIFOConfig(LPC_UART0, &FIFOCfg);

	    // Enable UART Transmit
	    UART_TxCmd(LPC_UART0, ENABLE);


}


void UART_Transmit(char *string) {
    while (*string) {
        while (!(LPC_UART0->LSR & 0x20));  // Espera hasta que THR esté vacío
        LPC_UART0->THR = *string++;         // Transmite el caracter actual y avanza
    }
}