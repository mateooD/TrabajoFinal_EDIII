
#include "lpc17xx.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_uart.h"
#include "lpc17xx_timer.h"

#define BUFFER_SIZE 5
uint16_t adcBuffer[BUFFER_SIZE];
uint8_t bufferIndex = 0;

#define LED_PORT LPC_GPIO2 // Suponiendo que el LED está en el puerto 2
#define LED_PIN (1 << 5)  // Suponiendo que el LED está en el pin 5



void config_UART(void){
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


void LED_Init(void) {
    GPIO_SetDir(LED_PORT, LED_PIN, 1); // Configurar el pin del LED como salida
}

void LED_On(void) {
    GPIO_SetValue(LED_PORT, LED_PIN); // Encender LED
}

void LED_Off(void) {
    GPIO_ClearValue(LED_PORT, LED_PIN); // Apagar LED
}

void ADC_Config(void) {
    PINSEL_CFG_Type PinCfg;
    PinCfg.Funcnum = PINSEL_FUNC_1;
    PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
    PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
    PinCfg.Portnum = PINSEL_PORT_0;
    PinCfg.Pinnum = PINSEL_PIN_25;
    PINSEL_ConfigPin(&PinCfg);

    ADC_Init(LPC_ADC, 200000);
    ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_2, ENABLE);
    ADC_BurstCmd(LPC_ADC, DISABLE);
    ADC_StartCmd(LPC_ADC, ADC_START_ON_MAT01);
}

void init_Timer0(void) {
    TIM_TIMERCFG_Type TIM_ConfigStruct;
    TIM_MATCHCFG_Type TIM_MatchConfigStruct;

    // Configuración del timer
    TIM_ConfigStruct.PrescaleOption = TIM_PRESCALE_USVAL;
    TIM_ConfigStruct.PrescaleValue  = 1000;

    // Configuración del match
    TIM_MatchConfigStruct.MatchChannel = 0;
    TIM_MatchConfigStruct.IntOnMatch   = ENABLE;
    TIM_MatchConfigStruct.ResetOnMatch = ENABLE;
    TIM_MatchConfigStruct.StopOnMatch  = DISABLE;
    TIM_MatchConfigStruct.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
    TIM_MatchConfigStruct.MatchValue   = 2000; // 2 segundos

    // Inicializa el Timer0
    TIM_Init(LPC_TIM0, TIM_TIMER_MODE, &TIM_ConfigStruct);
    TIM_ConfigMatch(LPC_TIM0, &TIM_MatchConfigStruct);

    // Habilita la interrupción del Timer0
    NVIC_EnableIRQ(TIMER0_IRQn);
    TIM_Cmd(LPC_TIM0, ENABLE);
}

void TIMER0_IRQHandler(void) {
    // Verifica si es una interrupción de match
    if (TIM_GetIntStatus(LPC_TIM0, TIM_MR0_INT) == SET) {
        ADC_StartCmd(LPC_ADC, ADC_START_NOW);
        while (!(ADC_ChannelGetStatus(LPC_ADC, ADC_CHANNEL_2, ADC_DATA_DONE)));

        uint32_t adcValue = ADC_ChannelGetData(LPC_ADC, ADC_CHANNEL_2);
        adcBuffer[bufferIndex] = adcValue;
        bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;

        LED_On();
        for (volatile int i = 0; i < 100000; i++); // Pequeña espera
        LED_Off();

        UART_Send(LPC_UART0, adcBuffer[bufferIndex],sizeof(adcBuffer[bufferIndex]),BLOCKING);
        TIM_ClearIntPending(LPC_TIM0, TIM_MR0_INT); // Limpia la interrupción
    }
}

/*void SysTick_Configuration(void) {
    SysTick_Config(SystemCoreClock / 20); // 5 segundos
}*/

/*void SysTick_Handler(void) {
    ADC_StartCmd(LPC_ADC, ADC_START_NOW);
    while (!(ADC_ChannelGetStatus(LPC_ADC, ADC_CHANNEL_2, ADC_DATA_DONE)));

    uint32_t adcValue = ADC_ChannelGetData(LPC_ADC, ADC_CHANNEL_2);
    adcBuffer[bufferIndex] = adcValue;
    bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;

    LED_On();
    for (volatile int i = 0; i < 100000; i++); // Pequeña espera
    LED_Off();

    UART_Send(LPC_UART0, adcBuffer[bufferIndex],sizeof(adcBuffer[bufferIndex]),BLOCKING);
}*/

int main(void) {
    SystemInit();    
    config_UART();
    LED_Init();
    ADC_Config();
    init_Timer0();
    //SysTick_Configuration();

    while (1) {
        // Realizar otras tareas
    }
}