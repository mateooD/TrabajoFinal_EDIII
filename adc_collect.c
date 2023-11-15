#include "LPC17xx.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"

#define BUFFER_SIZE 5
uint32_t adcBuffer[BUFFER_SIZE];
uint8_t bufferIndex = 0;

#define LED_PORT LPC_GPIO2 // Suponiendo que el LED está en el puerto 2
#define LED_PIN (1 << 5)  // Suponiendo que el LED está en el pin 5

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

void SysTick_Configuration(void) {
    SysTick_Config(SystemCoreClock / 20); // 5 segundos
}

void SysTick_Handler(void) {
    ADC_StartCmd(LPC_ADC, ADC_START_NOW);
    while (!(ADC_ChannelGetStatus(LPC_ADC, ADC_CHANNEL_2, ADC_DATA_DONE)));

    uint32_t adcValue = ADC_ChannelGetData(LPC_ADC, ADC_CHANNEL_2);
    adcBuffer[bufferIndex] = adcValue;
    bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;

    LED_On();
    for (volatile int i = 0; i < 100000; i++); // Pequeña espera
    LED_Off();
}

int main(void) {
    LED_Init();
    ADC_Config();
    SysTick_Configuration();

    while (1) {
        // Realizar otras tareas
    }
}