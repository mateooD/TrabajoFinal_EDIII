/*  Trabajo Integrador Final - Sistema de lectura y control automatico de temperatura 
Created on: 17 nov. 2023
    Alumnos: Britez Fabio         
             Diaz Mateo          */
             
#include "LPC17xx.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_uart.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_gpdma.h"

#define DMA_SIZE 20 // Tamaño del buffer para 10 muestras (20 bytes)

uint8_t dmaBuffer[DMA_SIZE]; // Buffer para DMA
uint8_t dmaIndex = 0; // Índice para el buffer DMA

#define BUFFER_SIZE 5
uint16_t adcBuffer[BUFFER_SIZE];
uint8_t bufferIndex = 0;
volatile  uint16_t adcValue = 0;
#define LED_PORT LPC_GPIO0 // Suponiendo que el LED está en el puerto 0
#define LED_PIN (1 << 22)  // Suponiendo que el LED está en el pin 22

// Pines utilizados  D7=P0.26 ; D6=P0.25 ;  D5=P0.24 ; D3=P0.23; E=P2.6 ; RW=P2.5 ; RS=P2.4
#define LCD_D4     23
#define LCD_D5     24
#define LCD_D6     25
#define LCD_D7     26

#define LCD_RS     4
#define LCD_RW     5
#define LCD_EN     6


// Máscaras para configurar la dirección del bus de datos y control
#define LCD_ctrlBusMask   ((1<<LCD_RS)|(1<<LCD_RW)|(1<<LCD_EN))
#define LCD_dataBusMask   ((1<<LCD_D4)|(1<<LCD_D5)|(1<<LCD_D6)|(1<<LCD_D7))

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


	    // Habilitar interrupciones de UART
	     UART_IntConfig(LPC_UART0, UART_INTCFG_RBR, ENABLE); // Habilita interrupciones de recepción
	     NVIC_EnableIRQ(UART0_IRQn); // Habilita la interrupción del UART0 en el NVIC
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
    PinCfg.Funcnum = PINSEL_FUNC_3; // Cambio para canal 5
    PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
    PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
    PinCfg.Portnum = PINSEL_PORT_1; // Puerto 1 para canal 5
    PinCfg.Pinnum = PINSEL_PIN_31; // Pin 31 para canal 5
    PINSEL_ConfigPin(&PinCfg);

    ADC_Init(LPC_ADC, 200000);
    ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_5, ENABLE); // Cambio a canal 5
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


volatile uint32_t msTicks = 0;

void SysTick_Handler(void) {
    msTicks++;
}

void SysTick_Configuration(void) {
    SysTick_Config(SystemCoreClock / 1000); // Configura SysTick para interrumpir cada 1ms
}

void Delay_MS(uint32_t ms) {
    uint32_t startTicks = msTicks;
    while ((msTicks - startTicks) < ms) {}
}

// Funciones para manejar el LCD
void sendNibble(char nibble);
void Lcd_CmdWrite(char cmd);
void Lcd_DataWrite(char dat);


// Implementación de funciones
void sendNibble(char nibble) {
    LPC_GPIO0->FIOPIN &= ~LCD_dataBusMask;
    LPC_GPIO0->FIOPIN |= ((nibble & 0x0F) << LCD_D4);
}

void Lcd_CmdWrite(char cmd) {
    sendNibble(cmd >> 4); // Nibble superior
    LPC_GPIO2->FIOCLR = (1 << LCD_RS) | (1 << LCD_RW);
    LPC_GPIO2->FIOSET = (1 << LCD_EN);
    Delay_MS(5); // Retardo de 1 ms
    LPC_GPIO2->FIOCLR = (1 << LCD_EN);

    sendNibble(cmd); // Nibble inferior
    LPC_GPIO2->FIOCLR = (1 << LCD_RS) | (1 << LCD_RW);
    LPC_GPIO2->FIOSET = (1 << LCD_EN);
    Delay_MS(5); // Retardo de 1 ms
    LPC_GPIO2->FIOCLR = (1 << LCD_EN);
}

void Lcd_DataWrite(char dat) {
    sendNibble(dat >> 4); // Nibble superior
    LPC_GPIO2->FIOSET = (1 << LCD_RS);
    LPC_GPIO2->FIOCLR = (1 << LCD_RW);
    LPC_GPIO2->FIOSET = (1 << LCD_EN);
    Delay_MS(5); // Retardo de 1 ms
    LPC_GPIO2->FIOCLR = (1 << LCD_EN);

    sendNibble(dat); // Nibble inferior
    LPC_GPIO2->FIOSET = (1 << LCD_RS);
    LPC_GPIO2->FIOCLR = (1 << LCD_RW);
    LPC_GPIO2->FIOSET = (1 << LCD_EN);
    Delay_MS(5); // Retardo de 1 ms
    LPC_GPIO2->FIOCLR = (1 << LCD_EN);
}

// Función para convertir un valor flotante a una cadena de caracteres

void floatToString(float value, char *buffer, int decimalPlaces) {
    // Parte entera de la temperatura
    int integerPart = (int)value;

    // Manejo de números negativos
    if (value < 0) {
        integerPart *= -1;
        value *= -1;
    }

    // Parte fraccionaria
    float fractionalPart = value - integerPart;

    // Convertir la parte entera a una cadena de caracteres
    int index = 0;
    while (integerPart > 0) {
        buffer[index++] = (char)((integerPart % 10) + '0');
        integerPart /= 10;
    }

    // Revertir la cadena para que esté en el orden correcto
    for (int i = 0; i < index / 2; ++i) {
        char temp = buffer[i];
        buffer[i] = buffer[index - i - 1];
        buffer[index - i - 1] = temp;
    }

    // Agregar el punto decimal
    buffer[index++] = '.';

    // Convertir la parte fraccionaria a una cadena de caracteres
    for (int i = 0; i < decimalPlaces; ++i) {
        fractionalPart *= 10;
        int digit = (int)fractionalPart;
        buffer[index++] = (char)(digit + '0');
        fractionalPart -= digit;
    }

    buffer[index] = '\0'; // Agregar el carácter nulo al final para terminar la cadena
}


void configLCD(void)
{
	// Configura los pines de datos y control del LCD como salida
	       LPC_GPIO0->FIODIR |= LCD_dataBusMask;
	       LPC_GPIO2->FIODIR |= LCD_ctrlBusMask;

	       // Inicializa el LCD
	           Lcd_CmdWrite(0x02);  // Modo 4 bits
	           Lcd_CmdWrite(0x28);  // 2 líneas, matriz 5x7
	           Lcd_CmdWrite(0x0C);  // Enciende el display, cursor apagado
	           Lcd_CmdWrite(0x06);  // Incrementa el cursor
	           Lcd_CmdWrite(0x01);  // Limpia el display
}

void configGPIO()
{
	// Configurar P0.4 como GPIO
	    PINSEL_CFG_Type PinCfg;
	    PinCfg.Portnum = PINSEL_PORT_0;
	    PinCfg.Pinnum = LED_PIN;
	    PinCfg.Funcnum = PINSEL_FUNC_0; // GPIO function
	    PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;
	    PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
	    PINSEL_ConfigPin(&PinCfg);

	    // Configurar P0.4 como salida
	    GPIO_SetDir(0, (1 << LED_PIN), 1);
}
void TIMER0_IRQHandler(void) {
    // Verifica si es una interrupción de match
	if (TIM_GetIntStatus(LPC_TIM0, TIM_MR0_INT) == SET) {
	        ADC_StartCmd(LPC_ADC, ADC_START_NOW);
	        while (!(ADC_ChannelGetStatus(LPC_ADC, ADC_CHANNEL_5, ADC_DATA_DONE))); // Cambio a canal 5

	    adcValue = ADC_ChannelGetData(LPC_ADC, ADC_CHANNEL_5); // Cambio a canal 5        adcBuffer[bufferIndex] = adcValue;
        bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;

        uint8_t sendBuffer[2];
        sendBuffer[0] = (uint8_t)(adcValue & 0xFF);         // Byte menos significativo
        sendBuffer[1] = (uint8_t)((adcValue >> 8) & 0xFF);  // Byte más significativo

            // Envía los datos
        UART_Send(LPC_UART0, sendBuffer, sizeof(sendBuffer), BLOCKING);



        // Acumula las muestras en el buffer DMA
        dmaBuffer[dmaIndex++] = (uint8_t)(adcValue & 0xFF);
        dmaBuffer[dmaIndex++] = (uint8_t)((adcValue >> 8) & 0xFF);

        // Si se han acumulado 10 muestras, inicia la transferencia DMA
        if (dmaIndex >= DMA_SIZE) {
             dmaIndex = 0; // Resetea el índice para la próxima vez
             GPDMA_ChannelCmd(1, ENABLE); // Inicia la transferencia DMA en el canal 1
        }


        TIM_ClearIntPending(LPC_TIM0, TIM_MR0_INT); // Limpia la interrupción
    }
}

void UART0_IRQHandler(void) {
    uint8_t data;

    // Comprobar si la interrupción es por recepción de datos
    if (UART_GetIntId(LPC_UART0) & UART_IIR_INTID_RDA) {
        data = UART_ReceiveByte(LPC_UART0); // Leer el dato recibido

        switch (data) {
            case '1':
                Lcd_CmdWrite(0x01);  // Limpia el display
                break;
            case '2':
                LED_On(); // Encender el LED
                break;
            case '3':
                LED_Off(); // Apagar el LED
                break;
            default:
                // Manejar otros casos si es necesario
                break;
        }
    }
}


void config_DMA_for_UART(void) {
    GPDMA_Channel_CFG_Type GPDMACfg;

    // Configuración del canal DMA
    GPDMACfg.ChannelNum = 1; // Seleccionar un canal DMA disponible
    GPDMACfg.SrcMemAddr = (uint32_t)dmaBuffer; // Dirección del buffer de origen
    GPDMACfg.DstMemAddr = (uint32_t)&(LPC_UART0->THR); // Dirección del registro THR del UART0
    GPDMACfg.TransferSize = DMA_SIZE; // Tamaño de la transferencia
    GPDMACfg.TransferWidth = GPDMA_WIDTH_BYTE; // Transferir un byte cada vez
    GPDMACfg.TransferType = GPDMA_TRANSFERTYPE_M2P; // Memoria a periférico
    GPDMACfg.SrcConn = 0; // Sin conexión de origen
    GPDMACfg.DstConn = GPDMA_CONN_UART0_Tx; // Conexión UART0 Tx
    GPDMACfg.DMALLI = 0; // Sin Linked List Item

    // Inicializa el canal DMA con la configuración
    GPDMA_Setup(&GPDMACfg);
}

int main(void) {
    SystemInit();

    SysTick_Configuration();
    configLCD();

    config_UART();
    LED_Init();
    ADC_Config();
    init_Timer0();



    while (1) {
    	Delay_MS(1000);
		{
    	    Lcd_CmdWrite(0x01);  // Limpia el display
            char tempString[20]; // Buffer para almacenar la cadena de caracteres
            char txt[]= "Temp:";

            float temperature = adcValue*330/4096;

            if(temperature>=30)
            {
                GPIO_SetValue(0, (1 << LED_PIN));
            }else
            {
                GPIO_ClearValue(0, (1 << LED_PIN));
            }

            floatToString(temperature, tempString, 2); // Convierte la temperatura a una cadena con 2 decimales

            // Envía el texto "Temp:" al LCD
             for (int j = 0; txt[j] != '\0'; ++j) {
              Lcd_DataWrite(txt[j]);
             }

                    // Envía los datos convertidos al LCD
               for (int i = 0; tempString[i] != '\0'; ++i) {
                   Lcd_DataWrite(tempString[i]);
                }


		}
    }
}