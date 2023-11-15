#include "LPC17xx.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_gpdma.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_uart.h"

#define BUFFER_SIZE 10
#define UART_BAUDRATE 9600

volatile uint32_t adcBuffer[BUFFER_SIZE];
volatile uint32_t bufferIndex = 0;


void config_UART(void) {
     // Configurar la UART para la transmisión serial
    LPC_SC->PCONP |= (1 << 3); // Habilitar el módulo UART0

    // Configurar el pin de TXD0 (P0.2)
    LPC_PINCON->PINSEL0 &= ~(0xF << 4);
    LPC_PINCON->PINSEL0 |= (1 << 4);

    // Configurar la velocidad de transmisión
    LPC_UART0->LCR = (3 << 0); // Configurar para 8 bits de datos
    LPC_UART0->DLM = 0;
    LPC_UART0->DLL = SystemCoreClock / (16 * UART_BAUDRATE);
    LPC_UART0->FDR = 0x10; // Divisor de frecuencia fraccional
    LPC_UART0->LCR = (3 << 0) | (1 << 7); // Habilitar divisor DLL/DLM y divisor de frecuencia fraccional

    // Habilitar transmisor y receptor
    LPC_UART0->FCR = (1 << 0) | (1 << 1); // Habilitar FIFO y restablecer FIFO
    LPC_UART0->TER = (1 << 7); // Habilitar transmisor

    // Configurar la interrupción de transmisión vacía (THR)
    NVIC_EnableIRQ(UART0_IRQn);
    LPC_UART0->IER = (1 << 1); // Habilitar interrupción de THR
}




void config_ADC(void) {
    // Configurar el ADC
    LPC_PINCON->PINSEL1 |= (1<<8);  //ADC0 Pin
	LPC_PINCON->PINMODE1 |= (1<<19); //Neither

    ADC_Init(LPC_ADC, 200000); // Frecuencia de muestreo de 200 kHz
    ADC_ChannelCmd(LPC_ADC, 2, ENABLE); // Configurar el canal del ADC (puerto P0.2)
}

void config_GPDMA(void) {

    NVIC_DisableIRQ(DMA_IRQn);
    
    GPDMA_Channel_CFG_Type GPDMACfg;
    
    GPDMA_Init();

    GPDMACfg.SrcMemAddr = (uint32_t) &(LPC_ADC->ADDR2);
    GPDMACfg.DstMemAddr = (uint32_t) adcBuffer;
    GPDMACfg.ChannelNum = 0;
    GPDMACfg.TransferSize = BUFFER_SIZE;
    GPDMACfg.TransferWidth = 0; // Transferencia de 32 bits
    GPDMACfg.SrcConn = GPDMA_CONN_ADC;
    GPDMACfg.DstConn = 0;
    GPDMACfg.DMALLI = 0;

    GPDMA_Setup(&GPDMACfg);

    // Habilitar interrupciones de finalización de transferencia
    GPDMA_ChannelCmd(0, ENABLE);
    NVIC_DisableIRQ(DMA_IRQn);
}

void GPDMA_IRQHandler(void) {
    if (GPDMA_IntGetStatus(GPDMA_STAT_INT, 0)) {
        // Transferencia DMA completada
        GPDMA_ClearIntPending(0, GPDMA_STATCLR_INTTC);

        // Puedes realizar acciones adicionales aquí si es necesario
    }
}

void UART0_IRQHandler(void) {
    if (LPC_UART0->LSR & (1 << 5)) {
        // Transmisor UART listo para enviar más datos
        if (bufferIndex < BUFFER_SIZE) {
            // Enviar el siguiente dato desde el búfer por UART
            LPC_UART0->THR = adcBuffer[bufferIndex++];
        } else {
            // Transmisión completa, deshabilitar la interrupción de THR
            LPC_UART0->IER &= ~(1 << 1);
        }
    }
}


int main(void) {

        LPC_GPIO0->FIODIR |= (1 << 6);   // Configurar el LED externo (P0.6) como salida

    // Configurar el ADC y el DMA
    config_ADC();

    config_UART();
    config_GPDMA();

    // Iniciar la toma de muestras (podría ser en respuesta a un evento específico)
    ADC_StartCmd(LPC_ADC, ADC_START_NOW);

    while (1) {
        // Tu programa principal puede continuar aquí
    }
}