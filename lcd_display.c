#include <lpc17xx.h>

// Definición de pines según tu hardware
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

int main() {
    char data[] = "12345"; // Datos a mostrar

    SystemInit();  // Inicializa el sistema
    SysTick_Configuration(); // Configura el temporizador

    // Configura los pines de datos y control del LCD como salida
    LPC_GPIO0->FIODIR |= LCD_dataBusMask;
    LPC_GPIO2->FIODIR |= LCD_ctrlBusMask;

    // Inicializa el LCD
    Lcd_CmdWrite(0x02);  // Modo 4 bits
    Lcd_CmdWrite(0x28);  // 2 líneas, matriz 5x7
    Lcd_CmdWrite(0x0C);  // Enciende el display, cursor apagado
    Lcd_CmdWrite(0x06);  // Incrementa el cursor
    Lcd_CmdWrite(0x01);  // Limpia el display

    // Envía datos al LCD
    for (int i = 0; data[i] != '\0'; ++i) {
        Lcd_DataWrite(data[i]);
    }

    while(1); // Bucle infinito
}

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
