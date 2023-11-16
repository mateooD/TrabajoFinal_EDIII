#include <lpc17xx.h>

// Definición de pines según tu hardware

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


int main() {
    float temperature = 25.5; // Ejemplo de temperatura (reemplaza con tu valor real)

    SystemInit();  // Inicializa el sistema
    SysTick_Configuration();

    // Configura los pines de datos y control del LCD como salida
    LPC_GPIO0->FIODIR |= LCD_dataBusMask;
    LPC_GPIO2->FIODIR |= LCD_ctrlBusMask;

    // Inicializa el LCD
    Lcd_CmdWrite(0x02);  // Modo 4 bits
    Lcd_CmdWrite(0x28);  // 2 líneas, matriz 5x7
    Lcd_CmdWrite(0x0C);  // Enciende el display, cursor apagado
    Lcd_CmdWrite(0x06);  // Incrementa el cursor
    Lcd_CmdWrite(0x01);  // Limpia el display

    char tempString[20]; // Buffer para almacenar la cadena de caracteres
    char txt[]= "Temp:";

    floatToString(temperature, tempString, 2); // Convierte la temperatura a una cadena con 2 decimales

    // Envía el texto "Temp:" al LCD
    for (int j = 0; txt[j] != '\0'; ++j) {
        Lcd_DataWrite(txt[j]);
    }

    // Envía los datos convertidos al LCD
    for (int i = 0; tempString[i] != '\0'; ++i) {
        Lcd_DataWrite(tempString[i]);
    }

    while(1); // Bucle infinito
}