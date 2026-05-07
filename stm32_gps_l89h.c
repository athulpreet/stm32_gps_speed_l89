#include "stm32f10x.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// --- Prototypes ---
void Clock_Init_72MHz(void);
void UART1_Init_115200(void);
void UART2_Init_115200(void);
void UART1_SendString(char* str);
void UART2_SendChar(char c);
void UART2_SendString(char* str);
void ProcessGPSData(char c);
void ParseRMC(char* rmc);

// --- Global Buffers ---
#define RX_BUF_SIZE 128
char rx_buffer[RX_BUF_SIZE];
int rx_index = 0;

// Delay function
void Delay_ms(uint32_t ms) {
    for(volatile uint32_t i = 0; i < ms * 8000; i++);
}

int main(void) {
    // 1. Initialize Hardware
    Clock_Init_72MHz();
    UART2_Init_115200(); // For PC Debugging
    UART1_Init_115200(); // For L89 GPS Module
    
    UART2_SendString("\r\n--- STM32 L89 5Hz Speedometer Start ---\r\n");
    Delay_ms(2000); // Wait for GPS to stabilize

    // 2. Configure GPS using working $PAIR commands
    UART2_SendString("> Configuring GPS for 5Hz...\r\n");
    
    // Set 5Hz Update Rate (200ms)
    UART1_SendString("$PAIR050,200*21\r\n"); 
    Delay_ms(200);

    // Disable unnecessary NMEA sentences
    UART1_SendString("$PAIR062,0,0*3E\r\n"); Delay_ms(100); // GGA Off
    UART1_SendString("$PAIR062,1,0*3F\r\n"); Delay_ms(100); // GLL Off
    UART1_SendString("$PAIR062,2,0*3C\r\n"); Delay_ms(100); // GSA Off
    UART1_SendString("$PAIR062,3,0*3D\r\n"); Delay_ms(100); // GSV Off
    UART1_SendString("$PAIR062,5,0*3B\r\n"); Delay_ms(100); // VTG Off
    
    // Ensure RMC is enabled at 5Hz
    UART1_SendString("$PAIR062,4,1*39\r\n"); 
    Delay_ms(100);

    // Save to Flash
    UART1_SendString("$PAIR513*3D\r\n");
    Delay_ms(500);

    UART2_SendString("=== CONFIG COMPLETE. PARSING SPEED ===\r\n");

    while (1) {
        // Main Loop: Feed GPS characters into Parser
        if (USART1->SR & USART_SR_RXNE) {
            char c = (char)USART1->DR;
            ProcessGPSData(c);
        }
    }
}

// --- Logic Functions ---

void ProcessGPSData(char c) {
    if (c == '$') {
        rx_index = 0;
    }
    if (rx_index < RX_BUF_SIZE - 1) {
        rx_buffer[rx_index++] = c;
        if (c == '\n') {
            rx_buffer[rx_index] = '\0';
            // Check for RMC sentence
            if (strstr(rx_buffer, "RMC")) {
                ParseRMC(rx_buffer);
            }
        }
    }
}

void ParseRMC(char* rmc) {
    int comma_count = 0;
    char *speed_ptr = NULL;
    char *status_ptr = NULL;
    
    // Find relevant fields by counting commas
    for (int i = 0; rmc[i] != '\0'; i++) {
        if (rmc[i] == ',') {
            comma_count++;
            if (comma_count == 2) status_ptr = &rmc[i+1]; 
            if (comma_count == 7) speed_ptr = &rmc[i+1];  
        }
    }

    // Check if GPS has a valid fix (Status 'A')
    if (status_ptr != NULL && *status_ptr == 'A') {
        float knots = atof(speed_ptr);
        float kmh = knots * 1.852;
        
        // --- CALIBRATION FORMULA ---
        kmh = (kmh * 1.05) + 1.2;
        
        // Rounding and Low Speed Filter
        int final_speed = (kmh < 1.5) ? 0 : (int)(kmh + 0.5);
        
        char msg[40];
        sprintf(msg, "SPEED: %d km/h\r\n", final_speed);
        UART2_SendString(msg);
    } else {
        UART2_SendString("SEARCHING SATELLITES...\r\n");
    }
}

// --- Driver Functions ---

void Clock_Init_72MHz(void) {
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY));
    FLASH->ACR |= FLASH_ACR_LATENCY_2;
    RCC->CFGR |= (RCC_CFGR_PLLSRC_HSE | RCC_CFGR_PLLMULL9);
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
}

void UART1_Init_115200(void) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_USART1EN;
    GPIOA->CRH &= ~(0xFF << 4);
    GPIOA->CRH |= (0x4B << 4); // PA9=TX, PA10=RX
    USART1->BRR = 0x271;       // 115200 @ 72MHz
    USART1->CR1 |= (USART_CR1_UE | USART_CR1_TE | USART_CR1_RE);
}

void UART2_Init_115200(void) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    GPIOA->CRL &= ~(0xFF << 8);
    GPIOA->CRL |= (0x4B << 8); // PA2=TX, PA3=RX
    USART2->BRR = 0x139;       // 115200 @ 36MHz
    USART2->CR1 |= (USART_CR1_UE | USART_CR1_TE | USART_CR1_RE);
}

void UART1_SendString(char* str) {
    while(*str) {
        while (!(USART1->SR & USART_SR_TXE));
        USART1->DR = *str++;
    }
}

void UART2_SendChar(char c) {
    while (!(USART2->SR & USART_SR_TXE));
    USART2->DR = c;
}

void UART2_SendString(char* str) {
    while(*str) UART2_SendChar(*str++);
}
