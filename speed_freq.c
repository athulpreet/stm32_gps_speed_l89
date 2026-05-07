#include "stm32f10x.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// --- Prototypes ---
void Clock_Init_72MHz(void);
void UART1_Init_115200(void);
void UART2_Init_9600(void);   // Set to 9600
void TIM2_PWM_Init(void);
void UART1_SendString(char* str);
void UART2_SendChar(char c);
void UART2_SendString(char* str);
void ProcessGPSByte(char c);
void ParseRMC(char* rmc);
void Set_PWM_Frequency(uint32_t speed);

// --- Globals ---
#define RX_BUF_SIZE 128
char rx_buffer[RX_BUF_SIZE];
int rx_index = 0;
int global_speed = 0;
int last_applied_speed = -1;

void Delay_ms(uint32_t ms) {
    for(volatile uint32_t i = 0; i < ms * 8000; i++);
}

int main(void) {
    Clock_Init_72MHz();
    UART1_Init_115200(); 
    UART2_Init_9600();   // Communication with external device at 9600
    TIM2_PWM_Init();     // PWM on PA0

    Delay_ms(2000); 

    // Configure L89 using the $PAIR commands you verified
    UART1_SendString("$PAIR050,200*21\r\n");            // 5Hz
    Delay_ms(100);
    UART1_SendString("$PAIR062,0,0*3E\r\n");            // GGA Off
    UART1_SendString("$PAIR062,1,0*3F\r\n");            // GLL Off
    UART1_SendString("$PAIR062,2,0*3C\r\n");            // GSA Off
    UART1_SendString("$PAIR062,3,0*3D\r\n");            // GSV Off
    UART1_SendString("$PAIR062,4,1*39\r\n");            // RMC ON
    UART1_SendString("$PAIR062,5,0*3B\r\n");            // VTG Off
    UART1_SendString("$PAIR513*3D\r\n");                // Save
    Delay_ms(500);

    while (1) {
        // 1. Handle Incoming GPS Data
        if (USART1->SR & USART_SR_RXNE) {
            char c = (char)USART1->DR;
            ProcessGPSByte(c); 
        }

        // 2. Handle PWM Update
        if (global_speed != last_applied_speed) {
            Set_PWM_Frequency(global_speed);
            last_applied_speed = global_speed;
        }
    }
}

void ProcessGPSByte(char c) {
    if (c == '$') {
        rx_index = 0;
    }
    if (rx_index < RX_BUF_SIZE - 1) {
        rx_buffer[rx_index++] = c;
        if (c == '\n') {
            rx_buffer[rx_index] = '\0';
            // Check for RMC (can be $GPRMC or $GNRMC)
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
    int speed_to_output = 0;

    for (int i = 0; rmc[i] != '\0'; i++) {
        if (rmc[i] == ',') {
            comma_count++;
            if (comma_count == 2) status_ptr = &rmc[i+1]; 
            if (comma_count == 7) speed_ptr = &rmc[i+1];  
        }
    }

    if (status_ptr != NULL && *status_ptr == 'A') {
        float knots = atof(speed_ptr);
        float kmh = knots * 1.852;
        kmh = (kmh * 1.05) + 1.2; // Calibration
        speed_to_output = (kmh < 1.5) ? 0 : (int)(kmh + 0.5);
    } else {
        speed_to_output = 0; // GPS Not Fixed
    }

    // Output formatted HEX to UART2
    char hex_msg[10];
    sprintf(hex_msg, "%03X\n", speed_to_output);
    UART2_SendString(hex_msg);

    global_speed = speed_to_output;
}

// --- PWM Logic ---
void TIM2_PWM_Init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    GPIOA->CRL &= ~(0xF << 0);
    GPIOA->CRL |= (0xB << 0); // PA0 AF PP
    TIM2->PSC = 720 - 1;      // 100kHz
    TIM2->CCMR1 |= 0x60;      // PWM Mode 1
    TIM2->CCER |= 0x01;
    TIM2->CR1 |= 0x01;
}

void Set_PWM_Frequency(uint32_t speed) {
    if (speed < 1) {
        TIM2->CCR1 = 0; // Frequency 0 / Off
    } else {
        if (speed < 10) speed = 10;
        if (speed > 200) speed = 200;
        uint32_t arr = 100000 / speed;
        TIM2->ARR = arr;
        TIM2->CCR1 = arr / 2; // 50% Duty
    }
}

// --- Drivers ---
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
    GPIOA->CRH |= (0x4B << 4); 
    USART1->BRR = 72000000 / 115200; 
    USART1->CR1 = (USART_CR1_UE | USART_CR1_TE | USART_CR1_RE);
}

void UART2_Init_9600(void) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    GPIOA->CRL &= ~(0xFF << 8);
    GPIOA->CRL |= (0x4B << 8); 
    USART2->BRR = 36000000 / 9600; // 3750 (0xEA6)
    USART2->CR1 = (USART_CR1_UE | USART_CR1_TE | USART_CR1_RE);
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
