#include "stm32f10x.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// --- Prototypes ---
void Clock_Init_72MHz(void);
void UART1_Init_115200(void);
void UART1_DMA_Init(void);        
void UART2_Init_9600(void);
void UART2_DMA_Init(void); 
void TIM2_PWM_Init(void);

void UART1_SendString(char* str);
void UART2_SendDMA(char* str);

void ProcessGPSByte(char c);
void ParseRMC(char* rmc);
void Set_PWM_Frequency(uint32_t speed);

// --- Globals ---
#define RX_BUF_SIZE 128
char rx_buffer[RX_BUF_SIZE];
int rx_index = 0;
int global_speed = 0;

// Non-blocking flag for UART2
volatile uint8_t uart2_busy = 0;

// --- DMA Globals ---
#define DMA_RX_BUF_SIZE 256
uint8_t dma_rx_buffer[DMA_RX_BUF_SIZE];
uint16_t dma_read_idx = 0;

char tx2_buffer[32];

// --- DMA1 Channel 7 Interrupt Handler (UART2 TX) ---
void DMA1_Channel7_IRQHandler(void) {
    // Check if Transfer Complete Interrupt flag is set
    if (DMA1->ISR & DMA_ISR_TCIF7) {
        DMA1->IFCR = DMA_IFCR_CTCIF7; // Clear the flag
        uart2_busy = 0;               // Mark UART as ready for next packet
    }
}

void Delay_ms(uint32_t ms) {
    for(volatile uint32_t i = 0; i < ms * 8000; i++);
}

int main(void) {
    // 1. Hardware Initialization
    Clock_Init_72MHz();
    UART1_Init_115200(); 
    UART1_DMA_Init();    
    UART2_Init_9600(); 
    UART2_DMA_Init();
    TIM2_PWM_Init();     

    Delay_ms(2000); // Wait for GPS stable

    // 2. Configure GPS Module (Startup Commands)
    UART1_SendString("$PAIR050,200*21\r\n");            // Set 5Hz update rate
    Delay_ms(100);
    UART1_SendString("$PAIR062,0,0*3E\r\n");            // GGA Off
    UART1_SendString("$PAIR062,1,0*3F\r\n");            // GLL Off
    UART1_SendString("$PAIR062,2,0*3C\r\n");            // GSA Off
    UART1_SendString("$PAIR062,3,0*3D\r\n");            // GSV Off
    UART1_SendString("$PAIR062,4,1*39\r\n");            // RMC ON (Primary Data)
    UART1_SendString("$PAIR062,5,0*3B\r\n");            // VTG Off
    UART1_SendString("$PAIR513*3D\r\n");                // Save settings
    Delay_ms(500);

    while (1) {
        // 3. Constant Monitoring of GPS Data via DMA Circular Buffer
        uint16_t dma_write_idx = DMA_RX_BUF_SIZE - DMA1_Channel5->CNDTR;
        
        while (dma_read_idx != dma_write_idx) {
            ProcessGPSByte((char)dma_rx_buffer[dma_read_idx]); 
            if (++dma_read_idx >= DMA_RX_BUF_SIZE) dma_read_idx = 0;
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

    for (int i = 0; rmc[i] != '\0'; i++) {
        if (rmc[i] == ',') {
            comma_count++;
            if (comma_count == 2) status_ptr = &rmc[i+1]; 
            if (comma_count == 7) speed_ptr = &rmc[i+1];  
        }
    }

    if (status_ptr != NULL && *status_ptr == 'A') {
        float kmh = atof(speed_ptr) * 1.852;
        kmh = (kmh * 1.05) + 1.2;
        // Logic for 1km/h to 200km/h
        global_speed = (kmh < 10.0) ? 0 : (int)(kmh + 0.5);
    } else {
        global_speed = 0;
    }

    // --- INSTANT UPDATE ---
    Set_PWM_Frequency(global_speed);

    // Prepare Speed String (Hex format) for UART2
    sprintf(tx2_buffer, "%03X\n", global_speed);
    UART2_SendDMA(tx2_buffer);
}

void Set_PWM_Frequency(uint32_t speed) {
	
	speed=33;
    // If speed is below threshold (0-9 km/h), kill the output
    if (speed < 10) {
        TIM2->CCR1 = 0; 
    } else {
        // Clamp maximum speed to 200 km/h
        if (speed > 200) speed = 200;
        
        // Calculate ARR based on 100kHz clock source
        uint32_t arr = 100000 / speed; 
        
        // Write to Shadow Registers. 
        // The hardware will switch to the new frequency only at the 
        // end of the current cycle, ensuring a stable signal.
        TIM2->ARR = arr - 1;
        TIM2->CCR1 = arr / 2; // 50% Duty Cycle
    }
}

// --- Initialization & Drivers ---

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

void TIM2_PWM_Init(void) {
    // 1. Enable Clocks
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // 2. Configure PA0 as Alternate Function Push-Pull
    GPIOA->CRL &= ~(0xF << 0);
    GPIOA->CRL |= (0xB << 0); 

    // 3. Set Timer Prescaler (72MHz / 720 = 100kHz clock)
    TIM2->PSC = 720 - 1;      

    // 4. Initialize with 0 (Output OFF)
    // This prevents the 100Hz startup glitch entirely.
    TIM2->ARR = 1000 - 1; 
    TIM2->CCR1 = 0; 

    // 5. Configure PWM Mode & Enable Shadow Registers (Preload)
    // OC1PE (Bit 3) ensures frequency changes are smooth and don't flicker.
    TIM2->CCMR1 |= (0x6 << 4) | (1 << 3);
    
    // 6. Enable Auto-Reload Preload (ARPE - Bit 7)
    // This is the fix for the 28Hz-32Hz fluctuation.
    TIM2->CR1 |= (1 << 7); 

    // 7. Enable Output and Start Timer
    TIM2->CCER |= 0x01;
    TIM2->CR1 |= 0x01;
}

void UART1_Init_115200(void) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_USART1EN;
    GPIOA->CRH &= ~(0xFF << 4);
    GPIOA->CRH |= (0x4B << 4); 
    USART1->BRR = 72000000 / 115200; 
    USART1->CR1 = (USART_CR1_UE | USART_CR1_TE | USART_CR1_RE);
}

void UART1_DMA_Init(void) {
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
    DMA1_Channel5->CCR = 0;
    DMA1_Channel5->CPAR = (uint32_t)&(USART1->DR);
    DMA1_Channel5->CMAR = (uint32_t)dma_rx_buffer;
    DMA1_Channel5->CNDTR = DMA_RX_BUF_SIZE;
    DMA1_Channel5->CCR |= (1 << 7) | (1 << 5); // MINC and CIRC
    DMA1_Channel5->CCR |= (1 << 0);            
    USART1->CR3 |= USART_CR3_DMAR;
}

void UART2_Init_9600(void) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    GPIOA->CRL &= ~(0xFF << 8);
    GPIOA->CRL |= (0x4B << 8); 
    USART2->BRR = 36000000 / 9600; 
    USART2->CR1 = (USART_CR1_UE | USART_CR1_TE | USART_CR1_RE);
}

void UART2_DMA_Init(void) {
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
    DMA1_Channel7->CCR = 0;
    DMA1_Channel7->CPAR = (uint32_t)&(USART2->DR);
    DMA1_Channel7->CMAR = (uint32_t)tx2_buffer;
    
    // MINC(7), DIR(4), and Transfer Complete Interrupt Enable(1)
    DMA1_Channel7->CCR |= (1 << 7) | (1 << 4) | (1 << 1); 
    
    // Enable the Interrupt in the NVIC controller
    NVIC_SetPriority(DMA1_Channel7_IRQn, 1);
    NVIC_EnableIRQ(DMA1_Channel7_IRQn);
    
    USART2->CR3 |= USART_CR3_DMAT; 
}

void UART1_SendString(char* str) {
    while(*str) {
        while (!(USART1->SR & USART_SR_TXE));
        USART1->DR = *str++;
    }
}

void UART2_SendDMA(char* str) {
    // If a previous transfer is still in progress, skip this one
    // This prevents the CPU from ever waiting/blocking
    if (uart2_busy) return; 

    uart2_busy = 1;
    DMA1_Channel7->CCR &= ~(1 << 0); // Disable Channel
    DMA1->IFCR |= (0xF << 24);       // Clear Channel 7 flags
    DMA1_Channel7->CNDTR = strlen(str);
    DMA1_Channel7->CCR |= (1 << 0);  // Re-enable Channel
}
