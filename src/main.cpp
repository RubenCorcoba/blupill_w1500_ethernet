#include <Arduino.h>
#include <Ethernet.h>  // poner libreria para el w1500

// Define buffers and variables
uint16_t adc_buffer[16];   // Buffer de ADC
uint8_t spi_buffer[32];    // Buffer de SPI

void ADC_DMA_Init(void);
void SPI_Init(void);
void Ethernet_Init(void);

extern "C" void DMA1_Channel1_IRQHandler(void);

int main(void) {
    ADC_DMA_Init();
    SPI_Init();
    Ethernet_Init();

    while (1) {
        // Procesa los datos aquí
    }
}

void ADC_DMA_Init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN | RCC_APB2ENR_IOPAEN;
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;

    GPIOA->CRL &= ~(GPIO_CRL_CNF1 | GPIO_CRL_MODE1);

    ADC1->SQR3 = 1;
    ADC1->SMPR2 = ADC_SMPR2_SMP1;
    ADC1->CR2 |= ADC_CR2_ADON;

    DMA1_Channel1->CPAR = (uint32_t)&ADC1->DR;
    DMA1_Channel1->CMAR = (uint32_t)adc_buffer;
    DMA1_Channel1->CNDTR = 16;
    DMA1_Channel1->CCR = DMA_CCR1_MINC | DMA_CCR1_CIRC | DMA_CCR1_TCIE | DMA_CCR1_EN;

    NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    ADC1->CR2 |= ADC_CR2_DMA | ADC_CR2_CONT;
    ADC1->CR2 |= ADC_CR2_ADON;
}

void SPI_Init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN | RCC_APB2ENR_IOPAEN;

    GPIOA->CRL |= GPIO_CRL_MODE5 | GPIO_CRL_CNF5_1;
    GPIOA->CRL |= GPIO_CRL_MODE6 | GPIO_CRL_CNF6_1;
    GPIOA->CRL |= GPIO_CRL_MODE7 | GPIO_CRL_CNF7_1;

    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SPE;
}

void Ethernet_Init(void) {
    // Inicializa el W5100
    static byte mac[6]={ 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
    static byte ip[4] = {192,168,1,100};
    Ethernet.begin(mac,ip);
}

void DMA1_Channel1_IRQHandler(void) {
    if (DMA1->ISR & DMA_ISR_TCIF1) {
        DMA1->IFCR |= DMA_IFCR_CTCIF1;

        // Copia los datos del buffer de ADC al buffer de SPI
        for (int i = 0; i < 16; i++) {
            spi_buffer[i * 2] = adc_buffer[i] >> 8;
            spi_buffer[i * 2 + 1] = adc_buffer[i] & 0xFF;
        }

        // Enviar datos por SPI
        for (int i = 0; i < 32; i++) {
            while (!(SPI1->SR & SPI_SR_TXE));
            SPI1->DR = spi_buffer[i];
        }

        // Enviar los datos por Ethernet
        W5100_SendData(spi_buffer, 32);
    }
}