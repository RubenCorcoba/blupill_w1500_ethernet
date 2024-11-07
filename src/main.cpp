#include <Arduino.h>
#include <Ethernet.h>  // poner libreria para el w1500
constexpr int LEN_BUFFER_ADC = 16;
constexpr int LEN_BUFFER_SPI = LEN_BUFFER_ADC/2;
// Define buffers and variables
uint16_t adc_buffer[LEN_BUFFER_ADC];   // Buffer de ADC
uint8_t spi_buffer[LEN_BUFFER_SPI];    // Buffer de SPI

void ADC_DMA_Init(void);
void Ethernet_Init(void);

static EthernetClient client;

static volatile bool transmitir = false;

extern "C" void DMA1_Channel1_IRQHandler(void);

void setup(void) {

    ADC_DMA_Init();
    Ethernet_Init();
}

void loop(void){
    static byte ip_servidor[4]={192,168,1,120};
    if (transmitir){
        if(client.connect(ip_servidor,4000)){
            client.write(spi_buffer,LEN_BUFFER_SPI);
        }
        transmitir = false;
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
    DMA1_Channel1->CCR = DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_TCIE | DMA_CCR_EN;

    NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    ADC1->CR2 |= ADC_CR2_DMA | ADC_CR2_CONT;
    ADC1->CR2 |= ADC_CR2_ADON;
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
        if (!transmitir){
            // Copia los datos del buffer de ADC al buffer de SPI
            for (int i = 0; i < 16; i++) {
                spi_buffer[i * 2] = adc_buffer[i] >> 8;
                spi_buffer[i * 2 + 1] = adc_buffer[i] & 0xFF;
            }
            transmitir = true;
        }
    }
}