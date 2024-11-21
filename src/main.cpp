// version 4 blupill_w5100_ethernet RDC
#include <Arduino.h>
#include <Ethernet.h>  // poner libreria para el w1500
constexpr int LEN_BUFFER_ADC = 16;
constexpr int LEN_BUFFER_SPI = LEN_BUFFER_ADC * 2;

// Define buffers and variables
uint16_t adc_buffer[LEN_BUFFER_ADC];   // Buffer de ADC
uint8_t spi_buffer[LEN_BUFFER_SPI];    // Buffer de SPI

void ADC_DMA_Init(void);
void Ethernet_Init(void);
static void transmite(void);

static EthernetClient client;

static volatile bool transmitir = false;

extern "C" void DMA1_Channel1_IRQHandler(void);

void setup(void) {
    ADC_DMA_Init();
    Ethernet_Init();
    transmitir = true;
    transmite();  // Llama a la función de transmisión una vez
}

void loop(void) {
    if (transmitir) {
        transmite();  // Llama a la función de transmisión si está habilitado
    }
}

static void transmite(void) {
    static byte ip_servidor[4] = {192, 168, 1, 120};
    static unsigned long lastTransmitTime = 0; // Para controlar el tiempo de transmisión
    const unsigned long timeout = 5000; // Tiempo de espera en milisegundos

    if (client.connected()) {
        // Si ya estamos conectados, solo enviamos los datos
        client.write(spi_buffer, LEN_BUFFER_SPI);
        lastTransmitTime = millis(); // Reinicia el temporizador
    } else {
        // Intentamos conectarnos si no estamos conectados
        if (client.connect(ip_servidor, 4000)) {
            client.write(spi_buffer, LEN_BUFFER_SPI);
            lastTransmitTime = millis(); // Reinicia el temporizador
        }
    }

    // Cierra la conexión si ha pasado el tiempo de espera
    if (millis() - lastTransmitTime > timeout) {
        client.stop(); // Cierra la conexión
    }
}

void ADC_DMA_Init(void) {
    // Habilita los relojes para ADC1, GPIOA y DMA1
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN | RCC_APB2ENR_IOPAEN;
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;

    // Configura el pin PA0 como entrada analógica
    GPIOA->CRL &= ~(GPIO_CRL_CNF0 | GPIO_CRL_MODE0);

    // Configura el ADC
    ADC1->SQR3 = 0;                        // Primer canal en la secuencia (canal 0 para PA0)
    ADC1->SMPR2 = ADC_SMPR2_SMP0_1;        // Tiempo de muestreo adecuado para el canal 0
    ADC1->CR1 = 0;
    ADC1->CR2 = ADC_CR2_ADON;              // Enciende el ADC
    delay(1);                              // Pequeña demora para estabilización
    ADC1->CR2 |= ADC_CR2_CAL;              // Realiza la calibración
    while (ADC1->CR2 & ADC_CR2_CAL);       // Espera a que termine la calibración
    ADC1->CR2 |= ADC_CR2_CONT | ADC_CR2_DMA; // Habilita el modo continuo y DMA
    ADC1->CR2 |= ADC_CR2_ADON;             // Enciende nuevamente el ADC

    // Configura el DMA
    DMA1_Channel1->CPAR = (uint32_t)&ADC1->DR; // Dirección periférica: registro de datos del ADC1
    DMA1_Channel1->CMAR = (uint32_t)adc_buffer; // Dirección de memoria: buffer de ADC
    DMA1_Channel1->CNDTR = LEN_BUFFER_ADC;  // Número de datos a transferir
    DMA1_Channel1->CCR = DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_TCIE | DMA_CCR_EN | DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0;  // Configuración del canal DMA:
                         // MINC: Incremento automático de la dirección de memoria
                                        // CIRC: Modo circular
                                                        // TCIE: Interrupción de transferencia completa
                                                                         // EN: Habilitar canal
                                                                                    // bit 0 de MSIZE en 1, para escribir 16 bit
                                                                                                        // bit 0 de PSIZE en 1, para leer 16 bit

    // Habilita la interrupción DMA1 Channel 1
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}


void Ethernet_Init(void) {
    // Inicializa el W5100
    static byte mac[6] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
    static byte ip[4] = {192, 168, 1, 100};
    Ethernet.begin(mac, ip);  // Configura la dirección MAC y IP
}

void DMA1_Channel1_IRQHandler(void) {
    if (DMA1->ISR & DMA_ISR_TCIF1) {  // Verifica la bandera de transferencia completa
        DMA1->IFCR |= DMA_IFCR_CTCIF1; // Limpia la bandera de transferencia completa
        if (!transmitir) {
            // Copia los datos del buffer de ADC al buffer de SPI
            for (int i = 0; i < LEN_BUFFER_ADC; i++) {
                spi_buffer[i * 2] = adc_buffer[i] >> 8;
                spi_buffer[i * 2 + 1] = adc_buffer[i] & 0xFF;
            }
            transmitir = true;  // Habilita la transmisión de datos
        }
    }
}
// mejoras?