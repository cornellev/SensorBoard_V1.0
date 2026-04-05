#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "hardware/adc.h"
#include "hardware/spi.h"
#include "pico/stdlib.h"

#include "../framed_spi.h"

#define LED 25
#define N_CH            2
#define ADC_VREF        3.3f // max voltage on ADC input
#define ADC_COUNTS_MAX  4095.0f // rp2040 12-bit ADC max count

#define SPI_PORT spi0
#define PIN_RX   16
#define PIN_CS   6
#define PIN_SCK  18
#define PIN_TX   19
// #define PIN_TX   5

#define PAYLOAD_LEN (4u + 4u * (unsigned)N_CH)
#define FRAME_MAX_BYTES FRAMED_SPI_FRAME_MAX_BYTES(PAYLOAD_LEN)

uint8_t payload_buf[PAYLOAD_LEN];
uint8_t frame_buf[FRAME_MAX_BYTES];
static framed_spi_t framed;

static inline float adc_counts_to_volts(uint16_t raw) {
    return ((float)raw) * (ADC_VREF / ADC_COUNTS_MAX);
}

static void build_payload(uint8_t *payload, uint32_t now_us) {
    framed_spi_pack_u32_le(&payload[0], now_us);

    // brake is gpio 27
    adc_select_input(1);
    uint16_t raw_brake = adc_read();
    float brake = adc_counts_to_volts(raw_brake);
    size_t brake_off = 4u;
    framed_spi_pack_f32_le(&payload[brake_off], brake);

    // steering is gpio 28
    adc_select_input(2);
    uint16_t raw_steer = adc_read();
    float steer = adc_counts_to_volts(raw_steer);
    size_t steer_off = brake_off + 4u;
    framed_spi_pack_f32_le(&payload[steer_off], steer);
}

static void irq_handler(uint gpio, uint32_t events) {
    if (events & GPIO_IRQ_EDGE_RISE) {
        uint32_t now_us = (uint32_t)time_us_64();
        build_payload(payload_buf, now_us);
        (void)framed_spi_send_payload(&framed, payload_buf, PAYLOAD_LEN);
    }

    if (events & GPIO_IRQ_EDGE_FALL) {
        framed_spi_end_transaction(&framed);
    }
}

int main(void) {
    stdio_init_all();

    gpio_init(LED);
    gpio_set_dir(LED, GPIO_OUT);
    gpio_put(LED, 1);

    adc_init();
    adc_gpio_init(27);
    adc_gpio_init(28);

    spi_init(SPI_PORT, 1000 * 1000);
    spi_set_slave(SPI_PORT, true);
    spi_set_format(SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_1, false);

    gpio_set_function(PIN_RX, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_TX, GPIO_FUNC_SPI);

    framed_spi_init(&framed, SPI_PORT, PIN_TX, DREQ_SPI0_TX, frame_buf, FRAME_MAX_BYTES);

    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_IN);
    gpio_set_irq_enabled_with_callback(
        PIN_CS,
        GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE,
        true,
        &irq_handler
    );

    while (true) {
        tight_loop_contents();
    }
}
