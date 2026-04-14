#include "encoder.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

#define ENC_A_L  4u
#define ENC_B_L  5u

volatile long left_ticks  = 0;
volatile long right_ticks = 0;

static void encoder_callback(uint gpio, uint32_t events) {
    bool a = gpio_get(ENC_A_L);
    bool b = gpio_get(ENC_B_L);
    if (gpio == ENC_A_L) {
        if (a == b) left_ticks++;
        else        left_ticks--;
    } else {
        if (a != b) left_ticks++;
        else        left_ticks--;
    }
}

static void setup_encoder_pin(uint pin) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    gpio_pull_up(pin);
}

void encoder_init() {
    setup_encoder_pin(ENC_A_L);
    setup_encoder_pin(ENC_B_L);

    gpio_set_irq_enabled_with_callback(
        ENC_A_L,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
        true,
        &encoder_callback
    );
    gpio_set_irq_enabled(ENC_B_L, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
}

long get_left_ticks()  { return left_ticks; }
long get_right_ticks() { return right_ticks; }