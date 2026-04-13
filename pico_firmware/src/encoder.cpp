#include "encoder.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

// ─── Encoder Pins ─────────────────────────────────────────────
#define ENC_A_L  10u
#define ENC_B_L  11u
#define ENC_A_R  12u
#define ENC_B_R  13u

volatile long left_ticks  = 0;
volatile long right_ticks = 0;

static void encoder_callback(uint gpio, uint32_t events) {
    // Left encoder
    if (gpio == ENC_A_L || gpio == ENC_B_L) {
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

    // Right encoder
    if (gpio == ENC_A_R || gpio == ENC_B_R) {
        bool a = gpio_get(ENC_A_R);
        bool b = gpio_get(ENC_B_R);
        if (gpio == ENC_A_R) {
            if (a == b) right_ticks++;
            else        right_ticks--;
        } else {
            if (a != b) right_ticks++;
            else        right_ticks--;
        }
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
    setup_encoder_pin(ENC_A_R);
    setup_encoder_pin(ENC_B_R);

    gpio_set_irq_enabled_with_callback(
        ENC_A_L,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
        true,
        &encoder_callback
    );
    gpio_set_irq_enabled(ENC_B_L, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(ENC_A_R, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(ENC_B_R, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
}

long get_left_ticks()  { return left_ticks; }
long get_right_ticks() { return right_ticks; }