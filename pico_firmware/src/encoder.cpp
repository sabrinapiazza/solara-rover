#include "encoder.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

#define ENC_A_L1  4u
#define ENC_B_L1  5u
#define ENC_A_R1  8u
#define ENC_B_R1  9u
#define ENC_A_L2 12u
#define ENC_B_L2 13u
#define ENC_A_R2 16u
#define ENC_B_R2 17u

volatile long left_ticks   = 0;
volatile long right_ticks  = 0;
volatile long left2_ticks  = 0;
volatile long right2_ticks = 0;

static void encoder_callback(uint gpio, uint32_t events) {
    if (gpio == ENC_A_L1 || gpio == ENC_B_L1) {
        bool a = gpio_get(ENC_A_L1), b = gpio_get(ENC_B_L1);
        if (gpio == ENC_A_L1) { if (a == b) left_ticks++; else left_ticks--; }
        else                  { if (a != b) left_ticks++; else left_ticks--; }
    }
    if (gpio == ENC_A_R1 || gpio == ENC_B_R1) {
        bool a = gpio_get(ENC_A_R1), b = gpio_get(ENC_B_R1);
        if (gpio == ENC_A_R1) { if (a == b) right_ticks++; else right_ticks--; }
        else                  { if (a != b) right_ticks++; else right_ticks--; }
    }
    if (gpio == ENC_A_L2 || gpio == ENC_B_L2) {
        bool a = gpio_get(ENC_A_L2), b = gpio_get(ENC_B_L2);
        if (gpio == ENC_A_L2) { if (a == b) left2_ticks++; else left2_ticks--; }
        else                  { if (a != b) left2_ticks++; else left2_ticks--; }
    }
    if (gpio == ENC_A_R2 || gpio == ENC_B_R2) {
        bool a = gpio_get(ENC_A_R2), b = gpio_get(ENC_B_R2);
        if (gpio == ENC_A_R2) { if (a == b) right2_ticks++; else right2_ticks--; }
        else                  { if (a != b) right2_ticks++; else right2_ticks--; }
    }
}

static void setup_encoder_pin(uint pin) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    gpio_pull_up(pin);
}

void encoder_init() {
    setup_encoder_pin(ENC_A_L1); setup_encoder_pin(ENC_B_L1);
    setup_encoder_pin(ENC_A_R1); setup_encoder_pin(ENC_B_R1);
    setup_encoder_pin(ENC_A_L2); setup_encoder_pin(ENC_B_L2);
    setup_encoder_pin(ENC_A_R2); setup_encoder_pin(ENC_B_R2);

    // gpio_set_irq_enabled_with_callback(ENC_A_L1,
    //     GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoder_callback);
    // gpio_set_irq_enabled(ENC_B_L1, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    // gpio_set_irq_enabled(ENC_A_R1, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    // gpio_set_irq_enabled(ENC_B_R1, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    // gpio_set_irq_enabled(ENC_A_L2, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    // gpio_set_irq_enabled(ENC_B_L2, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    // gpio_set_irq_enabled(ENC_A_R2, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    // gpio_set_irq_enabled(ENC_B_R2, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
}

long get_left_ticks()   { return left_ticks; }
long get_right_ticks()  { return right_ticks; }
long get_left2_ticks()  { return left2_ticks; }
long get_right2_ticks() { return right2_ticks; }