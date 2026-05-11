#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "pico/cyw43_arch.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define ENA 16
#define IN1 17
#define IN2 18
#define ENB 13
#define IN3 14
#define IN4 15
#define ENC 10
#define IN5 11
#define IN6 12
#define END 19
#define IN7 20
#define IN8 21

#define SERIAL_TIMEOUT_MS 100

void setup_pwm(uint gpio) {
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(gpio);
    pwm_set_wrap(slice, 255);
    pwm_set_chan_level(slice, pwm_gpio_to_channel(gpio), 0);
    pwm_set_enabled(slice, true);
}

void motor_init() {
    gpio_init(IN1); gpio_set_dir(IN1, GPIO_OUT);
    gpio_init(IN2); gpio_set_dir(IN2, GPIO_OUT);
    gpio_init(IN3); gpio_set_dir(IN3, GPIO_OUT);
    gpio_init(IN4); gpio_set_dir(IN4, GPIO_OUT);
    gpio_init(IN5); gpio_set_dir(IN5, GPIO_OUT);
    gpio_init(IN6); gpio_set_dir(IN6, GPIO_OUT);
    gpio_init(IN7); gpio_set_dir(IN7, GPIO_OUT);
    gpio_init(IN8); gpio_set_dir(IN8, GPIO_OUT);

    setup_pwm(ENA);
    setup_pwm(ENB);
    setup_pwm(ENC);
    setup_pwm(END);
}

void motor_set(uint en, uint in_a, uint in_b, int pwm) {
    if (pwm > 0) {
        gpio_put(in_a, 1); gpio_put(in_b, 0);
    } else if (pwm < 0) {
        gpio_put(in_a, 0); gpio_put(in_b, 1);
        pwm = -pwm;
    } else {
        gpio_put(in_a, 0); gpio_put(in_b, 0);
    }
    pwm_set_chan_level(pwm_gpio_to_slice_num(en), pwm_gpio_to_channel(en), pwm);
}

void motors_set(int left, int right) {
    // Left side
    motor_set(ENA, IN1, IN2, left);
    motor_set(ENB, IN3, IN4, left);
    // Right side
    motor_set(ENC, IN5, IN6, right);
    motor_set(END, IN7, IN8, right);
}

int main() {
    stdio_init_all();
    cyw43_arch_init();
    sleep_ms(2000);

    motor_init();

    // 3 blinks = ready
    for (int i = 0; i < 3; i++) {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        sleep_ms(200);
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        sleep_ms(200);
    }

    printf("READY\n");

    char buf[64];
    int buf_idx = 0;

    while (true) {
        int c = getchar_timeout_us(SERIAL_TIMEOUT_MS * 1000);

        if (c != PICO_ERROR_TIMEOUT && c != EOF) {
            if (c == '\n') {
                buf[buf_idx] = '\0';
                buf_idx = 0;

                if (strncmp(buf, "CMD:", 4) == 0) {
                    char *data = buf + 4;
                    char *comma = strchr(data, ',');
                    if (comma != NULL) {
                        *comma = '\0';
                        int left = atoi(data);
                        int right = atoi(comma + 1);
                        motors_set(left, right);
                    }
                }
            } else {
                if (buf_idx < 63) {
                    buf[buf_idx++] = (char)c;
                }
            }
        }
    }
}