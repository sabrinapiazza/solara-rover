// #include "pico/stdlib.h"
// #include "hardware/pwm.h"
// #include "hardware/timer.h"
// #include "pico/cyw43_arch.h"
// #include <stdio.h>

// // -------- Motor Pins --------
// #define IN1 0   // GP0
// #define IN2 1   // GP1
// #define ENA 2   // GP2 (PWM)

// // PWM settings
// uint slice_num;




// // ---------- Motor Functions ----------
// void motor_forward(int speed) {
//     gpio_put(IN1, 1);
//     gpio_put(IN2, 0);
//     pwm_set_gpio_level(ENA, speed);
// }

// void motor_stop() {
//     gpio_put(IN1, 0);
//     gpio_put(IN2, 0);
//     pwm_set_gpio_level(ENA, 0);
// }


// // ---------- Main ----------
// int main() {
//     stdio_init_all();
//     cyw43_arch_init();

//     for(int i = 0; i < 5; i++) {
//         cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
//         sleep_ms(500);
        
//         cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
//         sleep_ms(500);
//     }

//     // Motor GPIO setup
//     gpio_init(IN1);
//     gpio_set_dir(IN1, GPIO_OUT);

//     gpio_init(IN2);
//     gpio_set_dir(IN2, GPIO_OUT);

//     for(int i = 0; i < 5; i++) {
//         cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
//         sleep_ms(500);
        
//         cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
//         sleep_ms(500);
//     }

//     // PWM setup for ENA
//     gpio_set_function(ENA, GPIO_FUNC_PWM);
//     slice_num = pwm_gpio_to_slice_num(ENA);

//     pwm_set_wrap(slice_num, 65535); // max resolution
//     pwm_set_enabled(slice_num, true);

//     for(int i = 0; i < 5; i++) {
//         cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
//         sleep_ms(500);
        
//         cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
//         sleep_ms(500);
//     }
    
//     return 0;
// }

#include "pico/stdlib.h"

#define LED 25
#define IN3 0
#define IN4 1
#define ENB 2

int main() {
    gpio_init(LED); gpio_set_dir(LED, GPIO_OUT);
    gpio_init(IN3); gpio_set_dir(IN3, GPIO_OUT);
    gpio_init(IN4); gpio_set_dir(IN4, GPIO_OUT);
    gpio_init(ENB); gpio_set_dir(ENB, GPIO_OUT);

    gpio_put(ENB, 1);
    gpio_put(IN3, 1);
    gpio_put(IN4, 0);
    gpio_put(LED, 1);

    sleep_ms(5000);

    gpio_put(ENB, 0);
    gpio_put(IN3, 0);
    gpio_put(IN4, 0);
    gpio_put(LED, 0);

    while (true);
}