// #include "pico/stdlib.h"
// #include "hardware/pwm.h"
// #include "hardware/gpio.h"
// #include <cstdio>

// const uint MOTOR_FOW_PIN = 28; // GP28
// const uint MOTOR_BAK_PIN = 27; // GP27

// int main() {
//     stdio_init_all();
//     // Set the motor pins as outputs
//     // gpio_init(MOTOR_FOW_PIN); // initializes the GPIO for the forard case 
//     // gpio_set_dir(MOTOR_FOW_PIN, GPIO_OUT); // sets the directions of the pins to output 
//     // gpio_init(MOTOR_BAK_PIN); // initializes the GPIO for the backward case
//     // gpio_set_dir(MOTOR_BAK_PIN, GPIO_OUT); // sets the directions of the

//     gpio_set_function(MOTOR_FOW_PIN, GPIO_FUNC_PWM); // sets the forward pin to PWM function
//     gpio_set_function(MOTOR_BAK_PIN, GPIO_FUNC_PWM); // sets the backward pin to PWM function

//     uint slice_num_a = pwm_gpio_to_slice_num(MOTOR_FOW_PIN);
//     uint slice_num_b = pwm_gpio_to_slice_num(MOTOR_BAK_PIN);

//     pwm_set_clkdiv(slice_num_a, 125.0f); // sets the clock divider for the forward pin to 125, which gives us a 1us resolution
//     pwm_set_clkdiv(slice_num_b, 125.0f); // sets the clock
//     pwm_set_wrap(slice_num_a, 20000); // sets the wrap value for the forward pin to 20000, which gives us a 20ms period
//     pwm_set_wrap(slice_num_b, 20000); // sets the wrap value for the backward pin to 20000, which gives us a 20ms period

//     pwm_set_enabled(slice_num_a, true); // enables the PWM for the forward pin
//     pwm_set_enabled(slice_num_b, true); // enables the PWM for the backward pin

//     uint channel_a = pwm_gpio_to_channel(MOTOR_FOW_PIN); // gets the channel for the forward pin
//     uint channel_b = pwm_gpio_to_channel(MOTOR_BAK_PIN); // gets the channel for the backward pin




//     // Main loop
//     while (true) {

//         //move forward
//         // pwm_set_chan(MOTOR_FOW_PIN, 1); // sets the forward pin to high
//         // gpio_put(MOTOR_BAK_PIN, 0); // sets the backward pin to low


//         printf("Moving forward\n");
//         pwm_set_chan_level(slice_num_a, channel_a, 15000); // sets the duty cycle for the forward pin to 75% (15000/20000)
//         pwm_set_chan_level(slice_num_b, channel_b, 0); // sets the duty cycle for the backward pin to 0%
//         sleep_ms(2000); // move forward for 2 seconds

//         // stop
//         printf("Stopping\n");
//         pwm_set_chan_level(slice_num_a, channel_a, 0); // sets the duty cycle for the forward pin to 0%
//         pwm_set_chan_level(slice_num_b, channel_b, 0); // sets the duty cycle for the backward pin to 0%
//         sleep_ms(100);

//         //move backward
//         printf("Moving backward\n");
//         pwm_set_chan_level(slice_num_a, channel_a, 0); // sets the duty cycle for the forward pin to 0%
//         pwm_set_chan_level(slice_num_b, channel_b, 15000); // sets the duty cycle for the backward pin to 75% (15000/20000)
//         sleep_ms(2000); // move backward for 2 seconds
//     }


// }

#include "pico/stdlib.h"

#define LED 25
//motor B
#define IN3 0
#define IN4 1
#define ENB 2

// //motor A
#define IN1 4
#define IN2 5
#define ENA 6

int main() {
    gpio_init(LED); gpio_set_dir(LED, GPIO_OUT);
    
    gpio_init(IN3); gpio_set_dir(IN3, GPIO_OUT);
    gpio_init(IN4); gpio_set_dir(IN4, GPIO_OUT);
    gpio_init(ENB); gpio_set_dir(ENB, GPIO_OUT);
    
    gpio_init(IN1); gpio_set_dir(IN1, GPIO_OUT);
    gpio_init(IN2); gpio_set_dir(IN2, GPIO_OUT);
    gpio_init(ENA); gpio_set_dir(ENA, GPIO_OUT);

    gpio_put(ENB, 1);
    gpio_put(IN3, 1);
    gpio_put(IN4, 0);
    gpio_put(LED, 1);

    gpio_put(ENA, 1);
    gpio_put(IN1, 1);
    gpio_put(IN2, 0);

    sleep_ms(5000);

    gpio_put(ENB, 0);
    gpio_put(IN3, 0);
    gpio_put(IN4, 0);
    gpio_put(LED, 0);

    gpio_put(ENA, 0);
    gpio_put(IN1, 0);
    gpio_put(IN2, 0);

    while (true);
}