#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include <cstdio>

const uint MOTOR_FOW_PIN = 28; // GP28
const uint MOTOR_BAK_PIN = 27; // GP27

int main() {
    stdio_init_all();
    // Set the motor pins as outputs
    // gpio_init(MOTOR_FOW_PIN); // initializes the GPIO for the forard case 
    // gpio_set_dir(MOTOR_FOW_PIN, GPIO_OUT); // sets the directions of the pins to output 
    // gpio_init(MOTOR_BAK_PIN); // initializes the GPIO for the backward case
    // gpio_set_dir(MOTOR_BAK_PIN, GPIO_OUT); // sets the directions of the

    gpio_set_function(MOTOR_FOW_PIN, GPIO_FUNC_PWM); // sets the forward pin to PWM function
    gpio_set_function(MOTOR_BAK_PIN, GPIO_FUNC_PWM); // sets the backward pin to PWM function

    uint slice_num_a = pwm_gpio_to_slice_num(MOTOR_FOW_PIN);
    uint slice_num_b = pwm_gpio_to_slice_num(MOTOR_BAK_PIN);

    pwm_set_clkdiv(slice_num_a, 125.0f); // sets the clock divider for the forward pin to 125, which gives us a 1us resolution
    pwm_set_clkdiv(slice_num_b, 125.0f); // sets the clock
    pwm_set_wrap(slice_num_a, 20000); // sets the wrap value for the forward pin to 20000, which gives us a 20ms period
    pwm_set_wrap(slice_num_b, 20000); // sets the wrap value for the backward pin to 20000, which gives us a 20ms period

    pwm_set_enabled(slice_num_a, true); // enables the PWM for the forward pin
    pwm_set_enabled(slice_num_b, true); // enables the PWM for the backward pin

    uint channel_a = pwm_gpio_to_channel(MOTOR_FOW_PIN); // gets the channel for the forward pin
    uint channel_b = pwm_gpio_to_channel(MOTOR_BAK_PIN); // gets the channel for the backward pin




    // Main loop
    while (true) {

        //move forward
        // pwm_set_chan(MOTOR_FOW_PIN, 1); // sets the forward pin to high
        // gpio_put(MOTOR_BAK_PIN, 0); // sets the backward pin to low


        printf("Moving forward\n");
        pwm_set_chan_level(slice_num_a, channel_a, 15000); // sets the duty cycle for the forward pin to 75% (15000/20000)
        pwm_set_chan_level(slice_num_b, channel_b, 0); // sets the duty cycle for the backward pin to 0%
        sleep_ms(2000); // move forward for 2 seconds

        // stop
        printf("Stopping\n");
        pwm_set_chan_level(slice_num_a, channel_a, 0); // sets the duty cycle for the forward pin to 0%
        pwm_set_chan_level(slice_num_b, channel_b, 0); // sets the duty cycle for the backward pin to 0%
        sleep_ms(100);

        //move backward
        printf("Moving backward\n");
        pwm_set_chan_level(slice_num_a, channel_a, 0); // sets the duty cycle for the forward pin to 0%
        pwm_set_chan_level(slice_num_b, channel_b, 15000); // sets the duty cycle for the backward pin to 75% (15000/20000)
        sleep_ms(2000); // move backward for 2 seconds
    }


}