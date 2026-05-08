#include "motor_driver.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"

// Motor 1 (left front) & Motor 3 (left rear)
#define RPWM_L1  2u
#define LPWM_L1  3u
#define RPWM_L2 10u
#define LPWM_L2 11u

// Motor 2 (right front) & Motor 4 (right rear)
#define RPWM_R1  6u
#define LPWM_R1  7u
#define RPWM_R2 14u
#define LPWM_R2 15u

#define PWM_WRAP 255u

static void setup_pwm_pin(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pin);
    uint channel = pwm_gpio_to_channel(pin);
    pwm_set_clkdiv(slice, 245.1f);
    pwm_set_wrap(slice, PWM_WRAP);
    pwm_set_chan_level(slice, channel, 0);
    pwm_set_enabled(slice, true);
}

static void set_side(uint rpwm, uint lpwm, int pwm_val) {
    if (pwm_val > 0) {
        pwm_set_gpio_level(rpwm, (uint)pwm_val);
        pwm_set_gpio_level(lpwm, 0);
    } else if (pwm_val < 0) {
        pwm_set_gpio_level(rpwm, 0);
        pwm_set_gpio_level(lpwm, (uint)(-pwm_val));
    } else {
        pwm_set_gpio_level(rpwm, 0);
        pwm_set_gpio_level(lpwm, 0);
    }
}

static int current_left  = 0;
static int current_right = 0;

// void motor_init() {
//     setup_pwm_pin(RPWM_L1);
//     setup_pwm_pin(LPWM_L1);
//     setup_pwm_pin(RPWM_L2);
//     setup_pwm_pin(LPWM_L2);
//     setup_pwm_pin(RPWM_R1);
//     setup_pwm_pin(LPWM_R1);
//     setup_pwm_pin(RPWM_R2);
//     setup_pwm_pin(LPWM_R2);
// }

void motor_init() {
    // Initialize slices once, then set up individual pins
    setup_pwm_pin(RPWM_L1);  // slice 1
    setup_pwm_pin(RPWM_L2);  // slice 5
    setup_pwm_pin(RPWM_R1);  // slice 3
    setup_pwm_pin(RPWM_R2);  // slice 7

    // Now just set GPIO function for LPWM pins (same slices already initialized)
    gpio_set_function(LPWM_L1, GPIO_FUNC_PWM);
    gpio_set_function(LPWM_L2, GPIO_FUNC_PWM);
    gpio_set_function(LPWM_R1, GPIO_FUNC_PWM);
    gpio_set_function(LPWM_R2, GPIO_FUNC_PWM);

    pwm_set_gpio_level(LPWM_L1, 0);
    pwm_set_gpio_level(LPWM_L2, 0);
    pwm_set_gpio_level(LPWM_R1, 0);
    pwm_set_gpio_level(LPWM_R2, 0);
}

void motor_set(int left_pwm, int right_pwm) {
    if (left_pwm  >  255) left_pwm  =  255;
    if (left_pwm  < -255) left_pwm  = -255;
    if (right_pwm >  255) right_pwm =  255;
    if (right_pwm < -255) right_pwm = -255;

    if (left_pwm > current_left + 5)        current_left += 5;
    else if (left_pwm < current_left - 5)   current_left -= 5;
    else                                     current_left = left_pwm;

    if (right_pwm > current_right + 5)      current_right += 5;
    else if (right_pwm < current_right - 5) current_right -= 5;
    else                                     current_right = right_pwm;

    // set_side(RPWM_L1, LPWM_L1, current_left);
    // // set_side(RPWM_L2, LPWM_L2, current_left);   // rear left disabled
    // set_side(RPWM_R1, LPWM_R1, current_right);
    // // set_side(RPWM_R2, LPWM_R2, current_right);  // rear right disabled

    set_side(RPWM_L1, LPWM_L1, current_left);      // motor 1 - active
     set_side(RPWM_L2, LPWM_L2, current_left);   // motor 3 - disabled
    set_side(RPWM_R1, LPWM_R1, current_right);  // motor 2 - disabled
    //  set_side(RPWM_R2, LPWM_R2, current_right);  // motor 4 - disabled
}