#include "motor_driver.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"

#define FL_RPWM  2u
#define FL_LPWM  3u
#define RL_RPWM  4u
#define RL_LPWM  5u
#define FR_RPWM  6u
#define FR_LPWM  7u
#define RR_RPWM  8u
#define RR_LPWM  9u

#define PWM_WRAP 255u

static void setup_pwm_pin(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pin);
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, 245.1f);
    pwm_config_set_wrap(&cfg, PWM_WRAP);
    pwm_init(slice, &cfg, true);
    pwm_set_gpio_level(pin, 0);
}

static void set_side(uint rpwm_pin, uint lpwm_pin, int pwm_val) {
    if (pwm_val > 0) {
        pwm_set_gpio_level(rpwm_pin, (uint)pwm_val);
        pwm_set_gpio_level(lpwm_pin, 0);
    } else if (pwm_val < 0) {
        pwm_set_gpio_level(rpwm_pin, 0);
        pwm_set_gpio_level(lpwm_pin, (uint)(-pwm_val));
    } else {
        pwm_set_gpio_level(rpwm_pin, 0);
        pwm_set_gpio_level(lpwm_pin, 0);
    }
}

void motor_init() {
    setup_pwm_pin(FL_RPWM);
    setup_pwm_pin(FL_LPWM);
    setup_pwm_pin(RL_RPWM);
    setup_pwm_pin(RL_LPWM);
    setup_pwm_pin(FR_RPWM);
    setup_pwm_pin(FR_LPWM);
    setup_pwm_pin(RR_RPWM);
    setup_pwm_pin(RR_LPWM);
}

void motor_set(int left_pwm, int right_pwm) {
    if (left_pwm  >  255) left_pwm  =  255;
    if (left_pwm  < -255) left_pwm  = -255;
    if (right_pwm >  255) right_pwm =  255;
    if (right_pwm < -255) right_pwm = -255;

    set_side(FL_RPWM, FL_LPWM, left_pwm);
    set_side(RL_RPWM, RL_LPWM, left_pwm);
    set_side(FR_RPWM, FR_LPWM, right_pwm);
    set_side(RR_RPWM, RR_LPWM, right_pwm);
}