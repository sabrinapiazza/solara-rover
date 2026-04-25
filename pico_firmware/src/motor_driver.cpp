#include "motor_driver.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"

#define RPWM_L  2u
#define LPWM_L  3u
#define RPWM_R  6u
#define LPWM_R  7u
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

void motor_init() {
    setup_pwm_pin(RPWM_L);
    setup_pwm_pin(LPWM_L);
    setup_pwm_pin(RPWM_R);
    setup_pwm_pin(LPWM_R);
}

void motor_set(int left_pwm, int right_pwm) {
    if (left_pwm  >  255) left_pwm  =  255;
    if (left_pwm  < -255) left_pwm  = -255;
    if (right_pwm >  255) right_pwm =  255;
    if (right_pwm < -255) right_pwm = -255;

    set_side(RPWM_L, LPWM_L, left_pwm);
    set_side(RPWM_R, LPWM_R, right_pwm);
}