#include "motor_driver.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"

#define RPWM  2u
#define LPWM  3u
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

static void set_motor(int pwm_val) {
    if (pwm_val > 0) {
        pwm_set_gpio_level(RPWM, (uint)pwm_val);
        pwm_set_gpio_level(LPWM, 0);
    } else if (pwm_val < 0) {
        pwm_set_gpio_level(RPWM, 0);
        pwm_set_gpio_level(LPWM, (uint)(-pwm_val));
    } else {
        pwm_set_gpio_level(RPWM, 0);
        pwm_set_gpio_level(LPWM, 0);
    }
}

void motor_init() {
    setup_pwm_pin(RPWM);
    setup_pwm_pin(LPWM);
}

void motor_set(int left_pwm, int right_pwm) {
    if (left_pwm  >  255) left_pwm  =  255;
    if (left_pwm  < -255) left_pwm  = -255;
    set_motor(left_pwm);
}