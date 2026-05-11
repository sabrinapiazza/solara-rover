#include "motor_driver.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"

#define RPWM_L1  2u
#define LPWM_L1  3u
#define RPWM_L2 10u
#define LPWM_L2 11u
#define RPWM_R1  6u
#define LPWM_R1  7u
#define RPWM_R2 14u
#define LPWM_R2 15u
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

static int current_left  = 0;
static int current_right = 0;

void motor_init() {
    setup_pwm_pin(RPWM_L1);
    setup_pwm_pin(LPWM_L1);
    setup_pwm_pin(RPWM_L2);
    setup_pwm_pin(LPWM_L2);
    setup_pwm_pin(RPWM_R1);
    setup_pwm_pin(LPWM_R1);
    setup_pwm_pin(RPWM_R2);
    setup_pwm_pin(LPWM_R2);
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

    set_side(RPWM_L1, LPWM_L1, current_left);
    set_side(RPWM_L2, LPWM_L2, current_left);
    set_side(RPWM_R1, LPWM_R1, current_right);
    set_side(RPWM_R2, LPWM_R2, current_right);
}

