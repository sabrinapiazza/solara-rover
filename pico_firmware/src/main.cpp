// #include "pico/stdlib.h"
// #include "pico/cyw43_arch.h"
// #include "motor_driver.h"
// #include "encoder.h"
// #include <cstdio>
// #include <cstring>
// #include <cstdlib>

// #define SERIAL_TIMEOUT_MS 100

// int main() {
//     cyw43_arch_init();
//     stdio_init_all();
//     sleep_ms(2000);

//     motor_init();
//     encoder_init();

//     char buf[64];
//     int buf_idx = 0;

//     while (true) {
//         int c = getchar_timeout_us(SERIAL_TIMEOUT_MS * 1000);

//         if (c != PICO_ERROR_TIMEOUT) {
//             if (c == '\n') {
//                 buf[buf_idx] = '\0';
//                 buf_idx = 0;

//                 if (strncmp(buf, "CMD:", 4) == 0) {
//                     char *data = buf + 4;
//                     char *comma = strchr(data, ',');
//                     if (comma != NULL) {
//                         *comma = '\0';
//                         int left_pwm = atoi(data);
//                         int right_pwm = atoi(comma + 1);
//                         motor_set(left_pwm, right_pwm);
//                     }
//                 }
//             } else {
//                 if (buf_idx < 63) {
//                     buf[buf_idx++] = (char)c;
//                 }
//             }
//         }

//         static uint32_t last_enc_send = 0;
//         uint32_t now = to_ms_since_boot(get_absolute_time());
//         if (now - last_enc_send >= 50) {
//             last_enc_send = now;
//             printf("ENC:%ld,%ld,%ld,%ld\n",
//                 get_left_ticks(), get_right_ticks(),
//                 get_left2_ticks(), get_right2_ticks());
//         }
//     }
// }

#include "pico/stdlib.h"
// #include "pico/cyw43_arch.h"
#include "motor_driver.h"
#include "encoder.h"
#include <cstdio>
#include <cstring>
#include <cstdlib>

#define SERIAL_TIMEOUT_MS 100

int main() {
    // cyw43_arch_init();
    stdio_init_all();
    sleep_ms(2000);

    motor_init();
    encoder_init();

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
                        int left_pwm = atoi(data);
                        int right_pwm = atoi(comma + 1);
                        motor_set(left_pwm, right_pwm);
                    }
                }
            } else {
                if (buf_idx < 63) {
                    buf[buf_idx++] = (char)c;
                }
            }
        }

        static uint32_t last_enc_send = 0;
        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (now - last_enc_send >= 50) {
            last_enc_send = now;
            printf("ENC:%ld,%ld,%ld,%ld\n",
                get_left_ticks(), get_right_ticks(),
                get_left2_ticks(), get_right2_ticks());
        }
    }
}
