
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "pico/cyw43_arch.h"
#include "claw_controllerv3.h"
#include <math.h>

#define BASE 16
#define JOINT_1 14
#define JOINT_2 13
#define GRIPPER 17
#define debug false


void setServo(PWM *pin, float degree){ 
    if (degree < 0.0f || degree > 180.0f){
       // printf("Please use value of 0-180\n\r");
    } else {
        int value = (int)((((float)(pin->pulseMax-pin->pulseMin))*(degree/180.0))+pin->pulseMin);
        int lastDegree = pin->lastDegree; //
        pin->lastDegree = degree; //
        pin->value = value; 
        pwm_set_chan_level(pin->slice, pin->channel, pin->value);
        moveDelay(pin->delay, degree, lastDegree); //Need time for the servo to move to its new position
       // if(debug) printf("slice:%d channel:%d value:%d degree:%3.0f \r\n", pin->slice, pin->channel, pin->value, degree);
    }
} 

void setBaseServo(PWM *pin, float degree){ 
    if (degree < 0.0f || degree > 195.0f){
        //printf("Please use value of 0-195\n\r");
    } else {
        int value = (int)((((float)(pin->pulseMax-pin->pulseMin))*(degree/195.0))+pin->pulseMin);
        int lastDegree = pin->lastDegree; //
        pin->lastDegree = degree; //
        pin->value = value; 
        pwm_set_chan_level(pin->slice, pin->channel, pin->value);
        moveDelay(pin->delay, degree, lastDegree); //Need time for the servo to move to its new position
       // if(debug) printf("slice:%d channel:%d value:%d degree:%3.0f \r\n", pin->slice, pin->channel, pin->value, degree);
    }
} 


void set2Servos(PWM *pin1, PWM *pin2, float degree){ 
    if (degree < 0.0f || degree > 180.0f){
      //  printf("Please use value of 0-180\n\r");
    } else {
        int value = (int)((((float)(pin1->pulseMax-pin1->pulseMin))*(degree/180.0))+pin1->pulseMin);
        // int value2 = (int)((((float)(pin2->pulseMax-pin2->pulseMin))*(degree/180.0))+pin2->pulseMin); 

        int lastDegree = pin1->lastDegree;  
        pin1->lastDegree = degree; 

        // int lastDegree2 = pin2->lastDegree; 
        pin2->lastDegree = degree;

        pin1->value = value; 
        pin2->value = value;

        pwm_set_chan_level(pin1->slice, pin1->channel, pin1->value);  //initialize where it needs to go first motor 
        pwm_set_chan_level(pin2->slice, pin2->channel, pin2->value); //initialize where it needs to go second motor 
        // moveDelay(pin1->delay, degree, lastDegree); //ensures they wait
        // moveDelay(pin2->delay, degree, lastDegree2); //Need time for the servo to move to its new position
        moveDelay(pin1->delay, degree, lastDegree); //ensures they wait
       // if(debug) printf("slice:%d channel:%d value:%d degree:%3.0f \r\n", pin1->slice, pin1->channel, pin1->value, degree);
        //if(debug) printf("slice:%d channel:%d value:%d degree:%3.0f \r\n", pin2->slice, pin2->channel, pin2->value, degree);
    }
} //

void moveDelay(int myDelay, float startDeg, float endDeg){ //
    int delay = fabs(startDeg - endDeg) * myDelay;
    sleep_us(delay);
}

// void move2Delay(int myDelay,float startDeg1,float endDeg1, float startDeg2, float endDeg2){ 
//     int delay1 = fabs(startDeg1 - endDeg1) * myDelay;
//     int delay2 = fabs(startDeg2 - endDeg2) * myDelay;
//     int delay = fmax(delay1, delay2);
//     sleep_us(delay);
// }

void blink(int times, int delay){
    for(int i = 0; i < times; i++){
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        sleep_ms(delay);
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        sleep_ms(delay);
    }
}

// void blink(int times, int delay_ms){
//     for(int i = 0; i < times; i++){
//         cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
//         busy_wait_ms(delay_ms);
//         cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
//         busy_wait_ms(delay_ms);
//     }
// }

PWM enableServo(int pin){
    PWM pwm;
    pwm.pulseMax = 2500;
    pwm.pulseMin = 500;
    pwm.value = 0;
    pwm.delay = 2000;
    pwm.lastDegree = 0;
    pwm.pin = pin;
    gpio_set_function(pwm.pin, GPIO_FUNC_PWM);
    pwm.slice = pwm_gpio_to_slice_num(pin);
    pwm.channel = pwm_gpio_to_channel(pin);
    pwm_set_clkdiv(pwm.slice, 125.0f);
    pwm_set_wrap(pwm.slice, 20000);
    pwm_set_enabled(pwm.slice, true);
    return pwm;
}

void disableServos(PWM *base, PWM *joint1, PWM *joint2, PWM *gripper){
    pwm_set_enabled(base->slice, false);
    pwm_set_enabled(joint1->slice, false);
    pwm_set_enabled(joint2->slice, false);
    pwm_set_enabled(gripper->slice, false);
}

// void disable2Servos(PWM myServo1, PWM myServo2){
//     pwm_set_enabled(myServo1.slice, false);
//     pwm_set_enabled(myServo2.slice, false);
// }

void defaultPos(PWM *base, PWM *joint1, PWM *joint2, PWM *gripper){
    setBaseServo(base, 0.0);
    set2Servos(joint1, joint2, 0.0);
    setServo(gripper, 0.0);
}

void pickUp(PWM *base, PWM *joint1, PWM *joint2, PWM *gripper){
    setBaseServo(base, 190.0);
    set2Servos(joint1, joint2, 120.0);
    setServo(gripper, 180.0);
}

void deploy(PWM *base, PWM *joint1, PWM *joint2, PWM *gripper){
    setBaseServo(base, 0.0);
    set2Servos(joint1, joint2, 120.0);
    setServo(gripper, 180.0);
}

int main(){
    cyw43_arch_init();
    stdio_init_all();
    // while(!stdio_usb_connected()) {
    //     sleep_ms(100);
    // }
    // sleep_ms(5000);
   // printf("starting...\r\n");

   // Wait max 3 seconds for USB
    int timeout = 30;
    while(!stdio_usb_connected() && timeout > 0) {
        sleep_ms(100);
        timeout--;
    }

    sleep_ms(500); // small extra settle time
    
    // tell Python we're ready
    printf("READY\n");

    PWM joint1 = enableServo(JOINT_1);
    joint1.pulseMax = 2000;
    joint1.pulseMin = 1000;
    joint1.delay = 2000;

    PWM joint2 = enableServo(JOINT_2);
    joint2.pulseMax = 2000;
    joint2.pulseMin = 1000;
    joint2.delay = 2000;


    PWM base = enableServo(BASE);
    base.pulseMax = 2500;
    base.pulseMin = 500;
    base.delay = 2000;

    PWM gripper = enableServo(GRIPPER);
    gripper.pulseMax = 2500;
    gripper.pulseMin = 500;
    gripper.delay = 2000;

    // pwm_set_chan_level(gripper.slice, gripper.channel, 1500); // mid position
    // sleep_ms(3000);
    // pwm_set_chan_level(gripper.slice, gripper.channel, 2500); // full position
    // sleep_ms(3000);
  

//    while (1){
//     int c = getchar_timeout_us(100);
//     if (c != PICO_ERROR_TIMEOUT && c == 'S') {
//         blink(1, 700);
//         defaultPos(&base, &joint1, &joint2, &gripper);
//         sleep_ms(2000);
//         blink(2, 500);
//         pickUp(&base, &joint1, &joint2, &gripper);
//         sleep_ms(2000);
//         blink(3, 700);
//         defaultPos(&base, &joint1, &joint2, &gripper);
//         sleep_ms(2000);
//         blink(4, 700);
//         deploy(&base, &joint1, &joint2, &gripper);
//         sleep_ms(2000);

//         printf("DONE\n");
//         //SABRINA ADDED REMOVE IF CAUSES PROBLEMS
//         disableServos(&base, &joint1, &joint2, &gripper);
//         break;
//     } 

//    }


    // while (1){
    //     int c = getchar_timeout_us(100000); // 100ms instead of 100us
    //     if (c != PICO_ERROR_TIMEOUT && c == 'S') {
    //         blink(2, 500);
    //         setServo(&gripper, 0.0);
    //         blink(1, 700);
    //         setServo(&gripper, 180.0);
    //         sleep_ms(2000);

    //         printf("DONE\n");
    //         //SABRINA ADDED REMOVE IF CAUSES PROBLEMS
    //         disableServos(&base, &joint1, &joint2, &gripper);
    //         break;
    //     } 
    // }         

    // keep sending READY until we get any response
    while (1){
        printf("READY\n");
        sleep_ms(500);
        int c = getchar_timeout_us(100000);
        if (c != PICO_ERROR_TIMEOUT && c == 'S') {
            printf("GOT S\n");
            setServo(&gripper, 0.0);
            printf("SET 0\n");
            setServo(&gripper, 180.0);
            printf("SET 180\n");
            sleep_ms(2000);
            printf("DONE\n");
            break;
        }
    }





    //TEST CASES: SABRINA UNCOMMENT IF FUNCTIONS NOT WORKING 

    // blink(1, 700);
    // setBaseServo(&base, 0.0);
    // blink(2, 500);
    // set2Servos(&joint1, &joint2, 0.0);
    // blink(1, 700);
    // setServo(&base, 0.0);
    //blink(2, 500);
    //setServo(&gripper, 0.0);


    // blink(1, 700);
    // setBaseServo(&base, 190.0);

    // blink(2, 500);
    // set2Servos(&joint1, &joint2, 120.0);

    //blink(1, 700);
    //setServo(&gripper, 180.0);
    //sleep_ms(2000);



    // for(int i = 0; i < 180; i++){
    //     blink(1, 700);
    //     setServo(&myServo, i);
    // }
    // for(int i = 180; i > 0; i--){
    //     blink(1, 700);
    //     setServo(&myServo, i);
    // }


    // setServo(&myServo, 180.0);
    // blink(1, 700);
    // setServo(&myServo, 30.0);
    // blink(1, 700);
    // setServo(&myServo, 180.0);
    // blink(1, 500);
    // setServo(&myServo, 0.0);
    // blink(3, 800);
    // setServo(&myServo, 90.0);
    // disableServo(myServo);
    //printf("done\r\n");
}



// // #include "pico/stdlib.h"
// // #include "hardware/pwm.h"

// // #define GRIPPER 17

// // int main(){
// //     sleep_ms(2000);
    
// //     gpio_set_function(GRIPPER, GPIO_FUNC_PWM);
// //     uint slice = pwm_gpio_to_slice_num(GRIPPER);
// //     uint channel = pwm_gpio_to_channel(GRIPPER);
// //     pwm_set_clkdiv(slice, 125.0f);
// //     pwm_set_wrap(slice, 20000);
// //     pwm_set_enabled(slice, true);

// //     pwm_set_chan_level(slice, channel, 500);  // 0 degrees
// //     sleep_ms(3000);
// //     pwm_set_chan_level(slice, channel, 2500); // 180 degrees
// //     sleep_ms(3000);

// //     while(1);
// // }

// int main(){
//     stdio_init_all();
//     int timeout = 30;
//     while(!stdio_usb_connected() && timeout > 0) {
//         sleep_ms(100);
//         timeout--;
//     }
//     sleep_ms(1000);
    
//     while(1){
//         printf("hello\n");
//         sleep_ms(1000);
//     }
// }