// #include <stdio.h>
// #include "pico/stdlib.h"
// #include "hardware/pwm.h"
// #include "hardware/clocks.h"
// #include "claw_controllerv3.h"
// #include <math.h>


// #define PIN_OUT 16 //change to the actual pin using 
// #define debug true

// void setServo(PWM *pin,float degree){
//     if (0 > degree > 180){
//         printf("Please use value of 0-180\n\r");
//     }else{
//         //convert the degree to uSec ticks
//         int value = (int)((((float)(pin->pulseMax-pin->pulseMin))*(degree/180.0))+pin->pulseMin);
//         //Save the lastDegree so we can determine delay for movement of the servo
//         int lastDegree = pin->lastDegree;
//         //Store the current degree as lastDegree
//         pin->lastDegree = degree;
//         //Store the last value, probably don't need 
//         pin->value = value;
//         //Set the pwm level, this is only for the first pin->pulseMax uSecs of the frame.
//         pwm_set_chan_level(pin->slice,pin->channel, pin->value);
//         //Need time for the servo to move to its new position
//         moveDelay(pin->delay,degree,lastDegree);
//         if(debug) printf("slice:%d channel:%d value:%d degree:%3.0f \r\n",pin->slice,pin->channel, pin->value, degree);
//     }
// }

// void moveDelay(int myDelay,float startDeg,float endDeg){
//     //Get the absolute value of the difference between the start and end point, multiple by delay
//     int delay = fabs(startDeg - endDeg) * myDelay;
//     sleep_us(delay);
// }

// PWM enableServo(int pin){
//     //Set defaults
//     PWM pwm;
//     pwm.pulseMax=2600; //change to 2600 for rgb 
//     pwm.pulseMin=400; //change to 400 for rgb 
//     pwm.value = 0;
//     pwm.delay = 2000;
//     pwm.lastDegree = 0;
//     pwm.pin = pin;
//     //Turn the pin into a pwm pin
//     gpio_set_function(pwm.pin, GPIO_FUNC_PWM);
//     //Need to store the slice and channel
//     pwm.slice = pwm_gpio_to_slice_num(pin);
//     pwm.channel = pwm_gpio_to_channel(pin);
//     //Set the clock to uSecs
//     pwm_set_clkdiv(pwm.slice,125.0f);  
//     //wrap every 20000 uSecs or 1 frame, first 2500 uSecs determine duty cycle or how far the servo moves...
//     pwm_set_wrap(pwm.slice,20000);      
//     //Enable the servo
//     pwm_set_enabled(pwm.slice,true);
//     return pwm;
// }

// void disableServo(PWM myServo){
//     //Disable the servo
//     pwm_set_enabled(myServo.slice,false);
// }


// int main()
// {
//     stdio_init_all();
//     sleep_ms(5000);
//     printf("starting...\r\n");

//     PWM myServo = enableServo(PIN_OUT);
//     myServo.pulseMax=2400;
//     myServo.delay = 2000;
//     for(int i = 0 ; i < 180;i++){
//         setServo(&myServo,i);
//     }
//     for(int i = 180 ; i > 0;i--){
//         setServo(&myServo,i);
//     }
//     setServo(&myServo,0.0);
//     setServo(&myServo,90.0);
//     setServo(&myServo,180.0);
//     setServo(&myServo,90.0);
//     setServo(&myServo,0.0);
//     disableServo(myServo);
//     printf("done\r\n");
// }


// #include <stdio.h>
// #include "pico/stdlib.h"
// #include "hardware/pwm.h"
// #include "hardware/clocks.h"
// #include "pico/cyw43_arch.h"
// #include "claw_controllerv3.h"
// #include <math.h>

// #define PIN_OUT 14
// #define debug true

// void setServo(PWM *pin, float degree){ 
//     if (degree < 0.0f || degree > 180.0f){
//         printf("Please use value of 0-180\n\r");
//     } else {
//         int value = (int)((((float)(pin->pulseMax-pin->pulseMin))*(degree/180.0))+pin->pulseMin);
//         int lastDegree = pin->lastDegree; //
//         pin->lastDegree = degree; //
//         pin->value = value; 
//         pwm_set_chan_level(pin->slice, pin->channel, pin->value);
//         moveDelay(pin->delay, degree, lastDegree); //Need time for the servo to move to its new position
//         if(debug) printf("slice:%d channel:%d value:%d degree:%3.0f \r\n", pin->slice, pin->channel, pin->value, degree);
//     }
// } //

// void moveDelay(int myDelay, float startDeg, float endDeg){ //
//     int delay = fabs(startDeg - endDeg) * myDelay;
//     sleep_us(delay);
// }

// void blink(int times, int delay){
//     for(int i = 0; i < times; i++){
//         cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
//         sleep_ms(delay);
//         cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
//         sleep_ms(delay);
//     }
// }

// PWM enableServo(int pin){
//     PWM pwm;
//     pwm.pulseMax = 2500;
//     pwm.pulseMin = 500;
//     pwm.value = 0;
//     pwm.delay = 2000;
//     pwm.lastDegree = 0;
//     pwm.pin = pin;
//     gpio_set_function(pwm.pin, GPIO_FUNC_PWM);
//     pwm.slice = pwm_gpio_to_slice_num(pin);
//     pwm.channel = pwm_gpio_to_channel(pin);
//     pwm_set_clkdiv(pwm.slice, 125.0f);
//     pwm_set_wrap(pwm.slice, 20000);
//     pwm_set_enabled(pwm.slice, true);
//     return pwm;
// }

// void disableServo(PWM myServo){
//     pwm_set_enabled(myServo.slice, false);
// }

// int main(){
//     cyw43_arch_init();
//     stdio_init_all();
//     sleep_ms(5000);
//     printf("starting...\r\n");

//     PWM myServo = enableServo(PIN_OUT);
//     myServo.pulseMax = 2500;
//     myServo.pulseMin = 500;
//     myServo.delay = 2000;

//     // for(int i = 0; i < 180; i++){
//     //     blink(1, 700);
//     //     setServo(&myServo, i);
//     // }
//     // for(int i = 180; i > 0; i--){
//     //     blink(1, 700);
//     //     setServo(&myServo, i);
//     // }

//     blink(1, 700);
//     setServo(&myServo, 30.0);
//     blink(1, 500);
//     setServo(&myServo, 180.0);
//     blink(1, 700);
//     setServo(&myServo, 30.0);
//     blink(1, 700);
//     setServo(&myServo, 180.0);
//     // blink(1, 500);
//     // setServo(&myServo, 0.0);
//     // blink(3, 800);
//     // setServo(&myServo, 90.0);
//     // disableServo(myServo);
//     printf("done\r\n");
// }


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
#define debug true

void setServo(PWM *pin, float degree){ 
    if (degree < 0.0f || degree > 180.0f){
        printf("Please use value of 0-180\n\r");
    } else {
        int value = (int)((((float)(pin->pulseMax-pin->pulseMin))*(degree/180.0))+pin->pulseMin);
        int lastDegree = pin->lastDegree; //
        pin->lastDegree = degree; //
        pin->value = value; 
        pwm_set_chan_level(pin->slice, pin->channel, pin->value);
        moveDelay(pin->delay, degree, lastDegree); //Need time for the servo to move to its new position
        if(debug) printf("slice:%d channel:%d value:%d degree:%3.0f \r\n", pin->slice, pin->channel, pin->value, degree);
    }
} 

void setBaseServo(PWM *pin, float degree){ 
    if (degree < 0.0f || degree > 195.0f){
        printf("Please use value of 0-195\n\r");
    } else {
        int value = (int)((((float)(pin->pulseMax-pin->pulseMin))*(degree/195.0))+pin->pulseMin);
        int lastDegree = pin->lastDegree; //
        pin->lastDegree = degree; //
        pin->value = value; 
        pwm_set_chan_level(pin->slice, pin->channel, pin->value);
        moveDelay(pin->delay, degree, lastDegree); //Need time for the servo to move to its new position
        if(debug) printf("slice:%d channel:%d value:%d degree:%3.0f \r\n", pin->slice, pin->channel, pin->value, degree);
    }
} 


void set2Servos(PWM *pin1, PWM *pin2, float degree){ 
    if (degree < 0.0f || degree > 180.0f){
        printf("Please use value of 0-180\n\r");
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
        if(debug) printf("slice:%d channel:%d value:%d degree:%3.0f \r\n", pin1->slice, pin1->channel, pin1->value, degree);
        if(debug) printf("slice:%d channel:%d value:%d degree:%3.0f \r\n", pin2->slice, pin2->channel, pin2->value, degree);
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

void disableServo(PWM myServo){
    pwm_set_enabled(myServo.slice, false);
}

int main(){
    cyw43_arch_init();
    stdio_init_all();
    sleep_ms(5000);
    printf("starting...\r\n");

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

    // for(int i = 0; i < 180; i++){
    //     blink(1, 700);
    //     setServo(&myServo, i);
    // }
    // for(int i = 180; i > 0; i--){
    //     blink(1, 700);
    //     setServo(&myServo, i);
    // }

    blink(1, 700);
    setBaseServo(&base, 0.0);
    blink(2, 500);
    set2Servos(&joint1, &joint2, 0.0);
    blink(1, 700);
    setServo(&base, 0.0);


    blink(1, 700);
    setBaseServo(&base, 190.0);

    blink(2, 500);
    set2Servos(&joint1, &joint2, 120.0);

    blink(1, 700);
    setServo(&gripper, 180.0);

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
    printf("done\r\n");
}

