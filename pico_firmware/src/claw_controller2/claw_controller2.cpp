#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "claw_controller2.h"
#include <math.h>
#include <iostream>

using namespace std; 


#define PIN_OUT 17 //gpio pin 17 
#define debug true

void setServo(PWM *pin,float degree){
    if (degree < 0 || degree > 180){
        cout << ("Please use value of 0-180") << endl;
    }else{
        //convert the degree to uSec ticks
        int value = (int)((((float)(pin->pulseMax-pin->pulseMin))*(degree/180.0))+pin->pulseMin);
        //Save the lastDegree so we can determine delay for movement of the servo
        int lastDegree = pin->lastDegree;
        //Store the current degree as lastDegree
        pin->lastDegree = degree;
        //Store the last value, probably don't need 
        pin->value = value;
        //Set the pwm level, this is only for the first pin->pulseMax uSecs of the frame.
        pwm_set_chan_level(pin->slice,pin->channel, pin->value);
        //Need time for the servo to move to its new position
        moveDelay(pin->delay,degree,lastDegree);
        if(debug) printf("slice:%d channel:%d value:%d degree:%3.0f \r\n",pin->slice,pin->channel, pin->value, degree);
    }
}

void moveDelay(int myDelay,float startDeg,float endDeg){
    //Get the absolute value of the difference between the start and end point, multiple by delay
    int delay = fabs(startDeg - endDeg) * myDelay;
    sleep_us(delay);
}

PWM enableServo(int pin){
    //Set defaults
    PWM pwm;
    pwm.pulseMax=2200; //savox change; 2000 for s3003
    pwm.pulseMin=800; //savox change; 1000 for s3003 
    pwm.value = 0;
    pwm.delay = 2000;
    pwm.lastDegree = 0;
    pwm.pin = pin;
    //Turn the pin into a pwm pin
    gpio_set_function(pwm.pin, GPIO_FUNC_PWM);
    //Need to store the slice and channel
    pwm.slice = pwm_gpio_to_slice_num(pin); //what is a slice? 4 slices with 2 channels each, so 8 total pwm outputs
    pwm.channel = pwm_gpio_to_channel(pin); //each slice has 2 channels, A and B, this determines which one of those we are using
    //Set the clock to uSecs
    pwm_set_clkdiv(pwm.slice,125.0f);  
    //wrap every 20000 uSecs or 1 frame, first 2500 uSecs determine duty cycle or how far the servo moves...
    pwm_set_wrap(pwm.slice,20000);      
    //Enable the servo
    pwm_set_enabled(pwm.slice,true);
    return pwm;
}

void disableServo(PWM myServo){
    //Disable the servo
    pwm_set_enabled(myServo.slice,false);
}


int main()
{
    stdio_init_all();
    sleep_ms(5000);
    cout << ("starting...") << endl;

    PWM myServo = enableServo(PIN_OUT);
    myServo.pulseMax=2400;
    myServo.delay = 2000;
    for(int i = 0 ; i < 180;i++){ //move from 0 to 180 degrees
        setServo(&myServo,i);
    }
    for(int i = 180 ; i > 0;i--){ //move from 180 to 0 degrees 
        setServo(&myServo,i);
    }
    setServo(&myServo,0.0); 
    setServo(&myServo,90.0);
    setServo(&myServo,180.0);
    setServo(&myServo,90.0);
    setServo(&myServo,0.0);
    disableServo(myServo);
    printf("done\r\n");
}

//raspberry pi forum guy 