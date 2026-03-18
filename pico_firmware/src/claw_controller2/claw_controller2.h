#ifndef CLAW_CONTROLLER2_H
#define CLAW_CONTROLLER2_H

typedef struct{
    int pin;
    int slice;
    int channel;
    int value;
    bool enabled;
    int pulseMax;
    int pulseMin;
    int delay;
    float lastDegree;
} PWM;


void setServo(PWM *pin,float degree);
PWM enableServo(int pin);
void disableServo(PWM myServo);
//int calculateDelay(PWM myServo,int mSecPerDegree);
void moveDelay(int myDelay,float startDeg,float endDeg);


#endif