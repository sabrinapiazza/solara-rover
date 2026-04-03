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
    float lastDegree2;
} PWM;


void setServo(PWM *pin,float degree);
void set2Servos(PWM *pin1, PWM *pin2, float degree);
void setBaseServo(PWM *pin, float degree);
PWM enableServo(int pin);
void disableServo(PWM myServo);
int calculateDelay(PWM myServo,int mSecPerDegree);
void moveDelay(int myDelay,float startDeg,float endDeg);
// void move2Delay(int myDelay,float startDeg1,float endDeg1, float startDeg2, float endDeg2);