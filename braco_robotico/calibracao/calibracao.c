#include<pigpio.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>

#define BUT_SEL 17
#define BUT_INC   27
#define BUT_DEC   22
//#define BOBINA  4

#define SERVO_BASE 12
#define SERVO_Y 13
#define SERVO_X 19
//#define BUT_BOBINA 6 

int run=1;

void stop(int signum){
  run = 0;
}

void inc_dec_func(){

}

void read_button(int *i, int *state, int *pin){
    if(gpioRead(BUT_INC)==0){
        time_sleep(0.01);
        if(gpioRead(BUT_INC)>0)
            return;

        *i++;
        if (*i>2500)
            *i=2500;
        }
    else if(gpioRead(BUT_DEC) ==0){
        if(gpioRead(BUT_DEC)>0)
            return;

        time_sleep(0.01);
        *i--;
        if (*i<500)
            *i=500;
    }

    else if(gpioRead(BUT_SEL) ==0){
        if(gpioRead(BUT_SEL)>0)
            return;        
        time_sleep(0.01);
        *state++;
        if (*state>2)
            *state=0;
    }

            
    time_sleep(0.2);
    gpioServo(*pin, *i);

    return;

}







int main(int argc, char **argv){
    int i=500,j=500,k=500;
    int servo_sel=0;
    
    if(gpioInitialise() < 0)
        exit(1);
    
    gpioSetSignalFunc(SIGINT, stop);

    //gpioSetMode(BOBINA, PI_OUTPUT);
    //gpioWrite(BOBINA, 1);

    gpioSetMode(BUT_SEL, PI_INPUT);
    gpioSetMode(BUT_INC, PI_INPUT);
    gpioSetMode(BUT_DEC, PI_INPUT);


    while(run){

        gpioServo(SERVO_X, 500);
        gpioServo(SERVO_Y, 500);
      
        
        switch(servo_sel){
            case(0):
                read_button(i, servo_sel, SERVO_BASE);
            break;

            case(1):
                read_button(j, servo_sel, SERVO_X);
            break;

            case(2):
                read_button(k, servo_sel, SERVO_Y);
        }



    }
    gpioTerminate();
    return 0;
}


