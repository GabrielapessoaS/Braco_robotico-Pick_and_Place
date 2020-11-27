#include <pigpio.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <math.h>

#define BUT_SEL 17
#define BUT_INC   27
#define BUT_DEC   22
//#define BOBINA  4

#define SERVO_BASE 19
#define SERVO_Y 13
#define SERVO_X 12
//#define BUT_BOBINA 6 

#define SPEED 2.0

int run=1;
int i=0,j=0,k=0;
int dg_i=90, dg_j=90, dg_k=90;

void stop(int signum){
  run = 0;
}


int degree_to_us(int *degree, int *state){

  switch(*state){

    case 0:
      return ((int)(11.11*(*degree)+ 500.0));
      break;

    case 1:
      k=((int)(-15.55*(dg_k- *degree) + 1900.0));
      return ((int)(-11.11*(*degree) + 1500.0));
      break;

    case 2:
      return ((int)(-15.55*(*degree - dg_j) + 1900.0));
      break;

    default:
      *state= 0;
  }
  return 0.0;

}

void read_button(int *i, int *dg, int *state, int pin){
    if(gpioRead(BUT_INC)==0){
        time_sleep(0.01);
        if(gpioRead(BUT_INC)>0)
            return;

        *dg += 5;
        if (*dg>155 && *state ==2)
            *dg=155;
        else if(*state ==0 && *dg>180)
            *dg = 180;
        else if(*state == 1 && *dg > 90)
          *dg=90;
        *i = degree_to_us(dg, state);
        fprintf(stderr, "Valor do grau=%d\n", *dg);
        fprintf(stderr, "Valor do pulso=%d\n", *i);

        }
    else if(gpioRead(BUT_DEC) ==0){
        time_sleep(0.01);
        if(gpioRead(BUT_DEC)>0)
            return;

        *dg -= 5;
        if (*dg<0 && *state < 2)
            *dg=0;
        else if (*dg<10 && *state ==2)
          *dg=10;
        *i = degree_to_us(dg, state);

        //fprintf(stderr, "Valor do pulso=%d\n", gpioGetServoPulsewidth(pin));
        fprintf(stderr, "Valor do grau=%d\n", *dg);
        fprintf(stderr, "Valor do pulso=%d\n", *i);
    }

    else if(gpioRead(BUT_SEL) ==0){
        time_sleep(0.01);
        if(gpioRead(BUT_SEL)>0)
            return;        
        *state += 1;
        if (*state>2)
            *state=0;



        fprintf(stderr, "Valor do estado=%d\n", *state);
    }

            
    time_sleep(0.2);

    return;

}


void ease_func(){

  float pulse_base= (float)gpioGetServoPulsewidth(SERVO_BASE);
  float pulse_x= (float)gpioGetServoPulsewidth(SERVO_X);
  float pulse_y= (float)gpioGetServoPulsewidth(SERVO_Y);
  if(pulse_base < i){
        //fprintf(stderr, "pulse_base<i=%d\n", (int)(pulse_base + SPEED));
    if ((pulse_base+SPEED)>2500){
      gpioServo(SERVO_BASE, 2500);
    }
    else{
      gpioServo(SERVO_BASE, (unsigned int)(pulse_base + SPEED));
      fprintf(stderr, "pulse_base<k=%d\n", (int)(pulse_base + SPEED));
        //fprintf(stderr, "pulse_x<j=%d\n", (int)(pulse_x + SPEED));
    }

  }
  else if(pulse_base > i){
    if ((pulse_base-SPEED)<500){
      gpioServo(SERVO_BASE, 500);
    }
    else{
      gpioServo(SERVO_BASE, (unsigned int)(pulse_base - SPEED));
      fprintf(stderr, "pulse_base<k=%d\n", (int)(pulse_base - SPEED));
        //fprintf(stderr, "pulse_x<j=%d\n", (int)(pulse_x + SPEED));
    }

  }
  if(pulse_x < j){
    if ((pulse_x+SPEED)>1500){
      gpioServo(SERVO_X, 1500);
    }
    else{
      gpioServo(SERVO_X, (unsigned int)(pulse_x + SPEED));
      fprintf(stderr, "pulse_x<k=%d\n", (int)(pulse_x + SPEED));
        //fprintf(stderr, "pulse_x<j=%d\n", (int)(pulse_x + SPEED));
    }

  }
  else if(pulse_x > j){
    if ((pulse_x-SPEED)<500){
      gpioServo(SERVO_X,500);
    }
    else{
      gpioServo(SERVO_X, (unsigned int)(pulse_x - SPEED));
      fprintf(stderr, "pulse_x<k=%d\n", (int)(pulse_x - SPEED));
        //fprintf(stderr, "pulse_x<j=%d\n", (int)(pulse_x + SPEED));
    }

  }
  if (pulse_y < k){
    if ((pulse_y+SPEED)>1900){
      gpioServo(SERVO_Y, 1900);
    }
    else{
      gpioServo(SERVO_Y, (unsigned int)(pulse_y + SPEED));
      fprintf(stderr, "pulse_y<k=%d\n", (int)(pulse_y + SPEED));

    }

  }
  else if (pulse_y > k){
    if((pulse_y - SPEED)<500){

      gpioServo(SERVO_Y, 500);
    }
    else{
      gpioServo(SERVO_Y, (unsigned int)(pulse_y - SPEED));
      fprintf(stderr, "pulse_y<k=%d\n", (int)(pulse_y - SPEED));

    }

  }
}


void inverse_knematics(){
  
}<++>




int main(int argc, char **argv){
    int servo_sel=0;
    
    if(gpioInitialise() < 0)
        exit(1);
    
    gpioSetSignalFunc(SIGINT, stop);

    //gpioSetMode(BOBINA, PI_OUTPUT);
    //gpioWrite(BOBINA, 1);

    gpioSetMode(BUT_SEL, PI_INPUT);
    gpioSetMode(BUT_INC, PI_INPUT);
    gpioSetMode(BUT_DEC, PI_INPUT);

    //gpioSetPWMfrequency(SERVO_BASE, 50);
    //gpioSetPWMfrequency(SERVO_X, 50);
    //gpioSetPWMfrequency(SERVO_Y, 50);
    //
    gpioServo(SERVO_BASE, degree_to_us(&dg_i, &servo_sel));
    servo_sel+=1;
    gpioServo(SERVO_X, degree_to_us(&dg_j, &servo_sel));
    servo_sel+=1;
    gpioServo(SERVO_Y, degree_to_us(&dg_k, &servo_sel));
    gpioSetTimerFunc(0, 10, ease_func);
    
    servo_sel=0;

    while(run){

      
        
        switch(servo_sel){
            case(0):
                read_button(&i, &dg_i, &servo_sel, SERVO_BASE);
            break;

            case(1):
                read_button(&j, &dg_j, &servo_sel, SERVO_X);
            break;

            case(2):
                read_button(&k, &dg_k, &servo_sel, SERVO_Y);
        }



    }
    gpioTerminate();
    return 0;
}


