#include <stdio.h>
#include <stdlib.h>
#include <signal.h>

#include <pigpio.h>

/*
# servo_demo.c
# 2016-10-08
# Public Domain

gcc -Wall -pthread -o servo_demo servo_demo.c -lpigpio

sudo ./servo_demo          # Send servo pulses to GPIO 4.
sudo ./servo_demo 23 24 25 # Send servo pulses to GPIO 23, 24, 25.
*/

//#define NUM_GPIO 32

#define SERVO_BASE 12
#define SERVO_Y 13
#define SERVO_X 19
#define BOBINA 6 

//#define MIN_WIDTH 1000
//#define MAX_WIDTH 2000

int width;
int run = 1;

void stop(int signum){
  run = 0;
}

int main (int argc, char ** argv){

  int i;

  if (gpioInitialise()< 0) return -1;

  gpioSetSignalFunc(SIGINT, stop);

  gpioSetMode(BOBINA, PI_OUTPUT);
  gpioWrite(BOBINA, 1);

  
  printf("Testando o valor para os servos");


  while(run){

      gpioServo(SERVO_X, 500);
      gpioServo(SERVO_Y, 500);
      gpioServo(SERVO_BASE, 2000);

      time_sleep(0.1);

  }

  gpioServo(SERVO_BASE, 0);
  gpioServo(SERVO_X, 0);
  gpioServo(SERVO_Y, 0);
  gpioWrite(BOBINA, 0);
  gpioTerminate();

  return 0;
}


