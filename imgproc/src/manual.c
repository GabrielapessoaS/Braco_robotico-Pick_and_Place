#include <pigpio.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <math.h>
#include "inv_kinematics.h"

int sv=0;
int run=1;
int usbase = 0, usa1 = 0, usa2 = 0;

void stop() {
	run = 0;
}

void ease_func(){
  int pulse_base= gpioGetServoPulsewidth(SERVO_BASE);
  int pulse_x= gpioGetServoPulsewidth(SERVO_A1);
  int pulse_y= gpioGetServoPulsewidth(SERVO_A2);

  //printf("base: %d -> %d\n", pulse_base, usbase);
  //printf("x: %d -> %d\n", pulse_x, usa1);
  //printf("y: %d -> %d\n", pulse_y, usa2);

  if( pulse_base < usbase)
	  gpioServoBound(SERVO_BASE, pulse_base + SPEED);
  else if( pulse_base > usbase)
	  gpioServoBound(SERVO_BASE, pulse_base - SPEED);

  if( pulse_x < usa1)
	  gpioServoBound(SERVO_A1, pulse_x + SPEED);
  else if( pulse_x > usa1)
	  gpioServoBound(SERVO_A1, pulse_x - SPEED);

  if( pulse_y < usa2)
	  gpioServoBound(SERVO_A2, pulse_y + SPEED);
  else if( pulse_y > usa2)
	  gpioServoBound(SERVO_A2, pulse_y - SPEED);
}


int main(int argc, char **argv){
	double dg_a1=90, dg_a2=0, dg_base=90;
	double X, Y, Z;

	usbase = degree_to_us(dg_base, SERVO_BASE);
	usa1 = degree_to_us(dg_a1, SERVO_A1);
	usa2 = degree_to_us(dg_a2, SERVO_A2);

	if(gpioInitialise() < 0)
		exit(1);

	gpioSetSignalFunc(SIGINT, stop);

	//gpioSetPWMrange(BOBINA, 100);
	//gpioSetPWMfrequency(BOBINA, 100);

	//gpioSetMode(BUT_SEL, PI_INPUT);
	//gpioSetMode(BUT_INC, PI_INPUT);
	//gpioSetMode(BUT_DEC, PI_INPUT);

	//gpioSetPWMfrequency(SERVO_BASE, 50);
	//gpioSetPWMfrequency(SERVO_A1, 50);
	//gpioSetPWMfrequency(SERVO_A2, 50);
	//
	gpioServoBound(SERVO_BASE, degree_to_us(dg_base, 0));
	gpioServoBound(SERVO_A1, degree_to_us(dg_a1, 1));
	gpioServoBound(SERVO_A2, degree_to_us(dg_a2, 2));

	gpioSetTimerFunc(0, 10, ease_func);

	while(run){
		fprintf(stdout, "Insira os valores X, Y, Z\n");
		fprintf(stdout, "X: ");
		scanf("%lf", &X);
		fprintf(stdout, "Y: ");
		scanf("%lf", &Y);
		fprintf(stdout, "Z: ");
		scanf("%lf", &Z);

		inverse_kinematics(X, Y, Z, &usbase, &usa1, &usa2); 
		
	}
	gpioTerminate();
	return 0;
}


