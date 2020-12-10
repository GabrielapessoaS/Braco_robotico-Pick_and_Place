#include <pigpio.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <math.h>

//#define BUT_SEL	17
//#define BUT_INC	27
//#define BUT_DEC	22
//#define BOBINA	4

//pinos braco felipe 

#define SERVO_BASE 19
#define SERVO_Y 13
#define SERVO_X 12
#define BOBINA	6 

#define MIN_BASE	500
#define MAX_BASE	2500

/* Constantes felipe
#define MIN_X	500
#define MAX_X	1550

#define MIN_Y	1000
#define MAX_Y	2500
*/
#define MIN_X	900
#define MAX_X	2300

#define MIN_Y	500
#define MAX_Y	2500


#define SPEED 4

const double a1 = 8.0;
const double a2 =8.0; 

#define MAX_LEN	a1+a2
#define MIN_LEN 1

int sv=0;
int run=1;
int usbase = 0, usx = 0, usy = 0;

void stop(int signum){
  run = 0;
}

int gpioServoBound(int servo, int us) {
	switch(servo) {
		case SERVO_BASE:
			if( us < MIN_BASE ) us = MIN_BASE;
			if( us > MAX_BASE ) us = MAX_BASE;
			break;
		case SERVO_Y:
			if( us < MIN_Y ) us = MIN_Y;
			if( us > MAX_Y ) us = MAX_Y;
			break;
		case SERVO_X:
			if( us < MIN_X ) us = MIN_X;
			if( us > MAX_X ) us = MAX_X;
			break;
	}
	//printf("bound: %u us\n", us);
	return gpioServo(servo, us);
}
			


int degree_to_us(double degree, int servo){
	switch(servo){
		case SERVO_BASE:
			//k=((int)(-15.55*(dg_k- *degree) + 1900.0));
			//return ((int)(-11.11*(*degree) + 1500.0));
			return (int)(9.14*degree );
			//return ((int)11.1111*degree + 500);
		case SERVO_X:
			//return ((int)(11.11*(*degree)+ 500.0));
			return ((int)(9.14*(180-degree) + 547.4));
			//return ((int)-11.1111*(degree) + 1500);
		case SERVO_Y:
			//return ((int)(-15.55*(*degree - dg_j) + 1900.0));
			return ((int)(9.14*degree + 2100.0));
			//return ((int)10.7142*degree + 1800);
	}
	return 0;
}

void ease_func(){
  int pulse_base= gpioGetServoPulsewidth(SERVO_BASE);
  int pulse_x= gpioGetServoPulsewidth(SERVO_X);
  int pulse_y= gpioGetServoPulsewidth(SERVO_Y);

  //printf("base: %d -> %d\n", pulse_base, usbase);
  //printf("x: %d -> %d\n", pulse_x, usx);
  //printf("y: %d -> %d\n", pulse_y, usy);

  if( pulse_base < usbase)
	  gpioServoBound(SERVO_BASE, pulse_base + SPEED);
  else if( pulse_base > usbase)
	  gpioServoBound(SERVO_BASE, pulse_base - SPEED);

  if( pulse_x < usx)
	  gpioServoBound(SERVO_X, pulse_x + SPEED);
  else if( pulse_x > usx)
	  gpioServoBound(SERVO_X, pulse_x - SPEED);

  if( pulse_y < usy)
	  gpioServoBound(SERVO_Y, pulse_y + SPEED);
  else if( pulse_y > usy)
	  gpioServoBound(SERVO_Y, pulse_y - SPEED);
}


void inverse_kinematics(double x, double y, double theta1, double theta2){

  printf("Cinematica inversa para (%lf, %lf)...\n", x, y);

  if((sqrt(x*x + y*y) > MAX_LEN) || (sqrt(x*x + y*y) < MIN_LEN)) {
	  printf("Ponto alem do alcance.\n");
	  return;
  }
  theta1 = atan(y/x) + acos((x*x + y*y + a1*a1 - a2*a2)/(2*a1*sqrt(x*x + y*y)));
  theta2 = theta1 - acos((x*x + y*y - a1*a1 - a2*a2) / (2.0*a1*a2));

  theta1 = 180.0*theta1/M_PI;
  theta2 = 180.0*theta2/M_PI;

  printf("theta1 = %lf\n", theta1);
  printf("theta2 = %lf\n", theta2);


  usx = degree_to_us(theta1, SERVO_X);
  usy = degree_to_us(theta2, SERVO_Y);
}

int main(int argc, char **argv){
	double dg_x=90, dg_y=0, dg_base=90;
	double X, Y;

	usbase = degree_to_us(dg_base, SERVO_BASE);
	usx = degree_to_us(dg_x, SERVO_X);
	usy = degree_to_us(dg_y, SERVO_Y);

	if(gpioInitialise() < 0)
		exit(1);

	gpioSetSignalFunc(SIGINT, stop);

	gpioSetPWMrange(BOBINA, 100);
	gpioSetPWMfrequency(BOBINA, 100);

	//gpioSetMode(BUT_SEL, PI_INPUT);
	//gpioSetMode(BUT_INC, PI_INPUT);
	//gpioSetMode(BUT_DEC, PI_INPUT);

	//gpioSetPWMfrequency(SERVO_BASE, 50);
	//gpioSetPWMfrequency(SERVO_X, 50);
	//gpioSetPWMfrequency(SERVO_Y, 50);
	//
	gpioServoBound(SERVO_BASE, degree_to_us(dg_base, 0));
	gpioServoBound(SERVO_X, degree_to_us(dg_x, 1));
	gpioServoBound(SERVO_Y, degree_to_us(dg_y, 2));

	gpioSetTimerFunc(0, 10, ease_func);

	while(run){

		gpioPWM(BOBINA, 30);

		fprintf(stdout, "Insira os valores X e Y\n");
		fprintf(stdout, "X: ");
		scanf("%lf", &X);
		fprintf(stdout, "Y: ");
		scanf("%lf", &Y);
		

		inverse_kinematics(X, Y, dg_x, dg_y); 
	}
	gpioTerminate();
	return 0;
}


