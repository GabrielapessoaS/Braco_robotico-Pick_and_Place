#include <pigpio.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <math.h>

//#define BUT_SEL	17
//#define BUT_INC	27
//#define BUT_DEC	22
//#define BOBINA	4

#define FELIPE		1
//#define GABRIEL	1

#ifdef GABRIEL

#define SERVO_BASE	27
#define SERVO_A2	17
#define SERVO_A1	4
//#define BOBINA	6 

#define MIN_BASE	500
#define MAX_BASE	2500

#define MIN_X	500
#define MAX_X	1550

#define MIN_Y	1000
#define MAX_Y	2500
#endif

#ifdef FELIPE
#define SERVO_BASE		19
#define SERVO_A2		13
#define SERVO_A1		12

#define MIN_BASE	500
#define MAX_BASE	2500

#define MIN_X	500
#define MAX_X	1550

#define MIN_Y	1000
#define MAX_Y	2500
#endif


#define SPEED 4

const double a1 = 8.0;
const double a2 =8.0; 

#define MAX_LEN	a1+a2
#define MIN_LEN 1

int sv=0;
int run=1;
int usbase = 0, usa1 = 0, usa2 = 0;

void stop(int signum){
  run = 0;
}

int gpioServoBound(int servo, int us) {
	switch(servo) {
		case SERVO_BASE:
			if( us < MIN_BASE ) us = MIN_BASE;
			if( us > MAX_BASE ) us = MAX_BASE;
			break;
		case SERVO_A2:
			if( us < MIN_Y ) us = MIN_Y;
			if( us > MAX_Y ) us = MAX_Y;
			break;
		case SERVO_A1:
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
#ifdef GABRIEL
			return (int)(9.14*degree );
#endif
#ifdef FELIPE
			return (int)(11.1111*degree + 500);
#endif

		case SERVO_A1:
			//return ((int)(11.11*(*degree)+ 500.0));
#ifdef GABRIEL
			return ((int)(9.14*(180-degree) + 547.4));
#endif
#ifdef FELIPE
			return (int)(-11.1111*degree + 1500);
#endif
		case SERVO_A2:
			//return ((int)(-15.55*(*degree - dg_j) + 1900.0));
#ifdef GABRIEL
			return ((int)(9.14*degree + 1950.0));
#endif
#ifdef FELIPE
			return (int)(10.7142*degree + 1800);
#endif
	}
	return 0;
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


void inverse_kinematics(double x, double y, double z, double theta1, double theta2, double theta3){


  printf("Cinematica inversa para (%lf, %lf)...\n", x, z);

  if((sqrt(y*y + z*z) > MAX_LEN) || (sqrt(y*y + z*z) < MIN_LEN)) {
	  printf("Ponto alem do alcance.\n");
	  return;
  }
  theta1 = atan(z/y) + acos((y*y + z*z + a1*a1 - a2*a2)/(2*a1*sqrt(y*y + z*z)));
  theta2 = theta1 - acos((y*y + z*z - a1*a1 - a2*a2) / (2.0*a1*a2));
	theta3 = atan2(y,x);

	/*if(theta3 < 0){
		theta3 = -theta3;
		theta3 += 90.0;
	}
	*/
	

  theta1 = 180.0*theta1/M_PI;
  theta2 = 180.0*theta2/M_PI;
  theta3 = 180.0*theta3/M_PI;

  printf("theta1 = %lf\n", theta1);
  printf("theta2 = %lf\n", theta2);
	printf("theta3 = %lf\n", theta3);


  usa1 = degree_to_us(theta1, SERVO_A1);
  usa2 = degree_to_us(theta2, SERVO_A2);
	usbase = degree_to_us(theta3, SERVO_BASE);
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

	gpioSetPWMrange(BOBINA, 100);
	gpioSetPWMfrequency(BOBINA, 100);

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

		inverse_kinematics(X, Y, Z, dg_a1, dg_a2, dg_base); 
	}
	gpioTerminate();
	return 0;
}


