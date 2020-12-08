#ifndef INV_KINEMATICS_H
#define INV_KINEMATICS_H

#include <pigpio.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <math.h>

#define SERVO_BASE	27
#define SERVO_Y		17
#define SERVO_X		4
//#define BOBINA	6 

#define MIN_BASE	500
#define MAX_BASE	2500

#define MIN_X	1000
#define MAX_X	1530

#define MIN_Y	1000
#define MAX_Y	2500

#define SPEED 1

#define a1 8.0
#define a2 8.0

#define MAX_LEN	(a1+a2)
#define MIN_LEN 1

int gpioServoBound(int servo, int us);

int degree_to_us(double degree, int servo);

void ease_func();

void inverse_kinematics(double x, double y, double theta1, double theta2);


#endif

