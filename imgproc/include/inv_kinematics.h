#ifndef INV_KINEMATICS_H
#define INV_KINEMATICS_H

#include <pigpio.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <math.h>

//#define GABRIEL 1
#define FELIPE 1	// Pra usar as formulas do felipe

#define SERVO_BASE	19
#define SERVO_Z		13
#define SERVO_X		12
//#define BOBINA	6 

#ifdef FELIPE
#define MIN_BASE	500
#define MAX_BASE	2500

#define MIN_X	500
#define MAX_X	1550

#define MIN_Z	1000
#define MAX_Z	2500
#endif

#ifdef GABRIEL
#define MIN_BASE	500
#define MAX_BASE	2500

#define MIN_X	500
#define MAX_X	1530

#define MIN_Z	1000
#define MAX_Z	2500
#endif

#define SPEED 1

const int a1 = 8.0;
const int a2 = 8.0;

#define MAX_LEN	(a1+a2)
#define MIN_LEN 1

int gpioServoBound(int servo, int us);

int degree_to_us(double degree, int servo);

void inverse_kinematics(double x, double z, int *usb, int *usx, int *usz);

#endif

