/* 
 * Definicoes das funcoes de cinematica inversa e controle dos servos
 * para controle do braco robotico de 3 DOF.
 *
 * @Authors:	Felipe Sobrinho, Gabriela Pessoa, Gabriel Mendonca
 * Rev. 1:	Gabriel Mendonca
 *
 *
   This file is part of PicNPlace.

    PicNPlace is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    PicNPlace is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with PicNPlace.  If not, see <https://www.gnu.org/licenses/>.
 * 
 *
 * Copyright 2020 Felipe Sobrinho, Gabriela Pessoa, Gabriel Mendonca
 *
 */

#include "inv_kinematics.h"

/* 
 * Limitacao de largura de pulso a ser enviada para os servos.
 *
 * @returns: -1 caso servo nao estiver listado, 0 caso contrario
 */

int gpioServoBound(int servo, int us) {
	switch(servo) {
		case SERVO_BASE:
			if( us < MIN_BASE ) us = MIN_BASE;
			if( us > MAX_BASE ) us = MAX_BASE;
			break;
		case SERVO_A2:
			if( us < MIN_Z ) us = MIN_Z;
			if( us > MAX_Z ) us = MAX_Z;
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
	int v=-1;
	switch(servo){
		case SERVO_BASE:
			v = EXP_BASE;
			fprintf(stdout, "valor retornado por base: %d\n", v);
			break;
		case SERVO_A1:
			v = EXP_X;
			fprintf(stdout, "valor retornado por X: %d\n", v);
			break;
		case SERVO_A2:
			v = EXP_Z;
			fprintf(stdout, "valor retornado por Z: %d\n", v);
			break;
	}

	if( v < 0 ) return 0;

	if(v < 500) v = 500;
	else if(v > 2500) v = 2500;

	return v;
}

void inverse_kinematics(double x, double y, int *usb, int *usx, int *usz){
	double theta1, theta2, theta3;
	double z =0;
	printf("Cinematica inversa para (%lf, %lf, %lf)...\n", x,y, z);

	if((sqrt(y*y + z*z) > MAX_LEN) || (sqrt(y*y + z*z) < MIN_LEN)) {
		printf("Ponto alem do alcance.\n");
		return;
	}
	theta1 = atan(z/y) + acos((y*y + z*z + a1*a1 - a2*a2)/(2*a1*sqrt(y*y + z*z)));
	theta2 = theta1 - acos((y*y +z*z - a1*a1 - a2*a2) / (2.0*a1*a2));
	theta3 =  atan2(y, x);

	theta1 = 180.0*theta1/M_PI;
	theta2 = 180.0*theta2/M_PI;
	theta3 = 180.0*theta3/M_PI;

	printf("theta1 = %lf\n", theta1);
	printf("theta2 = %lf\n", theta2);
	printf("theta3 = %lf\n", theta3);


	*usx = degree_to_us(theta1, SERVO_A1);
	*usz = degree_to_us(theta2, SERVO_A2);
	*usb = degree_to_us(theta3, SERVO_BASE);

	printf("Entre na funcao de cinematica inversa\n\n");
}
