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

/* 
 * Limitacao de largura de pulso a ser enviada para os servos.
 *
 * @returns: -1 caso servo nao estar listado, 0 caso contrario
 */
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
		case SERVO_X:
			//return ((int)(11.11*(*degree)+ 500.0));
			return ((int)(9.14*(180-degree) + 547.4));
		case SERVO_Y:
			//return ((int)(-15.55*(*degree - dg_j) + 1900.0));
			return ((int)(9.14*degree + 1950.0));
	}
	return 0;
}

void inverse_kinematics(double x, double y){
  double theta1, double theta2;
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
