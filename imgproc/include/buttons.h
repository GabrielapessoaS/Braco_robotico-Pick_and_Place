/*
 * Programa para o teste dos botões e otimização das funções para serem
 * incorporadas ao código principal
 *
 * autores: Felipe Sobrinho, Gabriela Pessoa, Gabriel Williams
 */


#ifndef BUTTONS_H
#define BUTTONS_H

#include <stdio.h>
#include <stdlib.h>
#include <pigpio.h>
#include <unistd.h>
#include <signal.h>

// Macros para os pinos dos botoes

#define BUT1 17
#define BUT2 27
#define BUT3 22

extern int calib; 


// Prototipo para as funcoes

void buttonsConfig();
void initialisePolling();
void but1ISR();
void but2ISR();
void but3ISR();



#endif
