#include "buttons.h"

void buttonsConfig(){

  if(gpioInitialise()<0){
    printf("\tFalha na inicailizacao dos gpios\n");
    gpioTerminate();
    exit(-1);
  }
  else{
    printf("\tGpio inicializado com suceso\n");
  }
  //Definicao da direcao dos pinos

  gpioSetMode(BUT1, PI_INPUT);
  gpioSetMode(BUT2, PI_INPUT);
  gpioSetMode(BUT3, PI_INPUT);

  if(gpioGetMode(BUT1)==PI_INPUT && gpioGetMode(BUT2)==PI_INPUT && gpioGetMode(BUT3)==PI_INPUT){
    printf("Botoes com direcoes definidas");

  }
  else{
    printf("Erro na definicao de direcao dos botoes\n");
    gpioTerminate();
    exit(-1);
  }

}

void initialisePolling(){

  gpioSetISRFunc(BUT1, FALLING_EDGE, -1, but1ISR);
  gpioSetISRFunc(BUT2, FALLING_EDGE, -1, but2ISR);
  gpioSetISRFunc(BUT3, FALLING_EDGE, -1, but3ISR);

  //Loop infinito aguardando algum evento
  //
  while(1);
  
}


void but1ISR(){

  if(gpioRead(BUT1) == 0){
    usleep(2000);
    if(gpioRead(BUT1) !=0){
      printf("\t Bouncing detectado\n");

    }
    else{
      printf("Debouncing realizado. BUT1 pressionado!\n");
    }
    

  }
  else{
    printf("Botao nao precionado\n");
  }


}

void but2ISR(){
  if(gpioRead(BUT2) == 0){
    usleep(2000);
    if(gpioRead(BUT2) !=0){
      printf("\t Bouncing detectado\n");

    }
    else{
      printf("Debouncing realizado. BUT2 pressionado!\n");
    }
    

  }
  else{
    printf("Botao nao precionado\n");
  }

}


void but3ISR(){
  if(gpioRead(BUT3) == 0){
    usleep(2000);
    if(gpioRead(BUT3) !=0){
      printf("\t Bouncing detectado\n");

    }
    else{
      printf("Debouncing realizado. BUT3 pressionado!\n");
    }
    

  }
  else{
    printf("Botao nao precionado\n");
  }

}
