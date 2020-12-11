#include "buttons.h"



void sighandler(int signum){
  printf("Termino do programa\n");
  gpioTerminate();
  exit(0);
}


int main(int argc, char **argv){

  buttonsConfig();
  initialisePolling();


  signal(SIGINT, sighandler);

  while(1);



  return 0;
}
