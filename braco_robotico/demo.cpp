#include "robot.h"


using namespace std;
using namespace N;



int main (int argc, char **argv){


  robot robo;

  cout << "Bem-vindo(a) a essa demonstracao\n"
    << "Antes de inicarmos voce deve conhecer os pinos\n"
    << "Voce deve conectar o robo nos pinos P1-7; P1-12; P1-13; P1-15\n";

  cout << "Verifique o funcionamento do seu braco agora :)\n";

  
  robo.setup();

  sleep(5);

  for (int i=0 ; i< 4 ; i++){
    
    robo.move_percent(35, base);
    robo.move_percent(50, eixo_y);
    robo.move_percent(50, eixo_x);
    robo.move_percent(70, garra);
    for(int j=10 ; j<90; j++){
      robo.move_percent(j,i);
      usleep(200000);

    }
  
  }


  sleep(5);

  cout << "Demonstracao concluida\n";
  
  robo.close();



  return 0;
}
