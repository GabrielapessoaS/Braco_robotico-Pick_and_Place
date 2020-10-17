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
  // Inserindo posicao inicial
  robo.move_percent(35, base);
  //robo.move_percent(10, eixo_y);
  //robo.move_percent(10, eixo_x);
  //robo.move_percent(50, garra);

  sleep(5);

  for(int i = 20; i < 80; i++){
   // robo.move_increment(10, base);
    robo.move_percent(i, eixo_y);
    //robo.move_increment(10, eixo_y);
    //robo.move_increment(10, garra);
    usleep(200000);
  }

  sleep(5);

  for(int i = 80; i < 20; i--){
    //robo.move_decrement(10, base);
    robo.move_percent(i, eixo_y);
    //robo.move_decrement(10, eixo_y);
    //robo.move_decrement(10, garra);
    usleep(200000);
  }

  sleep(5);
  
  cout << "Demonstracao concluida\n";
  
  robo.close();



  return 0;
}
