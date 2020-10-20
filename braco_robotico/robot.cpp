#include "robot.h"

using namespace N;

void robot::setup(){ //inicializacao do daemon

  system("./servod --pcm --daemonise=1");
}

void robot::move_abs(int position, unsigned int pin){
  
  sprintf(cmd, "echo %d=%d > /dev/servoblaster", pin, position);
  system(cmd);

}

void robot::move_increment(int increment, unsigned int pin){

  sprintf(cmd, "echo %d=+%d > /dev/servoblaster", pin, increment);
  system(cmd);
}

void robot::move_decrement(int decrement, unsigned int pin){
  sprintf(cmd, "echo %d=-%d > /dev/servoblaster", pin, decrement);
  system(cmd);
}


void robot::move_percent(int percentage, unsigned int pin){
  
  sprintf(cmd, "echo %d=%d%% > /dev/servoblaster", pin, percentage);
  std::cout << cmd << "\n";
  system(cmd);

}


void robot::close(void){
  system("sudo killall servod");
}
  
