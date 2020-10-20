#ifndef _ROBOT_H
#define _ROBOT_H


#include <iostream>
#include <string>
#include <bits/stdc++.h>
#include <sys/types.h>
#include <unistd.h>


#define base 3
#define eixo_x 1
#define eixo_y 2
#define garra 0
namespace N{

  class robot {
    public:
      void setup(void);
      void move_abs(int position, unsigned int pin);
      void move_percent(int percentage, unsigned int pin);
      void move_increment(int increment, unsigned int pin);
      void move_decrement(int decrement, unsigned int pin);
      void close(void);
    private:
      char cmd[100];
  };
}


#endif
