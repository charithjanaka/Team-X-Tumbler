#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <iostream>

#define TIME_STEP 1
#define MAX_SPEED 5

// All the webots classes are defined in the "webots" namespace
using namespace webots;

int main(int argc, char **argv) 
{
  Robot *robot = new Robot();

  Motor *Mot = robot->getMotor("mot");
  PositionSensor *ps = robot->getPositionSensor("ps_1");

  ps->enable(TIME_STEP);

  Mot->setPosition(INFINITY);
  Mot->setVelocity(0.0);
  long c1 =  0;
  long c2 = 0;
  int begin = robot->getTime();
  while (robot->step(TIME_STEP) != -1) 
  {
    if (((robot->getTime() - begin) > 3) & ((robot->getTime() - begin) < 10))
    {
      Mot->setVelocity(MAX_SPEED);
      std::cout << ps->getValue() << std::endl;
      if (robot->getTime()>=(3+c1)+0.26){
        Mot->setVelocity(0.0);
      }
    }
    
    if (((robot->getTime() - begin) > 10) & ((robot->getTime() - begin) < 13))
    {
      Mot->setVelocity(-MAX_SPEED);
      if (robot->getTime() >= (10+c2)+0.26){
        Mot->setVelocity(0.0);
        c1=c1+10;
        c2=c2+10;
        begin = robot->getTime();
      }
    }
    /*if(robot->getTime() < 3){//ps->getValue() < 3.14
      std::cout << robot->getTime() << std::endl;
    } 
    else{
      Mot->setVelocity(0.0);
    }*/
    //begin = robot->getTime();
  }

  delete robot;
  return 0;
}