#include <webots/Robot.hpp>
#include <webots/Motor.hpp>

#define TIME_STEP 1
#define MAX_SPEED 5

using namespace webots;

int main(int argc, char **argv) 
{
  Robot *robot = new Robot();
  Motor *Mot = robot->getMotor("LG1M");

  Mot->setPosition(INFINITY);
  Mot->setVelocity(0.0);
  
  long c1 = 0;
  long c2 = 0;
  int begin = robot->getTime();
  
  while (robot->step(TIME_STEP) != -1) 
  {
    if (((robot->getTime() - begin) >= 0) & ((robot->getTime() - begin) < 10))
    {
      Mot->setVelocity(MAX_SPEED);
      if (robot->getTime()>=(c1)+0.26){
        Mot->setVelocity(0.0);
      }
    }
    
    else if (((robot->getTime() - begin) >= 10) & ((robot->getTime() - begin) < 20))
    {
      Mot->setVelocity(-MAX_SPEED);
      if (robot->getTime() >= (10+c2)+0.26){
        Mot->setVelocity(0.0);
      }
    }
    else{
      c1=c1+20;
      c2=c2+20;
      begin = robot->getTime();
    }
  }
  delete robot;
  return 0;
}