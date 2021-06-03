// File:          keyboard.cpp
// Date:
// Description:
// Author:
// Modifications:
#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <iostream>
#include <stdio.h>
#include <webots/Keyboard.hpp>
//#include <algorithm>
#include <bits/stdc++.h> 
// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#define TIME_STEP 64
// All the webots classes are defined in the "webots" namespace
using namespace webots;

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();
  
  Keyboard kb;
  // get the time step of the current world.
  //int timeStep = (int)robot->getBasicTimeStep();
  Motor *rm;
  rm=robot->getMotor("RM");
  Motor *rm2;
  rm2=robot->getMotor("RM2");
  Motor *rm3;
  rm3=robot->getMotor("RM3");
  Motor *rm4;
  rm4=robot->getMotor("RM4");
  kb.enable(TIME_STEP);
  double rotate=0.0;
  double rotate2=0.0;
  double rotate3=0.0;
  double rotate4=0.0;

  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  Motor *motor = robot->getMotor("motorname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(TIME_STEP) != -1) {
    int key=kb.getKey();
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();
    if (key==65 && rotate<0.5){ //press A on keyboard
    rotate+=0.05;
    }else if (key==66 && rotate>-2){ //press B on keyboard
    rotate+=-0.05;
    }else{
    rotate+=0;
    }
    rm->setPosition(rotate);
    // Process sensor data here.
    if (key==67 && rotate2<0.4 && rotate3>-0.4){ //press C on keyboard
    rotate2+=0.05;
    rotate3+=-0.05;
    }else if (key==68 && rotate2>-0.4 && rotate3<0.4){ //press D on keyboard
    rotate2+=-0.05;
    rotate3+=0.05;
    }else{
    rotate2+=0;
    rotate3+=0;
    }
    rm2->setPosition(rotate2);
    rm3->setPosition(rotate3);
    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);
    if (key==69 && rotate4<1.5){ //press E on keyboard
    rotate4+=0.05;
    }else if (key==70 && rotate4>-0.2){ //press F on keyboard
    rotate4+=-0.05;
    }else{
    rotate4+=0;
    }
    rm4->setPosition(rotate4);
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
