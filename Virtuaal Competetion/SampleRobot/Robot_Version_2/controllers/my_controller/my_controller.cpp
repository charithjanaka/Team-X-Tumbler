#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#define timeStep 64
#define threshold 500
using namespace webots;

double SL[9] = {0,0,0,0,0,0,0,0,0};
double l_d,r_d;
float Kp = 0.5 ,Kd = 0.05, Ki = 0.001;
int base_speed = 130;
int max_change = 255 - base_speed;

Robot *robot  = new Robot();
Motor *wheels[2];
DistanceSensor *ir[9];
DistanceSensor *wall[2];

void setMotorSpeeds(float leftSpeed, float rightSpeed);
double ReadSenAndDevi();
void PID_line();
void PID_Wall();
void ReadWall();
double checkForTurns();
void ReadSen();

double current_position = 0;
int line_error = 0;
double propotional =0;
double derivative = 0;
double integral = 0;
double prev_error_line = 0;
double correction = 0 ;

int correction_w = 0;
float Kpw = 0.1, Kdw = 0.1, Kiw = 0.001;
double wall_error=0;
double propotional_W=0; 
double derivative_W=0; 
double integral_W=0;
double prev_error_Wall=0;

int main (int argc, char  **argv){
    
    char wheel_names[2][10] = {"Lmotor" , "Rmotor"};
    for (int i=0; i< 2; i++){
        wheels[i] = robot-> getMotor(wheel_names[i]);
        wheels[i] ->setPosition(INFINITY);
        wheels[i] ->setVelocity(0.0);
    }
    
    char ir_sensor_names[9][10] = {"S1","S2","S3","S4","S5","S6","S7","S8","S9"};
    for (int j=0; j<9 ; j++){
        ir[j] = robot-> getDistanceSensor(ir_sensor_names[j]);
        ir[j]->enable(timeStep);
    }

    char wall_sensor_names[2][15] = {"pilSensorL","pilSensorR"};
    for (int k=0; k<2; k++){
        wall[k] = robot-> getDistanceSensor(wall_sensor_names[k]);
        wall[k]->enable(timeStep);
    }
    
    
    while (robot->step(timeStep) != -1){
       if(ReadSenAndDevi()==-2147483648){
         std::cout<<"go wall"<<std::endl;
         PID_Wall();
       }
       
       else if(checkForTurns() != 1){ //right Turn
         std::cout<<"Turning Right"<<std::endl;
         setMotorSpeeds(6.28,-6.28);
         while(checkForTurns()!=1){
           setMotorSpeeds(6.28,-6.28);
         }
       }
       
       else if(checkForTurns() != 2){ //left Turn
         std::cout<<"Turning left"<<std::endl;
         setMotorSpeeds(6.28,-6.28);
         while(checkForTurns()!=2){
           setMotorSpeeds(6.28,-6.28);
         }
        }
        */
       else{
         PID_line();
       }
    }    
   
}

double checkForTurns(){
   double mark = 0;
   ReadSen();
   if (SL[0]==1 && SL[1]==1 && SL[2]==1 && SL[3]==1 && SL[4]==1 && SL[6]==0 && SL[7]==0 && SL[8]==0){
     mark = 1; // right Turn
   }
   
   else if (SL[0]==0 && SL[1]==0 && SL[2]==0 && SL[4]==1 && SL[5]==1 && SL[6]==1 && SL[7]==1 && SL[8]==1){
     mark = 2; //Left Turn
   }
   return mark;
}


void setMotorSpeeds(float leftSpeed, float rightSpeed){
  std::cout<<"leftSpeed ="<<leftSpeed<<"    rightSpeed"<<rightSpeed<<std::endl;
  wheels[0]->setVelocity(leftSpeed);
  wheels[1]->setVelocity(rightSpeed);
}

double ReadSenAndDevi(){
  for (int i = 0; i<9; i++){
      SL[i] = (ir[i]->getValue()) < threshold; 
  }

  int deviation = 0;
  deviation = (100*SL[1] + 200*SL[2] + 300*SL[3] + 400*SL[4] + 500*SL[5] + 600*SL[6] + 700*SL[7])/(SL[1] + SL[2] + SL[3] + SL[4] + SL[5] + SL[6] + SL[7]);
  return deviation;
}

void ReadSen(){
  for (int i = 0; i<9; i++){
      SL[i] = (ir[i]->getValue()) < threshold; 
  }
}


void PID_line(){  
    current_position = ReadSenAndDevi();
    std::cout<<current_position<<std::endl;
    line_error = (int)current_position - 400;
    propotional = line_error * Kp;
    derivative = (line_error - prev_error_line) * Kd;
    integral = (integral + line_error) * Ki;
  
    correction = propotional + derivative + integral;
    if (correction > max_change){
       correction = max_change;
    }
    if (correction < -1*max_change){
       correction = -1 * max_change;
    }
    
    setMotorSpeeds((base_speed - correction)/40.6, (base_speed + correction)/40.6);
    prev_error_line = line_error;
  
}


void PID_Wall(){
  l_d = wall[0]->getValue();
  r_d = wall[1]->getValue();
  std::cout<<"Left :"<<l_d<<"     right :"<<r_d<<std::endl;
  if (l_d>999 && r_d<950){
    wall_error = r_d - 400;
    std::cout<<"right detected"<<std::endl;
    propotional_W = wall_error * Kpw;
    derivative_W = (wall_error - prev_error_Wall) * Kdw;
    integral_W = (integral_W + wall_error) * Kiw;
    correction_w = propotional_W + derivative_W + integral_W;
    if (correction_w > max_change){
      correction_w = max_change;
    }
    if (correction_w < -1*max_change){
      correction_w = -1 * max_change;
    }
    
    
    setMotorSpeeds((base_speed - correction_w)/80, (base_speed + correction_w)/80);
  }

    else if(r_d>999 && l_d<950){
      std::cout<<"left detected"<<std::endl;
      wall_error = l_d - 450;
      propotional_W = wall_error * Kpw;
      derivative_W = (wall_error - prev_error_Wall) * Kdw;
      integral_W = (integral_W + wall_error) * Kiw;

      correction_w = propotional_W + derivative_W + integral_W;
      if (correction_w > max_change){
        correction_w = max_change;
      }
      if (correction_w < -1*max_change){
        correction_w = -1 * max_change;
      }
      setMotorSpeeds((base_speed + correction_w)/80, (base_speed - correction_w)/80);
    }
    
    else{
      setMotorSpeeds(4.5,4.5);      
  }
  std::cout<<"###########"<<std::endl;
  prev_error_Wall = wall_error;
}
