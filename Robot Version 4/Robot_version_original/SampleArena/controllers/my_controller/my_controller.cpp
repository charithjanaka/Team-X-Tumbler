#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#define timeStep 16
#define threshold 500
using namespace webots;

/////change for line follwing smoothness/////////
float Kp = 0.0042 ,Kd = 0.0007, Ki = 0.00001; 

/////change for wall follwing smoothness////////
float Kpw = 0.004, Kdw = 0.003, Kiw = 0.0001;

double SL[8] = {0,0,0,0,0,0,0,0};
double l_d,r_d;
float base_speed = 0.13;
float max_change = 0.314 - base_speed;
float wheel_radius = 0.03;

Robot *robot  = new Robot();
Motor *wheels[2];
DistanceSensor *ir[8];
DistanceSensor *wall[2];
DistanceSensor *Front_sensor[1];

void setMotorSpeeds(float leftSpeed, float rightSpeed);
double ReadSenAndDevi();
void PID_line();
void PID_Wall();
void ReadWall();
int checkForTurns();
void ReadSen();
void TurnNintey(int direction);
void stop();
float get_linear(float an_speed);
float get_angular(float li_speed);
double last_print_time;
void ideal(double pre_time , int pref, double period);
bool Box_detected = false;

double startTime;
double forwardTime = 1.3;
double rotateTime = 1.4;
double LS;
double RS;
int stage=4;
int quad_num = 1;
int junction_count = 0;

double current_position = 0;
int line_error = 0;
double propotional =0;
double derivative = 0;
double integral = 0;
double prev_error_line = 0;
double correction = 0 ;

double correction_w = 0;
double wall_error=0;
double propotional_W=0; 
double derivative_W=0; 
double integral_W=0;
double prev_error_Wall=0;

int main (int argc, char  **argv){
    
    char wheel_names[2][10] = {"Lmotor" , "Rmotor"};
    for (int i=0; i< 2; i++){
        wheels[i] = robot->getMotor(wheel_names[i]);
        wheels[i] ->setPosition(INFINITY);
        wheels[i] ->setVelocity(0.0);
    }
    
    char ir_sensor_names[][4] = {"S1","S2","S3","S4","S5","S6","S7","S8"};
    for (int j=0; j<9 ; j++){
        ir[j] = robot-> getDistanceSensor(ir_sensor_names[j]);
        ir[j]->enable(timeStep);
    }

    char wall_sensor_names[2][15] = {"pilSensorL","pilSensorR"};
    for (int k=0; k<2; k++){
        wall[k] = robot-> getDistanceSensor(wall_sensor_names[k]);
        wall[k]->enable(timeStep);
    }
    
    Front_sensor[0] = robot->getDistanceSensor("FS");
    Front_sensor[0]->enable(timeStep);
    ideal(robot->getTime(),2,1.5);
    while (robot->step(timeStep) != -1){
       
       if (stage == 1){
         if(ReadSenAndDevi()==-2147483648){
           PID_Wall();
         }
         else if(checkForTurns() == 1){ //right Turn
           std::cout<<"Current stage = "<<stage<<" >> Turning Right at right junction"<<std::endl;
           TurnNintey(1);
            }
         
         else if(checkForTurns() == 2){ //left Turn
           std::cout<<"Current stage = "<<stage<<" >> Turning left at left junction"<<std::endl;
           TurnNintey(2);
          }
          
          else if(checkForTurns() == 3){ //T junction
           std::cout<<"Current stage = "<<stage<<" >> Turning left at T junction"<<std::endl;
           TurnNintey(2);
           stage ++;
           std::cout<<"Current stage = "<<stage<<" >> X-Tumbler robot at quadrant = "<<quad_num<<std::endl;
          }
         
         else{
           PID_line();
         }
      }
      
      if (stage == 2){
        
        if(checkForTurns() == 1){ //right Turn detected
            if(quad_num<5 && (robot->getTime() - last_print_time)>6){
              quad_num ++;
              if (quad_num<5){
                 std::cout<<"Current stage = "<<stage<<" >> X-Tumbler robot at quadrant = "<<quad_num<<std::endl;
                last_print_time = robot->getTime();
              }
            }
          }
        else if(quad_num == 5){
          TurnNintey(1);
          stage ++;
        } 
         else{
           PID_line();
         }  
      }
      
      if (stage == 3){
        while(robot->step(timeStep) != -1){
          if((Box_detected == false) && ((Front_sensor[0]->getValue()) < 0.16)){
              stop();
              ideal(robot->getTime(),1,5);
              Box_detected = true;
            }
          else if(checkForTurns() == 1){ //right Turn
           std::cout<<"Current stage = "<<stage<<" >> Turning Right at right junction"<<std::endl;
               if(Box_detected == false){
                 TurnNintey(1);
               }
               else if(Box_detected == true){
                 TurnNintey(1);
                 stage ++;
                 break;
               }
            }
          else if(checkForTurns() == 2){ //left Turn
           std::cout<<"Current stage = "<<stage<<" >> Turning left at left junction"<<std::endl;
             if(Box_detected == false){
                 TurnNintey(2);
               }
               else if(Box_detected == true){
                 TurnNintey(2);
                 stage ++;
                 break;
               }
          }
          else if(checkForTurns() == 3){ //T- junction
           std::cout<<"Current stage = "<<stage<<" >> T-junction Detected"<<std::endl;
           junction_count ++;
           //std::cout<<"Turning right at T junction"<<std::endl;
               if((junction_count % 2 == 0) && Box_detected == false){
                 TurnNintey(1);//turn right
               }
               else if((junction_count == 4) && Box_detected == true){
                 TurnNintey(1);//turn right
               }
               else if((junction_count % 2 == 0) && Box_detected == true){
                 TurnNintey(2);//turn left
               }
               else{
                 ideal(robot->getTime(),2,1);
               }
            }
          else{
            PID_line();
          }   
          
          std::cout<<junction_count<<"   "<<Box_detected<<std::endl;  
        }
        //std::cout<<Front_sensor[0]->getValue()<<stage<<std::endl; 
      }
      
      if (stage == 4){
        if(ReadSenAndDevi()==-2147483648){
           ideal(robot->getTime(),2,0.05);
         }
        else if(checkForTurns() == 1){ //right Turn
           std::cout<<"Current stage = "<<stage<<" >> Turning Right at right junction"<<std::endl;
           TurnNintey(1);
            }
         
        else if(checkForTurns() == 2){ //left Turn
           std::cout<<"Current stage = "<<stage<<" >> Turning left at left junction"<<std::endl;
           TurnNintey(2);
          }
          
         else if(checkForTurns() == 3){ //T junction
           std::cout<<"Current stage = "<<stage<<" >> Turning left at T junction"<<std::endl;
           TurnNintey(1);
           std::cout<<"Current stage = "<<stage<<" >> X-Tumbler robot at quadrant = "<<quad_num<<std::endl;
          }
         else{
           PID_line();
         }
      }
      
      //std::cout<<"Current stage = "<<stage<<std::endl;
}
}
/////////////////////Checking for junctions/////////////////
int checkForTurns(){
   double mark = 0;
   ReadSen();
   if (SL[1]==1 && SL[2]==1 && SL[3]==1 && SL[4]==1 && SL[6]==0 && SL[7]==0 && SL[8]==0){
     mark = 1; // right Turn
   }
   
   else if (SL[0]==0 && SL[1]==0 && SL[2]==0 && SL[4]==1 && SL[5]==1 && SL[6]==1 && SL[7]==1){
     mark = 2; //Left Turn
   }
   else if (SL[0]==1 && SL[1]==1 && SL[2]==1 && SL[3]==1 && SL[4]==1 && SL[5]==1 && SL[6]==1 && SL[7]==1 && SL[8]==1){
     mark = 3; //T Junction
     }
   else{
     mark = 4;
     }
   return mark;
}

//////////////////////////Turn ninty degrees/////////////////////////
void TurnNintey(int direction){
    if (direction == 1){
          LS = 0.1 ;
          RS = -0.1;
     }
     
     else if(direction == 2){
          LS = -0.1;
          RS = 0.1;
     }
     
    
     double forwardEndTime = robot->getTime() + forwardTime;
     while (robot->step(timeStep) != -1){
      if (forwardEndTime > robot->getTime()){
        setMotorSpeeds(0.1,0.1);
      }
      else{
        stop();
        break;
          }
    }
    
    
    double rotationEndTime = robot->getTime() + rotateTime;
    
    while (robot->step(timeStep) != -1){
      if (rotationEndTime > robot->getTime()){
        setMotorSpeeds(LS,RS);
      }
      else{
        stop();
        break;
          }
    }
    return;
}


void stop(){
  setMotorSpeeds(0, 0);
}
  
///////////////////////////////////set motor speeds///////////////////////////////
void setMotorSpeeds(float leftSpeed, float rightSpeed){
  //std::cout<<"leftSpeed ="<<get_angular(leftSpeed)<<"    rightSpeed ="<<get_angular(rightSpeed)<<std::endl;
  wheels[0]->setVelocity(get_angular(leftSpeed));
  wheels[1]->setVelocity(get_angular(rightSpeed));
}

////////////////////////////////////Read line following senosor values/////////
void ReadSen(){
  for (int i = 0; i<9; i++){
      SL[i] = (ir[i]->getValue()) < threshold; 
  }
}

//////////////////////Read line follwing sensor values and calculate deviation//////////////
double ReadSenAndDevi(){
  for (int i = 0; i<9; i++){
      SL[i] = (ir[i]->getValue()) < threshold; 
  }

  int deviation = 0;
  deviation = (10*SL[1] + 20*SL[2] + 30*SL[3] + 40*SL[4] + 50*SL[5] + 60*SL[6] + 70*SL[7])/(SL[1] + SL[2] + SL[3] + SL[4] + SL[5] + SL[6] + SL[7]);
  return deviation;
}

//////////////////////////PID line /////////////////////////////////////
void PID_line(){  
    current_position = ReadSenAndDevi();
    line_error = (int)current_position - 40;
    propotional = line_error * Kp;
    derivative = (line_error - prev_error_line) * Kd;
    integral = (integral + line_error) * Ki;
  
    correction = propotional + derivative + integral;
    //std::cout<<"correction ="<<correction<<std::endl;
    if (correction > max_change){
       correction = max_change;
       
    }
    if (correction < -1*max_change){
       correction = -1 * max_change;
    }

    setMotorSpeeds((base_speed - correction), (base_speed + correction));
    prev_error_line = line_error;
  
}

////////////////////////PID wall/////////////////////////////////////
void PID_Wall(){
    //std::cout<<"wall detected"<<std::endl;
  l_d = wall[0]->getValue();
  r_d = wall[1]->getValue();
  std::cout<<"Left :"<<l_d<<"     right :"<<r_d<<std::endl;
  if (l_d>15 && r_d<15){
    wall_error = r_d - 7;
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
    
    
    setMotorSpeeds((base_speed - correction_w), (base_speed + correction_w));
  }

    else if(r_d>15 && l_d<15){
      std::cout<<"left detected"<<std::endl;
      wall_error = l_d - 7;
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
      setMotorSpeeds((base_speed + correction_w), (base_speed - correction_w));
    }
    
    else{
      std::cout<<"going straight"<<std::endl;
      setMotorSpeeds(0.13,0.13);      
  }
  
  prev_error_Wall = wall_error;
}

float get_linear(float an_speed){
  return wheel_radius * an_speed;
  }

float get_angular(float li_speed){
  return li_speed/ wheel_radius;
  }

void ideal(double pre_time , int pref, double period){
     if (pref == 1){
        while (robot->step(timeStep) != -1){
          if((robot->getTime()-pre_time) > period){
            break;
          }
          else{
            stop();
            std::cout<<"Current stage = "<<stage<<" >> Colour detection task completing.."<<std::endl;
          }
        }
     }
      
     else if(pref == 2){
       while (robot->step(timeStep) != -1){
          if((robot->getTime()-pre_time) > period){
            break;
          }
          else{
            setMotorSpeeds(0.1,0.1);
            //std::cout<<"pass my mark"<<std::endl;
          }
       }
     }
} 