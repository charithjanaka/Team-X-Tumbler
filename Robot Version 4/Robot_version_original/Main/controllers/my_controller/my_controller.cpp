#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/LED.hpp>
#define timeStep 16
#define threshold 600
using namespace webots;

/////change for line follwing smoothness/////////
float Kp = 0.003 ,Kd = 0.00095, Ki = 0.00001; 

/////change for wall follwing smoothness////////
float Kpw = 0.0005, Kdw = 0.0001, Kiw = 0.00001;

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
PositionSensor *Encorders[2];
LED *LEDS[8];

void setMotorSpeeds(float leftSpeed, float rightSpeed);
double ReadSenAndDevi();
void PID_line();
void PID_Wall();
void ReadWall();
void Turnon_LED();
int checkForTurns();
void ReadSen();
void TurnNintey(int direction);
void stop();
float get_linear(float an_speed);
float get_angular(float li_speed);
float get_linear_distance(float radial_distance);
double last_print_time;
void ideal(double pre_time , int pref, double period);
void calculate_circumference(float left_x,float right_x);
bool Box_detected = false;

double startTime;
double forwardTime = 1.5;
double rotateTime = 1.5;
double LS;
double RS;
int stage=1;
int quad_num = 1;
int junction_count = 0;
float traveled_distance[2] = {0,0};


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
float wall_margin = 160;

int main (int argc, char  **argv){
    
    char wheel_names[2][10] = {"Lmotor" , "Rmotor"};
    for (int i=0; i< 2; i++){
        wheels[i] = robot->getMotor(wheel_names[i]);
        wheels[i] ->setPosition(INFINITY);
        wheels[i] ->setVelocity(0.0);
    }
    
    char position_sensor_names[2][10] = {"left_ps","right_ps"};
    for (int m=0; m<2; m++){
        Encorders[m] = robot-> getPositionSensor(position_sensor_names[m]);
        Encorders[m]->enable(timeStep);
    }
    char ir_sensor_names[8][4] = {"S1","S2","S3","S4","S5","S6","S7","S8"};
    for (int j=0; j<8 ; j++){
        ir[j] = robot-> getDistanceSensor(ir_sensor_names[j]);
        ir[j]->enable(timeStep);
    }
    
    char LED_names[8][7] = {"led_0","led_1","led_2","led_3","led_4","led_5","led_6","led_7"};
    for (int n=0; n<8 ; n++){
        LEDS[n] = robot-> getLED(LED_names[n]);
    }

    char wall_sensor_names[2][15] = {"pilSensorL","pilSensorR"};
    for (int k=0; k<2; k++){
        wall[k] = robot-> getDistanceSensor(wall_sensor_names[k]);
        wall[k]->enable(timeStep);
    }
    
    Front_sensor[0] = robot->getDistanceSensor("frontSensor");
    Front_sensor[0]->enable(timeStep);
    
    ideal(robot->getTime(),1,2);
    ideal(robot->getTime(),2,1.5);
    
    while (robot->step(timeStep) != -1){
       
       Turnon_LED();
       
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
              if(quad_num == 2){
                  traveled_distance[0] =  Encorders[0]->getValue();
                  traveled_distance[1] =  Encorders[1]->getValue();
              }
          }
        }
        else if(quad_num == 5){
          calculate_circumference(get_linear_distance(Encorders[0]->getValue()-traveled_distance[0]) , get_linear_distance(Encorders[1]->getValue()-traveled_distance[1]));
          TurnNintey(1);
          stage ++;
        } 
        else{
           PID_line();
        }  
      }
      
      
      if (stage == 3){
        while(robot->step(timeStep) != -1){
          if((Box_detected == false) && ((Front_sensor[0]->getValue()) > 650)){
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
           TurnNintey(2);
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
   if (SL[0]==1 && SL[1]==1 && SL[2]==1 && SL[3]==1 && SL[4]==1 && SL[6]==0 && SL[7]==0){
     mark = 1; // right Turn
   }
   
   else if (SL[0]==0 && SL[1]==0 && SL[3]==1 && SL[4]==1 && SL[5]==1 && SL[6]==1 && SL[7]==1){
     mark = 2; //Left Turn
   }
   else if (SL[0]==1 && SL[1]==1 && SL[2]==1 && SL[3]==1 && SL[4]==1 && SL[5]==1 && SL[6]==1 && SL[7]==1){
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
  for (int i = 0; i<8; i++){
      SL[i] = (ir[i]->getValue()) < threshold; 
  }
}

//////////////////////Read line follwing sensor values and calculate deviation//////////////
double ReadSenAndDevi(){
  for (int i = 0; i<8; i++){
      //std::cout<<"S1:"<<SL[0]<<" S2:"<<SL[1]<<" S3:"<<SL[2]<<" S4:"<<SL[3]<<" S5:"<<SL[4]<<" S6:"<<SL[5]<<" S7:"<<SL[6]<<" S8:"<<SL[7]<<std::endl;
      //std::cout<<"S1:"<<ir[0]->getValue()<<" | S2:"<<ir[1]->getValue()<<" | S3:"<<ir[2]->getValue()<<" | S4:"<<ir[3]->getValue()<<" | S5:"<<ir[4]->getValue()<<" | S6:"<<ir[5]->getValue()<<" | S7:"<<ir[6]->getValue()<<" | S8:"<<ir[7]->getValue()<<std::endl;
      SL[i] = (ir[i]->getValue()) < threshold; 
  }

  int deviation = 0;
  deviation = (10*SL[0] + 20*SL[1] + 30*SL[2] + 40*SL[3] + 50*SL[4] + 60*SL[5] + 70*SL[6] + 80*SL[7])/(SL[0] + SL[1] + SL[2] + SL[3] + SL[4] + SL[5] + SL[6] + SL[7] );
  return deviation;
}

//////////////////////////PID line /////////////////////////////////////
void PID_line(){  
    current_position = ReadSenAndDevi();
    line_error = (int)current_position - 45;
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
  if (l_d<wall_margin && r_d>wall_margin){
    wall_error = r_d - 195;
    std::cout<<"right detected | wall error = "<<wall_error<<std::endl;
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

    else if(r_d<wall_margin && l_d>wall_margin){
      wall_error = l_d - 195;
      std::cout<<"left detected | wall error = "<<wall_error<<std::endl;
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
    
    else if(r_d>wall_margin && l_d>wall_margin){
      wall_error = l_d - 195;
      std::cout<<"both detected | wall error = "<<wall_error<<std::endl;
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
  
float get_linear_distance(float radial_distance){
  return wheel_radius * radial_distance;
}

void calculate_circumference(float left_x,float right_x){
  float single_quadrant = (left_x + right_x)/6;
  std::cout<<"radius = "<<((single_quadrant*200)/3.1415)<<" | circumference = "<<single_quadrant*400<<std::endl;
  }

void ideal(double pre_time , int pref, double period){
     if (pref == 1){
        while (robot->step(timeStep) != -1){
          if((robot->getTime()-pre_time) > period){
            break;
          }
          else{
            stop();
            //std::cout<<"Current stage = "<<stage<<" >> Colour detection task completing.."<<std::endl;
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

void Turnon_LED(){
  LEDS[0]->set(255*SL[0]);
  LEDS[1]->set(255*SL[1]);
  LEDS[2]->set(255*SL[2]);
  LEDS[3]->set(255*SL[3]);
  LEDS[4]->set(255*SL[4]);
  LEDS[5]->set(255*SL[5]);
  LEDS[6]->set(255*SL[6]);
  LEDS[7]->set(255*SL[7]);
}