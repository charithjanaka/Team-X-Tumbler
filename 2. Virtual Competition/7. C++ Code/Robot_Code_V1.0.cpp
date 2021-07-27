// Group No : 05
// Last Modified : 26th July 2021

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Camera.hpp>
#include <webots/LED.hpp>

#define timeStep 16
#define threshold 600
#define wheel_radius 0.03

using namespace webots;

Robot *robot  = new Robot();
Motor *wheels[2];
DistanceSensor *ir[8];
DistanceSensor *wall[2];
DistanceSensor *Front_sensor[1];
DistanceSensor *sonar[1]; 
PositionSensor *Encorders[2];
Motor *Mot1 = robot->getMotor("RMB"); 
Motor *Mot2 = robot->getMotor("RML");
Motor *Mot3 = robot->getMotor("RMR");
Motor *Mot4 = robot->getMotor("RMC");
LED *LEDS[8];

//////////////////////////////////////////////////////////// Line PID Parameters ///////////////////////////////////////////////////////////

float Kp = 0.01, Ki = 0.0005, Kd = 0.01;

float base_speed = 0.483;
float max_change = 0.683 - base_speed;
double current_position = 0;
int line_error = 0;
double propotional =0;
double integral = 0;
double derivative = 0;
double prev_error_line = 0;
double correction = 0 ;

//////////////////////////////////////////////////////////// Wall PID Parameters ///////////////////////////////////////////////////////////

float Kpw = 0.009, Kiw = 0.0001, Kdw = 0.006;

double correction_w = 0;
double wall_error=0;
double propotional_W=0;
double integral_W=0; 
double derivative_W=0; 
double prev_error_Wall=0;
float wall_margin = 190;

/////////////////////////////////////////////////////// Sensor Values initiation //////////////////////////////////////////////////////////

double SL[8] = {0,0,0,0,0,0,0,0};
double l_d,r_d;

/////////////////////////////////////////////////////// Other Paramaters //////////////////////////////////////////////////////////////////

int stage = 1;
int junction_count = 0;
int quad_num = 1;
float traveled_distance[2] = {0,0};

double startTime;
double UTurnrotateTime = 1.5;
double last_print_time;
double gate_run;

double LS;
double RS;

int box_sens_cnt = 0;
bool Box_detected = false;
bool doted_line_detected = false;
int difference = 0;
int error_correct_direction;

int color1=0;    
int color2=0;    
int color3=0;    
int front_color=0;
int bottom_color=0;

int pillar_counter = 0;

bool Gate_1, Gate_2 = false;
int Gate1_sens_count = 0;
int Gate2_sens_count = 0;
int open_count = 0;
int finish = 0;

/////////////////////////////////////////////////////////// Functions ////////////////////////////////////////////////////////////////////

void setMotorSpeeds(float leftSpeed, float rightSpeed);
void stop();
void ReadSen();
double ReadSenAndDevi();
int checkForTurns();
void TurnNintey(int direction ,float user_speed, float user_forward_time, float user_rot_time);
void PID_line();
void PID_Wall();
float get_linear(float an_speed);
float get_angular(float li_speed);
float get_linear_distance(float radial_distance);
void calculate_circumference(float left_x,float right_x);
void ideal(double pre_time , int pref, double period);
void Turnon_LED();
int getColor();
bool Dash();
void ReadWall();
void turn_around();
void correct_error(int turn_error);
bool whiteSquare();
bool verify_pillar();

int main (int argc, char  **argv){
    
    ///////////////////////////////////////////////// Morors Initialization ///////////////////////////////////////////////////////
    
    char wheel_names[2][10] = {"Lmotor" , "Rmotor"};
    for (int i=0; i< 2; i++){
        wheels[i] = robot->getMotor(wheel_names[i]);
        wheels[i] ->setPosition(INFINITY);
        wheels[i] ->setVelocity(0.0);
    }

    /////////////////////////////////////////////// Encoders Initialization //////////////////////////////////////////////////////
    
    char position_sensor_names[3][10] = {"left_ps","right_ps"};
    for (int m=0; m<2; m++){
        Encorders[m] = robot-> getPositionSensor(position_sensor_names[m]);
        Encorders[m]->enable(timeStep);
    }

    ///////////////////////////////////////////// IR Sensor Pannel Initialization ///////////////////////////////////////////////

    char ir_sensor_names[8][4] = {"S1","S2","S3","S4","S5","S6","S7","S8"};
    for (int j=0; j<8 ; j++){
        ir[j] = robot-> getDistanceSensor(ir_sensor_names[j]);
        ir[j]->enable(timeStep);
    }
    
    /////////////////////////////////////////// Robot Arm motors Initialization ///////////////////////////////////////////////
    
    Mot1->setPosition(INFINITY);
    Mot1->setVelocity(0.0);
    Mot2->setPosition(INFINITY);
    Mot2->setVelocity(0.0);
    Mot3->setPosition(INFINITY);
    Mot3->setVelocity(0.0);
    Mot4->setPosition(INFINITY);
    Mot4->setVelocity(0.0); 
    
    /////////////////////////////////////////// LED Pannel Initialization //////////////////////////////////////////////////////

    char LED_names[8][7] = {"led_7","led_6","led_5","led_4","led_3","led_2","led_1","led_0"};
    for (int n=0; n<8 ; n++){
        LEDS[n] = robot-> getLED(LED_names[n]);
    }

    ////////////////////////////////////////// Wall Following Distance Sensors Initialization /////////////////////////////////

    char wall_sensor_names[2][15] = {"pilSensorL","pilSensorR"};
    for (int k=0; k<2; k++){
        wall[k] = robot-> getDistanceSensor(wall_sensor_names[k]);
        wall[k]->enable(timeStep);
    }
    
    ///////////////////////////////////////// Front Distance Sensors Initialization ////////////////////////////////////////////

    Front_sensor[0] = robot->getDistanceSensor("frontSensor");//frontSensor
    Front_sensor[0]->enable(timeStep);

    sonar[0] = robot->getDistanceSensor("sonar");
    sonar[0]->enable(timeStep);
    
    ideal(robot->getTime(),1,2);
    
    ideal(robot->getTime(),2,0.5);
    
    while (robot->step(timeStep) != -1){
       
        //Turnon_LED();
        
        //////////////////////////////////// Stage 1 (From white square to the Entrance of the Circle) //////////////////////////////////////

        if (stage == 1){
            if(ReadSenAndDevi()==-2147483648){ PID_Wall();}
            else if(checkForTurns() == 1){                                                                      //right Turn
                std::cout<<"Current stage = "<<stage<<" >> Turning Right at right junction"<<std::endl;
                TurnNintey(1,0.2,0.6,0.7);
            }
            else if(checkForTurns() == 2){                                                                      //left Turn
                std::cout<<"Current stage = "<<stage<<" >> Turning left at left junction"<<std::endl;
                TurnNintey(2,0.2,0.6,0.7);
            }
            else if(checkForTurns() == 3){                                                                      //T junction
                std::cout<<"Current stage = "<<stage<<" >> Turning left at T junction"<<std::endl;
                TurnNintey(2,0.2,0.6,0.7);
                stage ++;
                base_speed = 0.18;
                std::cout<<"Current stage = "<<stage<<" >> X-Tumbler robot at quadrant = "<<quad_num<<std::endl;
            }
            else{PID_line();}
        }
      
        //////////////////////////////////// Stage 2 (Going along the circumferrence while indicating the quadrant numbers) //////////////////////////////////////     
        
        if (stage == 2){
            if(checkForTurns() == 1){                                                                           //right Turn detected
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
                TurnNintey(1,0.2,0.7,0.8);  
                stage ++;
            } 
            else{PID_line();}  
        }
      
        //////////////////////////////////// Stage 3 (Color Detection and leaving the Circle ) //////////////////////////////////////

        if (stage == 3){
            while(robot->step(timeStep) != -1){
                //std::cout << Front_sensor[0]->getValue() << std::endl;
                if((Box_detected == false) && (box_sens_cnt >= 4)){
                    stop();
                    int begin = robot -> getTime();
                    while(robot->step(timeStep) != -1){ 
                        if ((robot->getTime() - begin) >= 0){
                            Mot3->setVelocity(-0.5); Mot2->setVelocity(0.5);
                        }
                        if (robot->getTime() - begin >=2.5){
                            Mot3->setVelocity(0.0); Mot2->setVelocity(0.0);
                        }
                        if ((robot->getTime() - begin) >= 2.5){
                            Mot1->setVelocity(0.5); 
                        }
                        if (robot->getTime() - begin >=5.1){
                            Mot1->setVelocity(0.0); 
                        }
                        if (((robot->getTime() - begin) >= 5.1)){
                            Mot3->setVelocity(0.5); Mot2->setVelocity(-0.5);
                        }
                        if (robot->getTime() - begin >=7.5){
                            Mot3->setVelocity(0.0); Mot2->setVelocity(0.0);
                        }
                        if (robot->getTime() - begin >=7.5){
                            Mot4->setVelocity(-0.9); 
                        }
                        if (robot->getTime() - begin >=8){
                            Mot4->setVelocity(0.0); 
                        }
                        //enter code here to detect front colour
                        if (((robot->getTime() - begin) >= 8) && ((robot->getTime() - begin) <= 8.5) && (!color1)){
                            color1 = getColor();
                        }
                        if ((robot->getTime() - begin) >=8.5){
                            Mot1->setVelocity(-0.25);
                        }
                        if (robot->getTime() - begin >=11.5){
                            Mot1->setVelocity(0.0);
                        }
                        if ((robot->getTime() - begin) >=11.5){
                            Mot4->setVelocity(-0.8);
                        }
                        if (robot->getTime() - begin >=13.5){
                            Mot4->setVelocity(0.0);
                        }
                        //enter code here to detect bottom colour
                        if (((robot->getTime() - begin) >= 13.5) && ((robot->getTime() - begin) <= 14) && (!color2)){
                            color2 = getColor();
                        }
                        if ((robot->getTime() - begin) >=14){
                            Mot4->setVelocity(-0.8);
                        }
                        if (robot->getTime() - begin >=16){
                            Mot4->setVelocity(0.0);
                        }
                        //enter code here to detect back colour
                        if (((robot->getTime() - begin) >= 16) && ((robot->getTime() - begin) <= 16.5) && (!color3)){
                            color3 = getColor();
                        } 
                        if (color1 && color2 && color3){  
                            if(junction_count%2 == 0){
                                front_color = color3;
                                bottom_color = color2;
                            }
                            
                            else{
                                front_color = color1;
                                bottom_color = color2;
                            }
                            
                            //std::cout << "color1: " <<color1<< std::endl;
                            //std::cout << "color2: " <<color2<< std::endl;
                            //std::cout << "color3: " <<color3<< std::endl;
                            difference = abs(front_color - bottom_color);
                            
                        }
                        if (robot->getTime() - begin >=17){
                            Mot1->setVelocity(0.25);
                        }
                        if (robot->getTime() - begin >=19.5){
                            Mot1->setVelocity(0.0);
                        }
                        //enter code here to turn the robot 90 degrees to put the box down
                        if ((robot->getTime() - begin <=19.5) && (robot->getTime() - begin) >=17){
   
                            double rotationEndTime = robot->getTime() + 1.5;
    
                            while (robot->step(timeStep) != -1){
                                if (rotationEndTime > robot->getTime()){
                                      setMotorSpeeds(0.05,-0.05);
                                }
                                else{
                                    stop();
                                    break;
                                }
                            } 
                        }
                        if (((robot->getTime() - begin) >= 19.5)){
                            Mot3->setVelocity(-0.5); Mot2->setVelocity(0.5);
                        }
                        if (robot->getTime() - begin >=20.5){
                            Mot3->setVelocity(0.0); Mot2->setVelocity(0.0);
                        }
                        if (robot->getTime() - begin >=20.5){
                            Mot1->setVelocity(-1);
                        }
                        if (robot->getTime() - begin >=22){
                            Mot1->setVelocity(0.0);
                        }
                        if ((robot->getTime() - begin) >= 22){
                            if ((robot->getTime() - begin <=24.5) && ((robot->getTime() - begin >=22))){
                                double rotationEndTime = robot->getTime() + 1.5;
    
                                while (robot->step(timeStep) != -1){
                                    if (rotationEndTime > robot->getTime()){
                                          setMotorSpeeds(-0.05,0.05);
                                     }
                                    else{
                                        stop();
                                        break;
                                    }
                                }
                            }
                        }
                        if (((robot->getTime() - begin) >= 24.5)){
                            Mot3->setVelocity(0.5); Mot2->setVelocity(-0.5); Mot4->setVelocity(-1);
                        }
                        if (robot->getTime() - begin >=26){
                            Mot3->setVelocity(0.0); Mot2->setVelocity(0.0); Mot4->setVelocity(0.0);
                            break;
                        }
                    }
                    std::cout << "[Menu - Red = 1 | Green = 2 | Blue = 3]" << std::endl;
                    std::cout << "Front Color : " << front_color << std::endl; 
                    std::cout << "Bottom Color : " << bottom_color << std::endl; 
                    std::cout << "Difference : " << difference << std::endl; 
                    Box_detected = true;
                }  
                
                else if (sonar[0]->getValue() < 40){
                    box_sens_cnt++;
                } 
                else if(checkForTurns() == 1){                                                                 //right Turn
                    std::cout<<"Current stage = "<<stage<<" >> Turning Right at right junction"<<std::endl;
                    if(Box_detected == false){
                        TurnNintey(1,0.2,0.6,0.75);
                                                
                    }
                    else if(Box_detected == true){
                        TurnNintey(1,0.2,0.7,0.6);
                        stage ++;
                        base_speed = 0.35;
                        break;
                    }
                }
                else if(checkForTurns() == 2){                                                                 //left Turn
                    std::cout<<"Current stage = "<<stage<<" >> Turning left at left junction"<<std::endl;
                    if(Box_detected == false){
                        TurnNintey(2,0.2,0.6,0.75);
                        
                    }
                    else if(Box_detected == true){
                        TurnNintey(2,0.2,0.7,0.6);
                        stage ++;
                        base_speed = 0.35;
                        break;
                    }
                }
                else if(checkForTurns() == 3){                                                                             //T- junction
                    std::cout<<"Current stage = "<<stage<<" >> T-junction Detected"<<std::endl;
                    junction_count ++;
                    if((junction_count % 2 == 0) && Box_detected == false){TurnNintey(1,0.2,0.6,0.75);}                    //turn right                   
                    else if((junction_count == 4) && Box_detected == true){TurnNintey(1,0.2,0.6,0.75);}                    //turn right
                    else if((junction_count % 2 == 0) && Box_detected == true){TurnNintey(2,0.2,0.6,0.75);}                //turn left                                                                        
                    else{ideal(robot->getTime(),2,0.3);}
                }
                else{PID_line();}   
            }
        } 

        //////////////////////////////////// Stage 4 (Dash Line Following and Entering to the Ramp ) //////////////////////////////////////

        if (stage == 4){
            if(ReadSenAndDevi()==-2147483648){
                base_speed = 0.2;
                ideal(robot->getTime(),2,0.05);
                doted_line_detected = true;
            }
            else if(checkForTurns() == 3){                                                                  //T junction on ramp
                if((doted_line_detected = true)){
                    if((difference % 2) == 0){
                        TurnNintey(2,0.2,0.8,0.8);
                        traveled_distance[0] =  Encorders[0]->getValue();
                        std::cout<<"Current stage = "<<stage<<" >> Difference is even, X-tumbler turning left"<<std::endl;
                        stage = stage + 2;
                        base_speed = 0.2;
                    }
                    else{
                        TurnNintey(1,0.2,0.8,0.8);
                        traveled_distance[0] =  Encorders[0]->getValue();
                        std::cout<<"Current stage = "<<stage<<" >> Difference is odd, X-tumbler turning right"<<std::endl;
                        stage = stage + 1;
                        base_speed = 0.2;
                    }
                }
                else{
                  std::cout<<"Current stage = "<<stage<<"False detected"<<std::endl;
                  ideal(robot->getTime(),2,0.05);
                }
            }
            else{PID_line();}
        }
      
        ////////////////////////////// Stage 5 (If robot turned right at T junction at ramp) //////////////////////////////////////////////////// 

        if (stage == 5){
            if(get_linear_distance(Encorders[0]->getValue() - traveled_distance[0]) < 0.53){
                ideal(robot->getTime(),2,0.1);
                PID_line();
            }
            else{
                while (robot->step(timeStep) != -1){
                    if(checkForTurns() == 1 && pillar_counter == 1){                    //right Turn
                        std::cout<<"Current stage = "<<stage<<" >>  "<<pillar_counter<<" pillar detected.. Correct direction"<<std::endl;
                        TurnNintey(1,0.2,0.8,0.8);
                        base_speed = 0.2;
                        stage = stage + 2;
                        break;
                    }
                    else if(checkForTurns() == 2){                                      //left Turn
                        std::cout<<"Current stage = "<<stage<<" >> Wrong direction"<<std::endl;
                        correct_error(1);
                        base_speed = 0.2;
                        stage = stage + 2;
                        break;
                    }
                    else{
                        if(wall[0]->getValue()>160 || wall[1]->getValue()>160){
                            stop();
                            if(verify_pillar() == true){
                              pillar_counter ++;
                              ideal(robot->getTime(),2,0.15);
                            }
                            else{
                              PID_line();
                            }
                  
                        }
                        else{
                            PID_line();
                        }  
                    } 
                }
            }
        }

        ////////////////////////////// Stage 6 (If robot turned left at T junction at ramp) //////////////////////////////////////////////////// 
        
        if (stage == 6){
            if(get_linear_distance(Encorders[0]->getValue() - traveled_distance[0]) < 0.53){
                ideal(robot->getTime(),2,0.1);
                PID_line();
            }
            else{
                while (robot->step(timeStep) != -1){
                    if(checkForTurns() == 1){                                                                   //right Turn
                        std::cout<<"Current stage = "<<stage<<" >> Wrong direction"<<std::endl;
                        correct_error(2);
                        base_speed = 0.2;
                        stage = stage + 1;
                        break;
                    }
                    else if(checkForTurns() == 2 && pillar_counter == 2){ //left Turn
                        std::cout<<"Current stage = "<<stage<<" >>  "<<pillar_counter<<" pillar detected.. Correct direction"<<std::endl;
                        TurnNintey(2,0.2,0.8,0.8);
                        base_speed = 0.2;
                        stage = stage + 1;
                        break;
                    }
                    else{
                        if(wall[0]->getValue()>150 || wall[1]->getValue()>150){
                            if(verify_pillar() == true){
                              pillar_counter ++;
                              std::cout<<pillar_counter<<std::endl;
                              ideal(robot->getTime(),2,0.15);
                            }
                            else{
                              PID_line();
                            }
                  
                        }
                        else{
                            PID_line();
                        }  
                   } 
                }
            }
        }

        ////////////////////////////////////////////////// Stage 7 (Passing the Gates ) //////////////////////////////////////////////////

        if (stage == 7){
            if (!Gate_1 && Dash()){                                                  // Stop at White Dash Line and srart passing Gate_1
                stop();
                //std::cout << Front_sensor[0]->getValue() << std::endl;
                if ((Gate1_sens_count > 10)){
                    if (open_count > 5){
                        ideal(robot->getTime(),2,0.3);
                        Gate_1 = true;
                        gate_run = robot->getTime();
                    }
                    else if (Front_sensor[0]->getValue() < 1400){
                        open_count ++;
                    }
                }
                else{
                    if (Front_sensor[0]->getValue() > 1400){Gate1_sens_count ++;}
                }
            }
            else if (Gate_1 && whiteSquare() && (robot->getTime()-gate_run > 5)){                                 // Stop at White Square
                ideal(robot->getTime(), 2, 0.7);
                stage++;
            }
            else{
                PID_line();
            }
        }
        
        /////////////////////////////////////////////////////  Final Stage (Stop at White Square) ////////////////////////////////////////////
        
        if (stage == 8){
            stop();
        }
        
    }
}

//////////////////////////////////////////////////// Function for setting Motor Speeds /////////////////////////////////////////////////////

void setMotorSpeeds(float leftSpeed, float rightSpeed){
    wheels[0]->setVelocity(get_angular(leftSpeed));
    wheels[1]->setVelocity(get_angular(rightSpeed));
}

////////////////////////////////////////// Function for Reading Line Following Senosor Values ////////////////////////////////////////////

void ReadSen(){
    for (int i = 0; i<8; i++){
    SL[i] = (ir[i]->getValue()) < threshold; 
  }
}

///////////////////////////////// Function for Reading line follwing sensor values and calculating deviation ////////////////////////////

double ReadSenAndDevi(){
    for (int i = 0; i<8; i++){
    //std::cout<<"S1:"<<SL[0]<<" S2:"<<SL[1]<<" S3:"<<SL[2]<<" S4:"<<SL[3]<<" S5:"<<SL[4]<<" S6:"<<SL[5]<<" S7:"<<SL[6]<<" S8:"<<SL[7]<<std::endl;
    //std::cout<<"S1:"<<ir[0]->getValue()<<" | S2:"<<ir[1]->getValue()<<" | S3:"<<ir[2]->getValue()<<" | S4:"<<ir[3]->getValue()<<" | S5:"<<ir[4]->getValue()<<" | S6:"<<ir[5]->getValue()<<" | S7:"<<ir[6]->getValue()<<" | S8:"<<ir[7]->getValue()<<std::endl;
        SL[i] = (ir[i]->getValue()) < threshold; 
        LEDS[i]->set(255*SL[i]);
    }

    int deviation = 0;
    deviation = (10*SL[0] + 20*SL[1] + 30*SL[2] + 40*SL[3] + 50*SL[4] + 60*SL[5] + 70*SL[6] + 80*SL[7])/(SL[0] + SL[1] + SL[2] + SL[3] + SL[4] + SL[5] + SL[6] + SL[7] );
    return deviation;
}

////////////////////////////////////////////// Function for Checking for junctions ////////////////////////////////////////////////////

int checkForTurns(){
    double mark = 0;
    ReadSen();
    //std::cout<<"S1:"<<ir[0]->getValue()<<" | S2:"<<ir[1]->getValue()<<" | S3:"<<ir[2]->getValue()<<" | S4:"<<ir[3]->getValue()<<" | S5:"<<ir[4]->getValue()<<" | S6:"<<ir[5]->getValue()<<" | S7:"<<ir[6]->getValue()<<" | S8:"<<ir[7]->getValue()<<std::endl;
    if (SL[0]==1 && SL[1]==1 && SL[2]==1 && SL[3]==1 && SL[4]==1  && SL[6]==0 && SL[7]==0){
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

//////////////////////////////////////////////////// Function for Turning ninty degrees //////////////////////////////////////////

void TurnNintey(int direction ,float user_speed, float user_forward_time, float user_rot_time){
    
    ///default TurnNintey(int direction ,0.3,1.1, 0.4)
    
    if (direction == 1){
        LS = user_speed;
        RS = -1 * user_speed;
    }
     
    else if(direction == 2){
        LS = -1 * user_speed;
        RS = user_speed;
     }
     
    double forwardEndTime = robot->getTime() + user_forward_time;

    while (robot->step(timeStep) != -1){
        if (forwardEndTime > robot->getTime()){
            setMotorSpeeds(user_speed,user_speed);
        }
        else{
            stop();
            break;
        }
    }
    
    double rotationEndTime = robot->getTime() + user_rot_time;
    
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

//////////////////////////////////////////////////////////// Function for stop ////////////////////////////////////////////////////////////

void stop(){
    setMotorSpeeds(0, 0);
}
  
///////////////////////////////////////////////////////////// Line Following PID Function //////////////////////////////////////////////

void PID_line(){  
    current_position = ReadSenAndDevi();
    line_error = (int)current_position - 45;
    
    propotional = line_error * Kp;
    integral = (integral + line_error) * Ki;
    derivative = (line_error - prev_error_line) * Kd;
    
    correction = propotional + integral + derivative;
    //std::cout<<"correction ="<<correction<<std::endl;
    if (correction > max_change){correction = max_change;}
    if (correction < -1*max_change){correction = -1 * max_change;}

    setMotorSpeeds((base_speed - correction), (base_speed + correction));
    prev_error_line = line_error;
}


///////////////////////////////////////////////////////// Wall Following PID Function /////////////////////////////////////////////////

void PID_Wall(){
    l_d = wall[0]->getValue();
    r_d = wall[1]->getValue();

    //std::cout<<"Left :"<<l_d<<"     right :"<<r_d<<std::endl;

    if (l_d < wall_margin && r_d > wall_margin){
        wall_error = r_d - 195;
        //std::cout<<"right detected | wall error = "<<wall_error<<std::endl;
        propotional_W = wall_error * Kpw;
        integral_W = (integral_W + wall_error) * Kiw;
        derivative_W = (wall_error - prev_error_Wall) * Kdw;

        correction_w = propotional_W + integral_W + derivative_W;

        if (correction_w > max_change){correction_w = max_change;}
        if (correction_w < -1*max_change){correction_w = -1 * max_change;}
    
        setMotorSpeeds((base_speed - correction_w), (base_speed + correction_w));
    }

    else if(r_d < wall_margin && l_d > wall_margin){
        wall_error = l_d - 195;
        //std::cout<<"left detected | wall error = "<<wall_error<<std::endl;
        propotional_W = wall_error * Kpw;
        integral_W = (integral_W + wall_error) * Kiw;
        derivative_W = (wall_error - prev_error_Wall) * Kdw;

        correction_w = propotional_W + integral_W + derivative_W;
      
        if (correction_w > max_change){correction_w = max_change;}
        if (correction_w < -1*max_change){correction_w = -1 * max_change;}
        
        setMotorSpeeds((base_speed + correction_w), (base_speed - correction_w));
    }
    
    else if(r_d > wall_margin && l_d > wall_margin){
        wall_error = l_d - 195;
        //std::cout<<"both detected | wall error = "<<wall_error<<std::endl;
        propotional_W = wall_error * Kpw;
        integral_W = (integral_W + wall_error) * Kiw;
        derivative_W = (wall_error - prev_error_Wall) * Kdw;

        correction_w = propotional_W + derivative_W + integral_W;
        if (correction_w > max_change){correction_w = max_change;}
        if (correction_w < -1*max_change){correction_w = -1 * max_change;}
        
        setMotorSpeeds((base_speed + correction_w), (base_speed - correction_w));
    }
    
    else{
      //std::cout<<"going straight"<<std::endl;
      setMotorSpeeds(base_speed,base_speed);      
    }
    
    prev_error_Wall = wall_error;
}

//////////////////////////////////////////////// Function for getting Linear Velocity of Motors /////////////////////////////////////////

float get_linear(float an_speed){
    return wheel_radius * an_speed;
}

//////////////////////////////////////////////// Function for getting Angular Velocity of Motors /////////////////////////////////////////

float get_angular(float li_speed){
    return li_speed/ wheel_radius;
}

//////////////////////////////////////////////// Function for getting Linear Distance ///////////////////////////////////////////////////

float get_linear_distance(float radial_distance){
    return wheel_radius * radial_distance;
}

/////////////////////////////////////////////// Function for calculate Radius of the Circle /////////////////////////////////////////////

void calculate_circumference(float left_x,float right_x){
    float single_quadrant = (left_x + right_x)/6;
    std::cout<<"radius = "<<((single_quadrant*200)/3.1415)<<" | circumference = "<<single_quadrant*400<<std::endl;
}

////////////////////////////////////////////// Function for moving robot foward at the begining of the run //////////////////////////////

void ideal(double pre_time , int pref, double period){
    if (pref == 1){
        while (robot->step(timeStep) != -1){
            if((robot->getTime()-pre_time) > period){break;}
            else{stop();}
        }
    } 
    else if(pref == 2){
        while (robot->step(timeStep) != -1){
            if((robot->getTime()-pre_time) > period){break;}
            else{setMotorSpeeds(0.3,0.3);}
        }
    }
} 

///////////////////////////////////////////////////////// Function for controlling LED Pannel //////////////////////////////////////////

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

///////////////////////////////////////////////////////// Function for turning 180 degrees //////////////////////////////////////////

void turn_around(){
    double UturnEndTime = robot->getTime() + UTurnrotateTime;
    while (robot->step(timeStep) != -1){
        if(UturnEndTime > robot->getTime()){
            setMotorSpeeds(-0.2,0.2);
        }    
        else{
            stop();
            break;
        }    
    }
}

///////////////////////////////////////////////////////// Function for correcting the path, if missed at the ramp //////////////////////////////////////////

void correct_error(int turn_error){
    turn_around();
    int turnCount = 0;
    if(turn_error == 1){
        while (robot->step(timeStep) != -1){
            if(checkForTurns() == 1){
                turnCount++;
                if(turnCount == 2){
                    TurnNintey(1,0.2,0.8,0.8);
                    break;
                }
                else{
                    ideal(robot->getTime(),2,1);
                }
            }
            else if(ReadSenAndDevi()==-2147483648){
                ideal(robot->getTime(),2,0.05);
            }
            else{PID_line();}
        }
    } 
    
    else if(turn_error == 2){
        while (robot->step(timeStep) != -1){
            if(checkForTurns() == 2){
                turnCount++;
                if(turnCount == 2){
                    TurnNintey(2,0.2,0.8,0.8);
                    break;
                }
                else{
                    ideal(robot->getTime(),2,1);
                }
            }
            else if(ReadSenAndDevi()==-2147483648){
                ideal(robot->getTime(),2,0.05);
            }
            else{PID_line();}
        }
    }      
}

/////////////////////////////////////////////////// Function for Detecting colors of the box //////////////////////////////////////////

int getColor(){
	
    int r=0;
    int g=0;
    int b=0;
    int color;
    
    Camera *cm;
    cm=robot->getCamera("CAM");
    cm->enable(timeStep);
    cm->recognitionEnable(timeStep);
    
    int image_width = cm->getWidth();
    int image_height = cm->getHeight();
    const unsigned char *image = cm->getImage();
        
    for (int x = 0; x < image_width; x++){
        for (int y = 0; y < image_height; y++) {
            r = cm->imageGetRed(image, image_width, x, y);
            g = cm->imageGetGreen(image, image_width, x, y);
            b = cm->imageGetBlue(image, image_width, x, y);
        }
    }
    //std::cout <<r<< std::endl;
    //std::cout <<g<< std::endl; 
    //std::cout <<b<< std::endl;
  
    if((r > 45) && (g <45) && (b <45)){
       // std::cout <<"Red"<< std::endl; 
        color=1  ;  
    }
    else if( (r < 45) && (g <45) && (b > 45) ){
        //std::cout <<"Blue"<< std::endl;
        color=3; 
    }
    else if((r < 45) && (g >45) && (b <45)){
        //std::cout <<"Green"<< std::endl; 
        color=2; 
    }
    else{
        //std::cout <<"no colour"<< std::endl;
        color=0;  
    } 
    return color;
}

/////////////////////////////////////////////////// Function for Detecting Dash Line at gates //////////////////////////////////////////

bool Dash(){
    ReadSen();
    if (SL[2] && SL[3] && SL[4] && SL[5]){return true;}
    return false;
}

//////////////////////////////////////////////////// Function for detecting white square /////////////////////////////////////////////

bool whiteSquare(){
    if (SL[0] && SL[1] && SL[2] && SL[3] && SL[4] && SL[5] && SL[6] && SL[7]){return true;}
    return false;
}

//////////////////////////////////////////////////// Function for detecting Pillars /////////////////////////////////////////////////

bool verify_pillar(){
    int distance_counter = 0;
    for (int i=0; i<50 ; i++){
      if (wall[0]->getValue()>160 || wall[1]->getValue()>160){
        distance_counter += 1;
      }
      else{break;}
    } 
    
    if (distance_counter >= 45){return true;}
    
    else{return false;}
}

//////////////////////////////////////////////////////////************* T ***********//////////////////////////////////////////////////
//////////////////////////////////////////////////////////************* E ***********//////////////////////////////////////////////////
//////////////////////////////////////////////////////////************* A ***********//////////////////////////////////////////////////
//////////////////////////////////////////////////////////************* M ***********//////////////////////////////////////////////////
//////////////////////////////////////////////////////////***************************//////////////////////////////////////////////////
//////////////////////////////////////////////////////////************* X ***********//////////////////////////////////////////////////
//////////////////////////////////////////////////////////************* - ***********//////////////////////////////////////////////////
//////////////////////////////////////////////////////////************* T ***********//////////////////////////////////////////////////
//////////////////////////////////////////////////////////************* U ***********//////////////////////////////////////////////////
//////////////////////////////////////////////////////////************* M ***********//////////////////////////////////////////////////
//////////////////////////////////////////////////////////************* B ***********//////////////////////////////////////////////////
//////////////////////////////////////////////////////////************* L ***********//////////////////////////////////////////////////
//////////////////////////////////////////////////////////************* E ***********//////////////////////////////////////////////////
//////////////////////////////////////////////////////////************* R ***********//////////////////////////////////////////////////