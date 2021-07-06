#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Camera.hpp>

#define TIME_STEP 16

using namespace webots;

double color1=0.0;    //red
double color2=0.0;    //green
double color3=0.0;     //blue
double front_color=0.0;
double bottom_color=0.0;
double val = 0.0;
int junc_count = 3;

float getColor();
Robot *robot = new Robot();


int main(int argc, char **argv) 
{
  Motor *Mot = robot->getMotor("rm1");
  Motor *Mot2 = robot->getMotor("rm2");
  Motor *Mot3 = robot->getMotor("lm1");
  Motor *Mot4 = robot->getMotor("lm2");
  Motor *Mot5 = robot->getMotor("rm3");
  

  Mot->setPosition(INFINITY);
  Mot->setVelocity(0.0);
  Mot2->setPosition(INFINITY);
  Mot2->setVelocity(0.0);
  Mot3->setPosition(INFINITY);
  Mot3->setVelocity(0.0);
  Mot4->setPosition(INFINITY);
  Mot4->setVelocity(0.0);
  Mot5->setPosition(INFINITY);
  Mot5->setVelocity(0.0);

  
  while (robot->step(TIME_STEP) != -1) 
  {
   if (robot->getTime()  >= 0)
    {
      Mot->setVelocity(0.5);
      Mot2->setVelocity(0.5);
      if (robot->getTime()>=3.1){
        Mot->setVelocity(0.0);
        Mot2->setVelocity(0.0);
      }
    }
    if (robot->getTime() >= 3.1)
    {
      Mot3->setVelocity(0.05);
      Mot4->setVelocity(0.05);
      if (robot->getTime()>=3.9){
        Mot3->setVelocity(0.0);
        Mot4->setVelocity(0.0);
      }
    }
    if (robot->getTime() >=4)
    {
      Mot->setVelocity(-0.5);
      Mot2->setVelocity(-0.5);
      if (robot->getTime()>=6){
        Mot->setVelocity(0.0);
        Mot2->setVelocity(0.0);
        Mot5->setVelocity(0.5);
      }
    }
  
      if (robot->getTime() >=6.7){
        Mot5->setVelocity(0.0);
        color1 = getColor(); 
       // std::cout <<"color1: "<<color1<< std::endl;
      }
      if (robot->getTime()>=7.2){
        Mot5->setVelocity(0.5);
        if (robot->getTime() >=9.7){
          Mot5->setVelocity(0.0);
          color2 = getColor();
          //std::cout <<"color2: "<<color2<< std::endl;
        }
      }
      if (robot->getTime()>=10.2){
        Mot5->setVelocity(0.5);
        if (robot->getTime() >=12.7){
          Mot5->setVelocity(0.0);
          color3 = getColor();
         // std::cout <<"color3: "<<color3<< std::endl;
        }
      } 
    if((robot->getTime()>=13.2) && (robot->getTime()<=13.25)){
      if(junc_count%2 == 0){
          front_color = color1;
          bottom_color = color2;
      }
      else{
        front_color = color3;
        bottom_color = color2;
      }
      std::cout <<"color1: "<<color1<< std::endl;
      std::cout <<"color2: "<<color2<< std::endl;
      std::cout <<"color3: "<<color3<< std::endl;
      val = abs(front_color - bottom_color);
      std::cout <<val<< std::endl;  
    }
  }
  delete robot;
  return 0;
}

float getColor(){
	
    int r=0;
    int b=0;
    int g=0;
    float color;
    
    Camera *cm;
    cm=robot->getCamera("CAM");
    cm->enable(TIME_STEP);
    cm->recognitionEnable(TIME_STEP);
    
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
  
    if((r > 150) && (g <50) && (b <50)){
       // std::cout <<"Red"<< std::endl; 
        color=1.0  ;  
    }
    else if( (r < 50) && (g <50) && (b > 150) ){
        //std::cout <<"Blue"<< std::endl;
        color=3.0; 
    }
    else if((r < 50) && (g >150) && (b <50)){
        //std::cout <<"Green"<< std::endl; 
        color=2.0; 
    }
    else{
        //std::cout <<"no colour"<< std::endl;
        color=0.0;  
    } 
    return color;
}

