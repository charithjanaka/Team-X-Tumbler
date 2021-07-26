#include <webots/Robot.hpp>
#include <webots/Camera.hpp>


#define TIME_STEP 64
using namespace webots;

double r=0.0;
double g=0.0;
double b=0.0;


int main(int argc, char **argv) {
  Robot *robot = new Robot();
  Camera *cm;
  cm=robot->getCamera("CAM");
  cm->enable(TIME_STEP);
  cm->recognitionEnable(TIME_STEP);
  
//Get the image
  while (robot->step(TIME_STEP) != -1) {
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
    std::cout <<r<< std::endl;
    std::cout <<g<< std::endl; 
    std::cout <<b<< std::endl;
  
    if((r > 150) && (g <50) && (b <50)){
      std::cout <<"Red"<< std::endl;   
    }
    else if( (r < 50) && (g <50) && (b > 150) ){
      std::cout <<"Blue"<< std::endl;
     }
    else if((r < 50) && (g >150) && (b <50)){
      std::cout <<"Green"<< std::endl;
     }
    else{
    std::cout <<"no colour"<< std::endl;
      }
  }
  delete robot;
  return 0;
}
