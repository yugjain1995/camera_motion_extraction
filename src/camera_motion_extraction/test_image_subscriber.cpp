/// [headers]
#include<image_subscriber.h>
#include<string>
/// [headers]

int main(int argc, char **argv){
  bool disp = false; // Display or Not the image recived from camera

// Parse options from argument list
  for(int i = 0; i<argc; i++){
    if(strcmp(argv[i], "-d") == 0){
      disp = true;
      break;
    }
  }

// Test image subscription
  RosToCvmat img_conv;
  img_conv.imageSubscriber(disp, argc, argv);
  return 0;
}