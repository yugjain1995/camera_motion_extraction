/// header
#include<feature_detection.h>
#include <string>
/// header

int main(int argc, char **argv){
  bool disp = false; // Display or Not the image recived from camera without the keypoints i.e. unaltered

// Parse options from argument list
  for(int i = 0; i<argc; i++){
    if(strcmp(argv[i], "-d") == 0){
      disp = true;
      break;
    }
  }

  // Test feature detection
  FeatureDetector test;
  test.imageSubscriber(disp, argc, argv);
  return 0;
}