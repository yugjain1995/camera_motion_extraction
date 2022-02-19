/************************************************************************************
Author - Yug Jain
***********************************************************************************/

/************************************************************************************
Use this code to test FeatureDetector class
************************************************************************************/


/// header
#include<feature_detection.h>
#include <string>
/// header

int main(int argc, char **argv){

  // Test feature detection
  FeatureDetector test;
  test.imageSubscriber(argc, argv);
  return 0;
}