/************************************************************************************
Author - Yug Jain
***********************************************************************************/

/************************************************************************************
Use this code to test MotionEstimate2D2D class
************************************************************************************/


/// header
#include<motion_extraction.h>
#include <string>
/// header

int main(int argc, char **argv){

  // Test feature detection
  MotionEstimate2D2D test;
  test.imageSubscriber(argc, argv);
  return 0;
}