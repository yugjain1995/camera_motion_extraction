/************************************************************************************
Author - Yug Jain
***********************************************************************************/

/************************************************************************************
Use this code to test FeatureMatcher class
************************************************************************************/


/// header
#include<feature_matching.h>
#include <string>
/// header

int main(int argc, char **argv){

  // Test feature detection
  FeatureMatcher test;
  test.imageSubscriber(argc, argv);
  return 0;
}