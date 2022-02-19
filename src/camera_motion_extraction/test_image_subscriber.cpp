/************************************************************************************
Author - Yug Jain
***********************************************************************************/

/************************************************************************************
Use this code to test RosToCvmat class
************************************************************************************/


/// [headers]
#include<image_subscriber.h>
#include<string>
/// [headers]

int main(int argc, char **argv){

// Test image subscription
  RosToCvmat img_conv;
  img_conv.imageSubscriber(argc, argv);
  return 0;
}