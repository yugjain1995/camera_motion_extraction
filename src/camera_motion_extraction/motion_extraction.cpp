/************************************************************************************
Author - Yug Jain
***********************************************************************************/

/************************************************************************************
This code provide function definitions for MotionEstimate2D2D class which can be used to
compute change of pose of camera given keypoint matches of the consecutive images from
a monocular camera.
************************************************************************************/

/// [headers]
#include <ros/console.h>
#include <opencv2/highgui/highgui.hpp>
#include <motion_extraction.h>
#include <chrono>
/// [headers]


/******************************************************************/
MotionEstimate2D2D::MotionEstimate2D2D(){
// Initialize camera intrinsics
  float fx = 1144.361;
  float fy = 1147.337;
  float cx = 966.359;
  float cy = 548.038;
  this->K = (Mat_<double>(3,3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
}
/******************************************************************/

// 2 weekday
/******************************************************************/
MotionEstimate2D2D::findEssentialMat(){
  
}
/******************************************************************/

// 1 weekend
/******************************************************************/
MotionEstimate2D2D::recoverPose(){
  
}
/******************************************************************/

// 2 weekday
/******************************************************************/
MotionEstimate2D2D::cameraPoseEstimate(){
  
}
/******************************************************************/


/******************************************************************/
MotionEstimate2D2D::imageCompute(){
  this->matchAndDisplay();
  this->cameraPoseEstimate();
}
/******************************************************************/