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
#include "opencv2/calib3d/calib3d.hpp"
#include <motion_extraction.h>
#include <chrono>
/// [headers]


/******************************************************************/
MotionEstimate2D2D::imageCompute(){
  this->matchAndDisplay();
  this->cameraPoseEstimate();
}
/******************************************************************/

/******************************************************************/
MotionEstimate2D2D::cameraPoseEstimate(){
// Start the timer
  auto start = std::chrono::high_resolution_clock::now();
// Get essential matrix from keypoints
  findEssentialMat();
// Recover pose from essential matrix
  recoverPose();
// End the timer and compute time elapsed
  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
// Display Pose
  ROS_INFO_STREAM( "\nR = \n" << _R << "\n" );
  ROS_INFO_STREAM( "\nt = \n" << _t << "\n" );
// Display time elapsed
  ROS_INFO_STREAM( "Total time to estimate pose = " << std::to_string(duration.count()) << std::endl );
}
/******************************************************************/

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

/******************************************************************/
MotionEstimate2D2D::findEssentialMat(){
  
}
/******************************************************************/

// 1 weekend
/******************************************************************/
MotionEstimate2D2D::recoverPose(){
  
}
/******************************************************************/