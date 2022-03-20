/************************************************************************************
Author - Yug Jain
***********************************************************************************/

/************************************************************************************
This code provide function definitions for FeatureMatcher class which can be used to
compute keypoint match for kepoints of two images. Keypoints and descriptors are
computed using inherited class FeatureDetector.
************************************************************************************/


/// [headers]
#include <ros/console.h>
#include <opencv2/highgui/highgui.hpp>
#include <feature_detection.h>
#include <feature_matching.h>
#include <chrono>
/// [headers]


/******************************************************************/
FeatureMatcher::FeatureMatcher(){
  this->bruteForceMatcher = cv::BFMatcher::create(cv::NORM_HAMMING2, true);
}
/******************************************************************/


/******************************************************************/
void FeatureMatcher::makePrevious(){
  /// Copy the latest recieved image and corresponding
  /// keypoints and descriptors to variable storing previous 
  /// image and corresponding keypoints and descriptors
  /// to match with next latest image
    this->preImage = this->image.clone();
    this->keypoints1 = this->keypoints;
    this->descriptors1 = this->descriptors.clone();
}
/******************************************************************/


/******************************************************************/
void FeatureMatcher::imageCompute(){
  if(this->preImage.empty()){
    ROS_WARN("No previous image!!");
    if(this->image.empty()){
      ROS_ERROR("No current image!!");
      return;
    }

  /// Detect keypoints and compute descriptors for latest
  /// recieved image.
    if(this->detectFeatures()) return;
    if(this->computeDescriptors()) return;

    this->makePrevious();
  }
  else{
    if(this->image.empty()){
      ROS_ERROR("No current image!!");
      return;
    }
    else{
    /// Detect keypoints and compute descriptors for latest
    /// recieved image.
      if(this->detectFeatures()) return;
      if(this->computeDescriptors()) return;

    /// Match the keypoints with previous image
      this->match();

    /// Display keypoint matching
      #ifdef DEBUG_MODE
        cv::namedWindow("Keypoint matching", cv::WINDOW_NORMAL);
        cv::resizeWindow("Keypoint matching", 1920, 1080);
        cv::imshow("Keypoint matching", this->matchedImage);
        cv::waitKey(30);
      #endif
      
    /// Make recently recieved image previous
      this->makePrevious();
    }
  }
}
/******************************************************************/


/******************************************************************/
void FeatureMatcher::match(){
  this->bruteForceMatcher->match(this->descriptors, this->descriptors1, this->matches);
  cv::drawMatches(this->image, this->keypoints, this->preImage, this->keypoints1, this->matches, this->matchedImage);
}
/******************************************************************/