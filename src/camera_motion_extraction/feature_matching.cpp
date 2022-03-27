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
  bruteForceMatcher = cv::BFMatcher::create(cv::NORM_HAMMING2, true);
}
/******************************************************************/


/******************************************************************/
void FeatureMatcher::makePrevious(){
  /// Copy the latest recieved image and corresponding
  /// keypoints and descriptors to variable storing previous 
  /// image and corresponding keypoints and descriptors
  /// to match with next latest image
    preImage = image.clone();
    keypoints1 = keypoints;
    descriptors1 = descriptors.clone();
}
/******************************************************************/


/******************************************************************/
void FeatureMatcher::imageCompute(){
  matchAndDisplay();
}
/******************************************************************/


/******************************************************************/
void FeatureMatcher::matchAndDisplay(){
  if(preImage.empty()){
    ROS_WARN("No previous image!!");
    if(image.empty()){
      ROS_ERROR("No current image!!");
      return;
    }

  /// Detect keypoints and compute descriptors for latest
  /// recieved image.
    if(detectFeatures()) return;
    if(computeDescriptors()) return;

    makePrevious();
  }
  else{
    if(image.empty()){
      ROS_ERROR("No current image!!");
      return;
    }
    else{
    /// Detect keypoints and compute descriptors for latest
    /// recieved image.
      if(detectFeatures()) return;
      if(computeDescriptors()) return;

    /// Match the keypoints with previous image
      match();

    /// Display keypoint matching
      #ifdef DEBUG_MODE
        cv::namedWindow("Keypoint matching", cv::WINDOW_NORMAL);
        cv::resizeWindow("Keypoint matching", 1920, 1080);
        cv::imshow("Keypoint matching", matchedImage);
        cv::waitKey(30);
      #endif
      
    /// Make recently recieved image previous
      makePrevious();
    }
  }
}
/******************************************************************/


/******************************************************************/
void FeatureMatcher::match(){
  bruteForceMatchermatch(descriptors, descriptors1, matches);
  cv::drawMatches(image, keypoints, preImage, keypoints1, matches, matchedImage);
}
/******************************************************************/