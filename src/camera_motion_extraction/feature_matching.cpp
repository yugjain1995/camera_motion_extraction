/// [headers]
#include <ros/console.h>
#include <opencv2/highgui/highgui.hpp>
#include <feature_detection.h>
#include <feature_matching.h>
#include <chrono>
/// [headers]

FeatureMatcher::FeatureMatcher(){
  bruteForceMatcher = cv::BFMatcher::create(cv::NORM_HAMMING2, true);
}

void FeatureMatcher::imageCompute(){
  if(image1.empty()){
    ROS_WARN("No previous image!!");
    if(image.empty()){
      ROS_ERROR("No current image!!");
      return;
    }
    detectFeatures();
    computeDescriptors();
    image1 = image.clone();
    keypoints1 = keypoints;
    descriptors1 = descriptors.clone();
  }
  else{
    if(image.empty()){
      ROS_ERROR("No current image!!");
      return;
    }
    else{
    /// Detect features on latest feteched image
      if(detectFeatures()) return;

    /// Compute descriptors for latest feteched image
      if(computeDescriptors()) return;

    /// Match the keypoints
      match();

    /// Display keypoint matching
      #ifdef DEBUG_MODE
        cv::namedWindow("Keypoint matching", cv::WINDOW_NORMAL);
        cv::resizeWindow("Keypoint matching", 1920, 1080);
        cv::imshow("Keypoint matching", matchedImage);
        cv::waitKey(30);
      #endif
      
    /// Clone to previous image data member to use it for next cycle
      image1 = image.clone();
      keypoints1 = keypoints;
      descriptors1 = descriptors.clone();
    }
  }
}

void FeatureMatcher::match(){
  bruteForceMatcher->match(descriptors, descriptors1, matches);
  cv::drawMatches(image, keypoints, image1, keypoints1, matches, matchedImage);
}