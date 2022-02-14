/// [headers]
#include <ros/console.h>
#include <opencv2/highgui/highgui.hpp>
#include <feature_detection.h>
#include <feature_matching.h>
#include <chrono>
/// [headers]

FeatureMatcher::FeatureMatcher(){
  bruteForceMatcher = create (cv::NORM_HAMMING2, bool crossCheck=true);
}

void FeatureMatcher::imageCompute(){
  if(image1.empty()){
    ROS_WARN("No previous image!!");
    if(image1.empty()){
      ROS_ERROR("No current image!!");
      return;
    }
    image1 = image.clone();
    keypoints1 = keypoints;
    descriptors1 = descriptors.clone();
  }
  else{
    detectFeatures();
    computeDescriptors();
    cv::namedWindow("Keypoint matching");
    cv::imshow("Keypoint matching", matchedImage);
    match("Keypoint matching");
    cv::destroyWindow("Detected Kepoints");
    image1 = image.clone();
    keypoints1 = keypoints;
    descriptors1 = descriptors.clone();
  }
}

void FeatureMatcher::match(string s = " "){
   bruteForceMatcher.match(descriptors, descriptors1, matches);
   if(strcmp(s, " ") != 0){
    cv::drawMatches(image, keypoints, image1, keypoints2, matches, matchedImage);
   }
}