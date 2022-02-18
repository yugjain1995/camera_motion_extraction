/// [headers]
#include <ros/console.h>
#include <opencv2/highgui/highgui.hpp>
#include <feature_detection.h>
#include <chrono>
/// [headers]

FeatureDetector::FeatureDetector(){
  orb = cv::ORB::create(100, 1.1f, 16, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, 20);
  if(orb != NULL){
    ROS_INFO("cv::ORB object created");
  }
  else
  {
    ROS_ERROR("cv::ORB returned NULL");
  }
}

bool FeatureDetector::detectFeatures(){
/// Detect feature points
  orb->detect(image, keypoints, cv::noArray());

/// Check that keypoints is not empty
  if(keypoints.empty()){ROS_ERROR("No keypoints found!!"); return 1;}
  
  return 0;
}

bool FeatureDetector::computeDescriptors(){
/// Compute descriptors from provided image and corresponding keypoints
  orb->compute(image, keypoints, descriptors);

/// Check is descriptors computed successfully
  if(descriptors.empty()){ROS_WARN("Not able to compute descriptors!!"); return 1;}
  
  return 0;
}

void FeatureDetector::displayKeypoints(){
/// Draw keypoints on image
  cv::drawKeypoints(image, keypoints, keypointImage,
                  cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);

/// Put the image with keypoints on view
  if(keypointImage.empty()){ROS_ERROR("keypointImage frame is empty"); return;}
  /// Create window
  cv::namedWindow("Detected Kepoints", cv::WINDOW_NORMAL);
  cv::resizeWindow("Detected Kepoints", 1920, 1080);
  cv::imshow("Detected Kepoints", keypointImage);
  cv::waitKey(30);
  return;
}

void FeatureDetector::imageCompute(){
  if(detectFeatures()) return;

  #ifdef DEBUG_MODE
    displayKeypoints();
  #endif
  
  if(computeDescriptors()) return;
  return;
}