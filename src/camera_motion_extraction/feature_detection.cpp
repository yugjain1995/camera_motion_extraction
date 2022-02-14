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

void FeatureDetector::detectFeatures(){
/// Check that image is not empty
  if(image.empty()){ROS_ERROR("Image frame is empty"); return;}
/// Detect feature points
  orb->detect(image, keypoints, cv::noArray());
  return;
}

void FeatureDetector::computeDescriptors(){
  /// Check that image is not empty
  if(image.empty()){ROS_ERROR("Image frame is empty!!"); return;}
  /// Check that keypoints is not empty
  if(keypoints.empty()){ROS_ERROR("No keypoints!!"); return;}
  /// Compute descriptors from provided image and corresponding keypoints
  orb->compute(image, keypoints, descriptors);
  if(descriptors.empty()){ROS_WARN("Not able to compute descriptors!!");}
}

void FeatureDetector::displayKeypoints(){
  if(image.empty()){ROS_ERROR("Image frame is empty"); return;}
  if(keypoints.empty()){ROS_ERROR("No keypoints"); return;}

/// Create window
  cv::namedWindow("Detected Kepoints");

/// Draw keypoints on image
  cv::drawKeypoints(image, keypoints, keypointImage,
                  cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);

/// Put the image with keypoints on view
  if(keypointImage.empty()){ROS_ERROR("Image frame is empty"); return;}
  cv::imshow("view1", keypointImage);
  cv::waitKey(30);
  return;
}

void FeatureDetector::imageCompute(){
  detectFeatures();
  displayKeypoints();
  cv::destroyWindow("Detected Kepoints");
  computeDescriptors();
  return;
}