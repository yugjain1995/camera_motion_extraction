/// [headers]
#include <ros/console.h>
#include <opencv2/highgui/highgui.hpp>
#include<feature_detection.h>
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


void FeatureDetector::setImage(){
  featureImage = image.clone();
}


void FeatureDetector::detectFeatures(){
/// Check that image is not empty
  if(featureImage.empty()){ROS_ERROR("Image frame is empty"); return;}
/// Get feature points
  orb->detect(featureImage, keypoints, cv::noArray());
  return;
}


void FeatureDetector::displayKeypoints(){
  if(featureImage.empty()){ROS_ERROR("Image frame is empty"); return;}
  if(keypoints.empty()){ROS_ERROR("No keypoints"); return;}

/// Create window
  cv::namedWindow("view1");

/// Draw keypoints on image
  cv::drawKeypoints(featureImage, keypoints, keypointImage,
                  cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);

/// Put the image with keypoints on view
  if(keypointImage.empty()){ROS_ERROR("Image frame is empty"); return;}
  cv::imshow("view1", keypointImage);
  cv::waitKey(30);
  return;
}

void FeatureDetector::imageCompute(){
  setImage();
  detectFeatures();
  displayKeypoints();
  return;
}