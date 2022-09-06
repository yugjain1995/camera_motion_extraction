/************************************************************************************
Author - Yug Jain
***********************************************************************************/

/************************************************************************************
This code provide function definitions for FeatureDetector class which can be used to
compute image keyopoint and corresponding descriptor for an image. image is recieveing
code is inheriited from class RosToCvmat.
************************************************************************************/

/// [headers]
#include <ros/console.h>
#include <opencv2/highgui/highgui.hpp>
#include <feature_detection.h>
#include <chrono>
/// [headers]

/******************************************************************/
FeatureDetector::FeatureDetector(){
  this->orb = cv::ORB::create(100, 1.1f, 16, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, 20);
  if(this->orb != NULL){
    ROS_INFO("cv::ORB object created");
  }
  else
  {
    ROS_ERROR("cv::ORB returned NULL");
  }
  #ifdef DEBUG_MODE
    ready = true;
  #endif
}
/******************************************************************/


/******************************************************************/
/// Finds ORB keypoints
/// If unable to detect keypoints then returns 1
bool FeatureDetector::detectFeatures(){
/// Detect feature points
  this->orb->detect(this->image, this->keypoints, cv::noArray());

/// Check that keypoints is not empty
  if(this->keypoints.empty()){ROS_ERROR("No keypoints found!!"); return 1;}
  
  return 0;
}
/*******************************************************************/


/******************************************************************/
/// Computes descriptors for ORB keypoints
/// If unable to compute descriptors then returns 1
bool FeatureDetector::computeDescriptors(){
/// Compute descriptors from provided image and corresponding keypoints
  this->orb->compute(this->image, this->keypoints, this->descriptors);

/// Check if descriptors are computed successfully
  if(this->descriptors.empty()){ROS_WARN("Not able to compute descriptors!!"); return 1;}
  
  return 0;
}
/******************************************************************/


/******************************************************************/
void FeatureDetector::displayKeypoints(){
/// Draw keypoints on image
  #ifdef DEBUG_MODE
    ready = false;
    std::unique_lock<std::mutex> lck1(mtx1);
  #endif

    auto start = std::chrono::high_resolution_clock::now();

    cv::drawKeypoints(this->image, this->keypoints, this->keypointImage,
                      cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);

    /// Create window and put the image with keypoints on window
      cv::namedWindow("Detected Kepoints", cv::WINDOW_NORMAL);
      cv::resizeWindow("Detected Kepoints", 1920, 1080);
      cv::imshow("Detected Kepoints", this->keypointImage);
    
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    auto frame_rate = 1000/(float)duration.count();
    ROS_INFO_STREAM("Keypoints Display Frame rate = " << std::to_string(frame_rate) << std::endl);

  #ifdef DEBUG_MODE
    lck1.unlock();
  #endif

  cv::waitKey(30);

  #ifdef DEBUG_MODE
    ready = true;
  #endif

  return;
}
/******************************************************************/


/******************************************************************/
void FeatureDetector::imageCompute(){
  #ifdef DEBUG_MODE
    std::unique_lock<std::mutex> lck1(mtx1);
  #endif

  if(!this->detectFeatures()){
    /// Raise exception
  }
  
  if(!this->computeDescriptors()){
    /// Raise exception
  }

  #ifdef DEBUG_MODE
    lck1.unlock();
  #endif

  #ifdef DEBUG_MODE
    if(ready){
      std::thread(&FeatureDetector::displayKeypoints, this).detach();
    }
  #endif

  return;
}
/******************************************************************/