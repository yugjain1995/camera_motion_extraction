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
  #ifdef DEBUG_MODE
    ready1 = true;
  #endif
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
        if(ready1){
          std::thread(&FeatureMatcher::displayMatch, this).detach();
        }
      #endif
      
    /// Make recently recieved image previous
    #ifdef DEBUG_MODE
      std::unique_lock<std::mutex> lck2(mtx2);
    #endif
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


/******************************************************************/
void FeatureMatcher::displayMatch(){
/// Draw keypoints on image
  #ifdef DEBUG_MODE
    ready1 = false;
    std::unique_lock<std::mutex> lck2(mtx2);
  #endif

    auto start = std::chrono::high_resolution_clock::now();

    cv::drawKeypoints(this->image, this->keypoints, this->keypointImage,
                      cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);

    /// Create window and put the image with keypoints on window
      cv::namedWindow("Keypoint matching", cv::WINDOW_NORMAL);
      cv::resizeWindow("Keypoint matching", 1920, 1080);
      cv::imshow("Keypoint matching", this->matchedImage);
    
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    auto frame_rate = 1000/(float)duration.count();
    ROS_INFO_STREAM("Matching Display Frame rate = " << std::to_string(frame_rate) << std::endl );

  #ifdef DEBUG_MODE
    lck2.unlock();
  #endif

  cv::waitKey(30);

  #ifdef DEBUG_MODE
    ready1 = true;
  #endif

  return;
}
/******************************************************************/