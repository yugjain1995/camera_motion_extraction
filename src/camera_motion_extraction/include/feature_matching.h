#ifndef FEATURE_MATCHING_H
#define FEATURE_MATCHING_H


/// header
#include <opencv2/features2d.hpp>
#include<feature_detection.h>
/// header


class FeatureMatcher: public FeatureDetector {
  private:
    cv::Ptr<cv::BFMatcher> bruteForceMatcher;
    std::vector<cv::DMatch> matches;
    cv::Mat matchedImage;
    #ifdef DEBUG_MODE
      bool ready1;
      std::mutex mtx2;
    #endif
    
  protected:
    cv::Mat preImage; // Previously recieved image
    std::vector<cv::KeyPoint> keypoints1; // Keypoints calculated for preImage
    cv::Mat descriptors1; // Descriptors calculated for preImage
    void match();
    void displayMatch();

  public:
    FeatureMatcher();
    void makePrevious();
    void imageCompute();
};

#endif