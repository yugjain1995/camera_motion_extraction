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
    
  protected:
    cv::Mat image1;
    std::vector<cv::KeyPoint> keypoints1;
    cv::Mat descriptors1;
    void match();

  public:
    FeatureMatcher();
    void imageCompute();
};


#endif