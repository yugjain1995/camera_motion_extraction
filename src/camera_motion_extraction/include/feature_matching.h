#ifndef FEATURE_MATCHING_H
#define FEATURE_MATCHING_H


/// header
#include <opencv2/features2d.hpp>
#include<image_subscriber.h>
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
    void match(string); // If you want to display the keypoint matching result then create
                        // a cv image window and pass image window name as string to this fuction.
                        // By default it does not display any image 

  public:
    FeatureMatcher();
    void imageCompute();
};


#endif