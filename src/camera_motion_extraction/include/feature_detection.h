#ifndef FEATURE_DETECTION_H
#define FEATURE_DETECTION_H


/// header
#include <opencv2/features2d.hpp>
#include<image_subscriber.h>
/// header

class FeatureDetector: public RosToCvmat {
  private:
    cv::Ptr<cv::ORB> orb;
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat keypointImage;
    void detectFeatures();
    void displayKeypoints();
  public:
    FeatureDetector();
    void imageCompute();
};


#endif