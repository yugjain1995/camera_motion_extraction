#ifndef FEATURE_DETECTION_H
#define FEATURE_DETECTION_H


/// header
#include <opencv2/features2d.hpp>
#include<image_subscriber.h>
/// header

class FeatureDetector: public RosToCvmat {
  private:
    cv::Ptr<cv::ORB> orb;

  protected:
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    cv::Mat keypointImage;
    bool detectFeatures();
    void displayKeypoints();
    bool computeDescriptors();

  public:
    FeatureDetector();
    virtual void imageCompute();
};


#endif