#ifndef MOTION_EXTRACTION_H
#define MOTION_EXTRACTION_H


/// header
#include <opencv2/features2d.hpp>
#include<feature_matching.h>
/// header


class MotionEstimate2D2D: public FeatureMatcher {
  private:
    cv::Mat fundamental_matrix;
    cv::Mat essential_matrix;
    cv::Mat K; // Matrix for camera intrinsics
    
  public:
    cv::Mat R, t;
    MotionEstimate2D2D();
    void findEssentialMat();
    void cameraPoseEstimate();
    void recoverPose();
    void imageCompute();
};

#endif