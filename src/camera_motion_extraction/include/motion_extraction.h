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
    std::vector<cv::Point2f> projPoints1; // Co-ordinates of keypoints querry set
    std::vector<cv::Point2f> projPoints2; // Co-ordinates of keypoints training set
    cv::Mat _R, _t;
    void findEssentialMat();
    void cameraPoseEstimate();
    void recoverPose();
    void decomposeEssentialMat(cv::InputArray, cv::OutputArray, cv::OutputArray, cv::OutputArray);
    
  public:
    MotionEstimate2D2D();
    void imageCompute();

};

#endif