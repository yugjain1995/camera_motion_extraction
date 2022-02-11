#ifndef FEATURE_MATCHING_H
#define FEATURE_MATCHING_H


/// header
#include <opencv2/features2d.hpp>
#include<image_subscriber.h>
/// header

class FeatureMatcher: public FeatureDetector {
  public:
    FeatureMatcher();
    void imageCompute();
};


#endif