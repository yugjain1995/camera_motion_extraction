#ifndef IMAGE_SUBSCRIBER_H
#define IMAGE_SUBSCRIBER_H


#include <opencv2/core/core.hpp>
#include <image_transport/image_transport.h>
class RosToCvmat
{
private:
    cv::Mat image;
    void imageCallback(const sensor_msgs::ImageConstPtr&);

public:
    void imageSubscriber(bool displayOn, int &argc, char** &argv);
};

#endif