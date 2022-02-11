#ifndef IMAGE_SUBSCRIBER_H
#define IMAGE_SUBSCRIBER_H


#include <opencv2/core/core.hpp>
#include <image_transport/image_transport.h>
class RosToCvmat
{
private:
    void imageCallback(const sensor_msgs::ImageConstPtr&);
protected:
    cv::Mat image;
public:
    void imageSubscriber(bool displayOn, int &argc, char** &argv);
    virtual void imageCompute();
};

#endif