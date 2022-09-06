#ifndef IMAGE_SUBSCRIBER_H
#define IMAGE_SUBSCRIBER_H


/// header
#include <opencv2/core/core.hpp>
#include <image_transport/image_transport.h>
#include <thread>
#include <future>
#include <mutex>
/// header


class RosToCvmat
{
private:
    void imageCallback(const sensor_msgs::ImageConstPtr&);
    #ifdef DEBUG_MODE
      std::mutex mtx;
    #endif

protected:
    cv::Mat image;
    
public:
    void imageSubscriber(int &argc, char** &argv);
    void displayRecievedImage();
    virtual void imageCompute();
};

#endif