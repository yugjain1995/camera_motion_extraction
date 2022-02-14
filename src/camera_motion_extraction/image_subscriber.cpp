/// [headers]
#include <ros/ros.h>
#include <ros/console.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include<image_subscriber.h>
#include <chrono>
/// [headers]

// Subscribe to image but, do not display
void RosToCvmat::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    image = cv_bridge::toCvShare(msg, "mono8")->image.clone();
    ROS_INFO("Image recieved");
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'mono8'.", msg->encoding.c_str());
  }
}

void RosToCvmat::imageCompute(){
// This function intentionally does nothing here
// This function is used to as a override to do computions by any
// class that inherits this class to get image data
  return;
}

void RosToCvmat::imageSubscriber(bool displayOn, int &argc, char** &argv){
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber image_sub = it.subscribe("camera/image", 1, &RosToCvmat::imageCallback, this);
  if(displayOn){
    cv::namedWindow("view");
    while(nh.ok()){
      auto start = std::chrono::high_resolution_clock::now();

      ros::spinOnce();
      if(!image.empty()){
        cv::imshow("view", image);
        cv::waitKey(30);
        imageCompute();
      }

      auto stop = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
      auto frame_rate = 1000/(float)duration.count();
      ROS_INFO_STREAM( "Compute Frame rate = " << std::to_string(frame_rate) << std::endl );
    }
  }
  else{
    while(nh.ok()){
      auto start = std::chrono::high_resolution_clock::now();

      ros::spinOnce();
      if(!image.empty()){
        imageCompute();
      }

      auto stop = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
      auto frame_rate = 1000/(float)duration.count();
      ROS_INFO_STREAM( "Compute Frame rate = " << std::to_string(frame_rate) << std::endl );
    }
  }
  if(displayOn){
    cv::destroyWindow("view");
  }
}