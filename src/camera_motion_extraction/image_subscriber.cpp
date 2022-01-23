#include <iostream>
#include <cstdlib>

/// [headers]
/// #include <ros/ros.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include<image_subscriber.h>
/// [headers]

// Subscribe to image but, do not display
void RosToCvmat::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    image = cv_bridge::toCvShare(msg, "mono8")->image;
    ROS_INFO("Image recieved");
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'mono8'.", msg->encoding.c_str());
  }
}

// Subscribe to image and display
void RosToCvmat::imageCallbackWithDisplay(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    image = cv_bridge::toCvShare(msg, "mono8")->image;
    cv::imshow("view", image);
    cv::waitKey(30);
    ROS_INFO("Image recieved");
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'mono8'.", msg->encoding.c_str());
  }
}

// Initialization
void RosToCvmat::imageSubscriber(int &argc, char** &argv){
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber image_sub = it.subscribe("camera/image", 1, &RosToCvmat::imageCallback, this);
  ros::spin();
}

// Subscribe and display
void RosToCvmat::imageSubscriber(bool displayOn, int &argc, char** &argv){
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  if(displayOn == true){
    cv::namedWindow("view");
  }
  else{
    RosToCvmat::imageSubscriber(argc, argv);
  }
  image_transport::Subscriber image_sub = it.subscribe("camera/image", 1, &RosToCvmat::imageCallbackWithDisplay, this);
  if(displayOn == true){
    ros::spin();
  }
  if(displayOn == true){
    cv::destroyWindow("view");
  }
}