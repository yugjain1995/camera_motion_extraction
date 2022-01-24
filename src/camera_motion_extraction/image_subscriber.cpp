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

void RosToCvmat::imageSubscriber(bool displayOn, int &argc, char** &argv){
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber image_sub = it.subscribe("camera/image", 1, &RosToCvmat::imageCallback, this);
  if(displayOn){
    cv::namedWindow("view");
    while(nh.ok()){
      ros::spinOnce();
      if(!image.empty()){
        cv::imshow("view", image);
        cv::waitKey(30);
      }
    }
  }
  else{
    ros::spin();
  }
  if(displayOn){
    cv::destroyWindow("view");
  }
}