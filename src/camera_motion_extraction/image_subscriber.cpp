/************************************************************************************
Author - Yug Jain
***********************************************************************************/

/************************************************************************************
This code provides function definition for RosToCvmat class funcions which can be used to 
recieve image as a ROS message. After receving the image message this code converts 
the image form ROS message format to cv::Mat format
which can be used for further computations.
************************************************************************************/


/// [headers]
#include <ros/ros.h>
#include <ros/console.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include<image_subscriber.h>
#include <chrono>
/// [headers]


/******************************************************************/
/// Get image -> convert to cv::Mat -> and clone to a seperate cv::Mat
/// for storage and further computation 
void RosToCvmat::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    std::unique_lock<std::mutex> lck (mtx);
    image = cv_bridge::toCvShare(msg, "mono8")->image.clone();
    ROS_INFO("Image recieved");
    imageCompute();
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'mono8'.", msg->encoding.c_str());
  }
}
/******************************************************************/


/******************************************************************/
void RosToCvmat::imageCompute(){
/// This function intentionally does nothing here
/// This function is used to as a override to do computions by any
/// class that inherits this class to get image data
  return;
}
/******************************************************************/


/******************************************************************/
void RosToCvmat::imageSubscriber(int &argc, char** &argv){
/// Inintialize node and set-up node as image subscriber
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber image_sub = it.subscribe("camera/image", 1, &RosToCvmat::imageCallback, this);

/// Generate a window to display recieved image if DEBUG_MODE is set  
  #ifdef DEBUG_MODE
    cv::namedWindow("Received image", cv::WINDOW_NORMAL);
    cv::resizeWindow("Received image", 1920, 1080);
  #endif

/// Listen to image topic and perform computations as necessary

  #ifdef DEBUG_MODE
    std::future<void> imgDispFut = std::async(std::launch::async, &RosToCvmat::displayRecievedImage, this); // Create a future obj
  #endif
  
  while(nh.ok()){
    auto start = std::chrono::high_resolution_clock::now();

    ros::spinOnce();

    #ifdef DEBUG_MODE
      // If thread completed
      if(imgDispFut.wait_for(std::chrono::seconds(0)) == std::future_status::ready){
          // Rerun the thread
          imgDispFut = std::async(std::launch::async, &RosToCvmat::displayRecievedImage, this);
      }
    #endif

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    auto frame_rate = 1000/(float)duration.count();
    ROS_INFO_STREAM( "Compute Frame rate = " << std::to_string(frame_rate) << std::endl );
  }

/// Display image recieved if DEBUG_MODE is set
  #ifdef DEBUG_MODE
    cv::destroyWindow("Received image");
  #endif
}
/******************************************************************/


/******************************************************************/
void RosToCvmat::displayRecievedImage(){
  if(!image.empty()){
    std::unique_lock<std::mutex> lck (mtx);
    cv::imshow("Received image", image);
    lck.unlock();
    cv::waitKey(30);
  }
}
/******************************************************************/