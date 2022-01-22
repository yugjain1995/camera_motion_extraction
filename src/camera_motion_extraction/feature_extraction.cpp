#include <iostream>
#include <cstdlib>

/// [headers]
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d.hpp>
/// [headers]

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
/// Convert ros image to cv::Mat
  try
  {
    cv::Mat image = cv_bridge::toCvShare(msg, "mono8")->image;
    ROS_INFO("Image recieved\n");
/// Create ORB object
    static cv::Ptr<cv::ORB> orb = cv::ORB::create(10, 1.1f, 16, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, 20);

/// Get feature points
    std::vector<cv::KeyPoint> keypoints;
    orb->detect(image, keypoints, cv::noArray());

/// Draw keypoints on image
    cv::Mat keypointImage;
    cv::drawKeypoints(image, keypoints, keypointImage,
                    cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);

/// Put the image with keypoints on view
    cv::imshow("view", keypointImage);
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'mono8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  cv::namedWindow("view");
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber image_sub = it.subscribe("camera/image", 1, imageCallback);
  ros::spin();
  cv::destroyWindow("view");
}