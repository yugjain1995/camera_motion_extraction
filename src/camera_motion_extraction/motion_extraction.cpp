/************************************************************************************
Author - Yug Jain
***********************************************************************************/

/************************************************************************************
This code provide function definitions for MotionEstimate2D2D class which can be used to
compute change of pose of camera given keypoint matches of the consecutive images from
a monocular camera.
************************************************************************************/

/// [headers]
#include <ros/console.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/calib3d/calib3d.hpp"
#include <motion_extraction.h>
#include <chrono>
/// [headers]


/******************************************************************/
MotionEstimate2D2D::MotionEstimate2D2D(){
// Initialize camera intrinsics
  float fx = 1144.361;
  float fy = 1147.337;
  float cx = 966.359;
  float cy = 548.038;
  this->K = (Mat_<double>(3,3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
}
/******************************************************************/

/******************************************************************/
MotionEstimate2D2D::imageCompute(){
  this->matchAndDisplay(); // Compute keypoints and keypoint matches
  this->cameraPoseEstimate();
}
/******************************************************************/

/******************************************************************/
MotionEstimate2D2D::cameraPoseEstimate(){
// Start the timer
  auto start = std::chrono::high_resolution_clock::now();
// Get essential matrix from keypoints
  findEssentialMat();
// Recover pose from essential matrix
  recoverPose();
// End the timer and compute time elapsed
  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
// Display Pose
  ROS_INFO_STREAM( "\nR = \n" << _R << "\n" );
  ROS_INFO_STREAM( "\nt = \n" << _t << "\n" );
// Display time elapsed
  ROS_INFO_STREAM( "Total time to estimate pose = " << std::to_string(duration.count()) << std::endl );
}
/******************************************************************/

/******************************************************************/
void MotionEstimate2D2D::findEssentialMat(){
  std::vector<cv::Point2f> p1(matches.size()), p2(matches.size());

// Compute fundamental matrix
  fundamental_matrix = cv::findFundamentalMat(p1, p2, cv::FM_RANSAC, 2, 0.95);

// From camera intrinsics and funtamental matrix compute essential matrix
  essential_matrix = K.t()*fundamental_matrix*K;
}
/******************************************************************/

/******************************************************************/
void MotionEstimate2D2D::recoverPose(){
// Map points from keypoints to points object according to index in
// matches
  for(std::vector<cv::DMatch>::iterator it = matches.begin();
      it != matches.end(); ++it){
        projPoints1.push_back(keypoints[it->queryIdx].pt);
        projPoints2.push_back(keypoints1[it->trainIdx].pt);
  }

// Decompose essential matrix to 2 rotation matrices and translation matrix
  cv::Mat R1, R2, t;
  decomposeEssentialMat(essential_matrix, R1, R2, t);

// Triangulate all the points to get the chirality of points w.r.t. to
// each of the four combinations of camera rotation and translation
  
  cv::Mat points4D; // Reconstructed points in homogeneous co-ordinates
  double dist = 50.0; // Ignore far away points

  // Create projMatr1
    cv::Mat projMatr1 = cv::Mat::eye(3, 4, R1.type());

  cv::Mat projMatr2;

  // Find chirality for R1 and t
    // Create projMatr2 for R1 and t
      projMatr2(cv::Range::all(), cv::Range(0, 3)) = R1 * 1.0; projMatr2.col(3) = t * 1.0;
    cv::triangulatePoints(projMatr1, projMatr2, projPoints1, projPoints2, points4D);
    cv::Mat mask = points4D.row(2).mul(points4D.row(3)) > 0;
    points4D.row(0) /= points4D.row(3);
    points4D.row(1) /= points4D.row(3);
    points4D.row(2) /= points4D.row(3);
    points4D.row(3) /= points4D.row(3);
    mask = (points4D.row(2) < dist) & mask;
    points4D = projMatr1 * points4D;
    mask = (points4D.row(2) > 0) & mask;
    mask = (points4D.row(2) < dist) & mask;
    int cnt1 = countNonZero(mask);

  // Find chirality for R2 and t
    // Create projMatr2 for R2 and t
      projMatr2(cv::Range::all(), cv::Range(0, 3)) = R2 * 1.0; projMatr2.col(3) = t * 1.0;
    cv::triangulatePoints(projMatr1, projMatr2, projPoints1, projPoints2, points4D);
    cv::triangulatePoints(projMatr1, projMatr2, projPoints1, projPoints2, points4D);
    mask = points4D.row(2).mul(points4D.row(3)) > 0;
    points4D.row(0) /= points4D.row(3);
    points4D.row(1) /= points4D.row(3);
    points4D.row(2) /= points4D.row(3);
    points4D.row(3) /= points4D.row(3);
    mask = (points4D.row(2) < dist) & mask;
    points4D = projMatr1 * points4D;
    mask = (points4D.row(2) > 0) & mask;
    mask = (points4D.row(2) < dist) & mask;
    int cnt2 = countNonZero(mask);

  // Find chirality for R1 and -t
    // Create projMatr2 for R1 and -t
      projMatr2(cv::Range::all(), cv::Range(0, 3)) = R1 * 1.0; projMatr2.col(3) = -t * 1.0;
    cv::triangulatePoints(projMatr1, projMatr2, projPoints1, projPoints2, points4D);
    cv::triangulatePoints(projMatr1, projMatr2, projPoints1, projPoints2, points4D);
    mask = points4D.row(2).mul(points4D.row(3)) > 0;
    points4D.row(0) /= points4D.row(3);
    points4D.row(1) /= points4D.row(3);
    points4D.row(2) /= points4D.row(3);
    points4D.row(3) /= points4D.row(3);
    mask = (points4D.row(2) < dist) & mask;
    points4D = projMatr1 * points4D;
    mask = (points4D.row(2) > 0) & mask;
    mask = (points4D.row(2) < dist) & mask;
    int cnt3 = countNonZero(mask);

  // Find chirality for R2 and -t
    // Create projMatr2 for R2 and -t
      projMatr2(cv::Range::all(), cv::Range(0, 3)) = R2 * 1.0; projMatr2.col(3) = -t * 1.0;
    cv::triangulatePoints(projMatr1, projMatr2, projPoints1, projPoints2, points4D);
    cv::triangulatePoints(projMatr1, projMatr2, projPoints1, projPoints2, points4D);
    mask = points4D.row(2).mul(points4D.row(3)) > 0;
    points4D.row(0) /= points4D.row(3);
    points4D.row(1) /= points4D.row(3);
    points4D.row(2) /= points4D.row(3);
    points4D.row(3) /= points4D.row(3);
    mask = (points4D.row(2) < dist) & mask;
    points4D = projMatr1 * points4D;
    mask = (points4D.row(2) > 0) & mask;
    mask = (points4D.row(2) < dist) & mask;
    int cnt4 = countNonZero(mask);

// Select the the camera pose with highest number of points
// with positive chirality
  if(cnt1 >= cnt2 && cnt1 >= cnt3 && cnt1 >= cnt4){
    R1.copyTo(_R);
    t.copyTo(_t);
  }
  else if(cnt2 >= cnt1 && cnt2 >= cnt3 && cnt2 >= cnt4){
    R2.copyTo(_R);
    t.copyTo(_t);      
  }
  else if(cnt3 >= cnt1 && cnt3 >= cnt2 && cnt3 >= cnt4){
      R1.copyTo(_R);
      t = -t;
      t.copyTo(_t);    
  }
  else{
      R2.copyTo(_R);
      t = -t;
      t.copyTo(_t);    
  }
// Clear keypoints co-ordinate vector
  projPoints1.clear();
  projPoints2.clear();
}
/******************************************************************/

/******************************************************************/
void MotionEstimate2D2D::decomposeEssentialMat(cv::InputArray _E, cv::OutputArray _R1, 
                                                cv::OutputArray _R2, cv::OutputArray _t)
{
    cv::Mat E = _E.getMat().reshape(1, 3);
    CV_Assert(E.cols == 3 && E.rows == 3);

    cv::Mat D, U, Vt;
    cv::SVD::compute(E, D, U, Vt);

    if (determinant(U) < 0) U *= -1.;
    if (determinant(Vt) < 0) Vt *= -1.;

    cv::Mat W = (cv::Mat_<double>(3, 3) << 0, 1, 0, -1, 0, 0, 0, 0, 1);
    W.convertTo(W, E.type());

    cv::Mat R1, R2, t;
    R1 = U * W * Vt;
    R2 = U * W.t() * Vt;
    t = U.col(2) * 1.0;

    R1.copyTo(_R1);
    R2.copyTo(_R2);
    t.copyTo(_t);
}
/******************************************************************/