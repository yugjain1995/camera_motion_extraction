# motion_extraction
Estimate motion

Use the TEST_IMAGE_SUB=True to build for image subscription (image_subscriber.cpp) test

catkin build -DTEST_IMAGE_SUB=True


Use the TEST_KEYPOINT_DETECTION=True to build for ORB keypoint detection (feature_detection.cpp) test

catkin build -DTEST_KEYPOINT_DETECTION=True


Use the TEST_FEATURE_MATCHING=True to build for matching keypoints of two images (feature_matching.cpp) test

catkin build -DTEST_FEATURE_MATCHING=True


Use "-DDEBUG_MODE=True" argument while catkin build to display optput images from each of the above step

Example - catkin build -DTEST_FEATURE_MATCHING=True -DDEBUG_MODE=True

This build for featur matching and display it's output
 

To run the camera_motion_extraction module of the package

rosrun  motion_extraction camera_motion_extraction
