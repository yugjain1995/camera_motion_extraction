# motion_extraction
Estimate motion

Use the TEST_IMAGE_SUB=True to build for image subscription (image_subscriber.cpp) test

catkin build -DTEST_IMAGE_SUB=True

Use the TEST_KEYPOINT_DETECTION=True to build for ORB keypoint detection (feature_detection.cpp) test

catkin build -DTEST_KEYPOINT_DETECTION=True

Use "-d" argument to display image from image subscription
Example - "rosrun  motion_extraction camera_motion_extraction -d"
