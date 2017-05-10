# MEAM620_Project2

In phase 1, I implemented a vision based 3D pose estimator that estimates position and orientation of the quadrotor based on AprilTags.

In phase 2, I extracted matching features between consecutive images, conducted RANSAC to remove outliers, then compute sparse optical flow to estimate 3D velocity of the quadrotor.

In phase 3, I applied extended kalman filters for state estimation. I fuse VICON and Vision in EKF1 and fuse IMU and Vision in EKF2.

All test results suggest my vision-based estimator is very accurate compared to ground truth VICON data.
