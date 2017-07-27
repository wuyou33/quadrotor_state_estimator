# MEAM620_Project2

In phase 1, I implemented a vision based 3D pose estimator that estimates position and orientation of the quadrotor based on AprilTags. Position error 0.015-0.025; Quaternion error 0.006-0.009; 0.4ms per iteration 

In phase 2, I extracted matching features between consecutive images, conducted RANSAC to remove outliers, then compute sparse optical flow to estimate translational and rotational velocity of the quadrotor. Velocity error 0.060-0.075; Omega error 0.13-0.16; 4.8-6.0ms per iteration

In phase 3, I applied extended kalman filters for state estimation. I fuse VICON and Vision in EKF1 and fuse IMU and Vision in EKF2.
