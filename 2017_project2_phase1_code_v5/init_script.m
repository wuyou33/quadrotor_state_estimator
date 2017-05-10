% add additional inputs after sensor if you want to
% Example:
% your_input = 1;
% estimate_pose_handle = @(sensor) estimate_pose(sensor, your_input);
% We will only call estimate_pose_handle in the test function.
% Note that unlike project 1 phase 3, thise will only create a function
% handle, but not run the function at all.

% camera calibration matrix
K = [311.0520 0 201.8724; 0 311.3885 113.6210; 0 0 1];
% [314.1779 0         199.4848; ...
% 0         314.2218  113.7838; ...
% 0         0         1];
% Camera-robot transformation robot expressed in camera  yaw pi + pitch pi/4 
H_cr = [0.7071   -0.7071         0   -0.04;...
       -0.7071   -0.7071         0       0;...
             0         0   -1.0000   -0.03;...
             0         0         0       1]; 

load('corners.mat');

estimate_pose_handle = @(sensor) estimate_pose(sensor,K,H_cr,P);
