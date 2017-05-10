% Add additional inputs after sensor if you want to
% Example:
% your_input = 1;
% estimate_vel_handle = @(sensor) estimate_vel(sensor, your_input);
%
% We will only call estimate_vel_handle in the test function.
% Note that thise will only create a function handle, but not run the function

% camera calibration matrix
K = [311.0520 0 201.8724; 0 311.3885 113.6210; 0 0 1];

% Camera-robot transformation robot expressed in camera  yaw pi + pitch pi/4 
H_cr = [sqrt(2)/2   -sqrt(2)/2         0   -0.04;...
       -sqrt(2)/2   -sqrt(2)/2         0       0;...
             0         0              -1   -0.03;...
             0         0               0       1]; 

load('corners.mat');

estimate_vel_handle = @(sensor) estimate_vel(sensor,K,H_cr,P);
