% Add additional inputs after the given ones if you want to
% Example:
% your_input = 1;
% ekf_handle1 = @(sensor, vic) eskf1(sensor, vic, your_input);
% ekf_handle2 = @(sensor) eskf2(sensor, your_input);
%
% We will only call ekf_handle in the test function.
% Note that this will only create a function handle, but not run the function

% camera calibration matrix
K = [311.0520 0 201.8724; 0 311.3885 113.6210; 0 0 1];

% Camera-robot transformation robot expressed in camera  yaw pi + pitch pi/4 
H_cr = [sqrt(2)/2   -sqrt(2)/2         0   -0.04;...
       -sqrt(2)/2   -sqrt(2)/2         0       0;...
             0         0              -1   -0.03;...
             0         0               0       1]; 

load('corners.mat');

ekf1_handle = @(sensor, vic) ekf1(sensor, vic, K,H_cr,P);
ekf2_handle = @(sensor) ekf2(sensor, K,H_cr,P);

% eskf1_handle = @(sensor, vic) eskf1(sensor, vic);
% eskf2_handle = @(sensor) eskf2(sensor);
