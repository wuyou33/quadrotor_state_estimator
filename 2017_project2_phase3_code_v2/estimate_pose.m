function [pos, q, H, H_wr] = estimate_pose(sensor, varargin)
%ESTIMATE_POSE 6DOF pose estimator based on apriltags
%   sensor - struct stored in provided dataset, fields include
%          - is_ready: logical, indicates whether sensor data is valid
%          - rpy, omg, acc: imu readings, you should not use these in this phase
%          - img: uint8, 240x376 grayscale image
%          - id: 1xn ids of detected tags
%          - p0, p1, p2, p3, p4: 2xn pixel position of center and
%                                four corners of detected tags
%            Y
%            ^ P3 == P2
%            | || P0 ||
%            | P4 == P1
%            o---------> X

%   varargin - any variables you wish to pass into the function, could be
%              a data structure to represent the map or camera parameters,
%              your decision. But for the purpose of testing, since we don't
%              know what inputs you will use, you have to specify them in
%              init_script by doing
%              estimate_pose_handle = ...
%                  @(sensor) estimate_pose(sensor, your personal input arguments);

%   pos - 3x1 position of the quadrotor in world frame
%   q   - 4x1 quaternion of the quadrotor [w, x, y, z] where q = w + x*i + y*j + z*k

% camera calibration matrix
K = varargin{1};
% Camera-robot transformation robot expressed in camera
H_cr = varargin{2};
% load corners world coordinates
P = varargin{3}; %10*108 for p0p1p2p3p4

% load data from sensors
id = sensor.id+1;
p = [sensor.p0 sensor.p1 sensor.p2 sensor.p3 sensor.p4];

if size(id,2) == 0 % return empty for empty packets
    pos =[];
    q =[];
else
% compute homography
u = (p(1,:))';
v = (p(2,:))';
X = [P(1,id) P(3,id) P(5,id) P(7,id) P(9,id)]';
Y = [P(2,id) P(4,id) P(6,id) P(8,id) P(10,id)]';
H = CalculateHomography(u,v,X,Y);

% estimate r1 r2 t world expressed in camera
r1 = K\H(:,1) / norm(K\H(:,1));
r2 = K\H(:,2) / norm(K\H(:,2)); 
t_cw = K\H(:,3) / norm(K\H(:,2)); % norm h2 better
R_cw = [r1 r2 cross(r1,r2)];
H = [r1 r2 t_cw];

% transformation to robot in world frame
% camera expressed in world * robot expressed in camera Hwr=Hwc*Hcr
H_wr = [(R_cw)' -(R_cw)'*t_cw; 0 0 0 1]*H_cr; 
pos = H_wr(1:3,end);
q = R2q(H_wr(1:3,1:3));
end
end
