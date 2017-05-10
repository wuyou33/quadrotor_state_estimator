function [X, Z] = ekf1(sensor, vic, varargin)
% EKF1 Extended Kalman Filter with Vicon velocity as inputs
%
% INPUTS:
%   sensor - struct stored in provided dataset, fields include
%          - is_ready: logical, indicates whether sensor data is valid
%          - t: sensor timestamp
%          - rpy, omg, acc: imu readings
%          - img: uint8, 240x376 grayscale image
%          - id: 1xn ids of detected tags
%          - p0, p1, p2, p3, p4: 2xn pixel position of center and
%                                four corners of detected tags
%            Y
%            ^ P3 == P2
%            | || P0 ||
%            | P4 == P1
%            o---------> X
%   vic    - struct for storing vicon linear velocity in world frame and
%            angular velocity in body frame, fields include
%          - t: vicon timestamp
%          - vel = [vx; vy; vz; wx; wy; wz]
%   varargin - any variables you wish to pass into the function, could be
%              a data structure to represent the map or camera parameters,
%              your decision. But for the purpose of testing, since we don't
%              know what inputs you will use, you have to specify them in
%              init_script by doing
%              ekf1_handle = ...
%                  @(sensor, vic) ekf1(sensor, vic, your input arguments);
%
% OUTPUTS:
% X - nx1 state of the quadrotor, n should be greater or equal to 6
%     the state should be in the following order
%     [x; y; z; qw; qx; qy; qz; other states you use]
%     we will only take the first 7 rows of X
% OPTIONAL OUTPUTS:
% Z - mx1 measurement of your pose estimator, m shoulb be greater or equal to 7
%     the measurement should be in the following order
%     [x; y; z; qw; qx; qy; qz; other measurement you use]
%     note that this output is optional, it's here in case you want to log your
%     measurement

persistent ur up cr cp t; % state real&predicted, corivance real&predicted, time
bg = 0*[1;1;1]; % gryo bias
Nv = 0*[1;1;1]; 
Ng = 0*[1;1;1];
% til now best 1:0.02 1:0.015 good 1:1 1:0.1 1:0.01 bad 1:100 1:10
Q = 1; % VICON noise
R = 0.015; % VISION noise 

% prior / initialization
if isempty(ur)
    if ~isempty(sensor.id)
        % load data
        K = varargin{1};
        H_cr = varargin{2};
        P = varargin{3}; 
        [p, q, ~, ~] = estimate_pose(sensor, K,H_cr,P);
        X = [p;q];
        Z = [p;q];
        [yaw,roll,pitch] = quat2angle(q', 'ZXY');
        ur = [p;roll;pitch;yaw]; 
        cr = 0; % ???
        t = sensor.t;
    else
        X = zeros(7,1);
        Z = zeros(7,1);
        ur = zeros(6,1); 
        cr = 0; % ???
        t = vic.t;
    end
else

% process model VJ note (30)
G = [cos(ur(5)) 0 -cos(ur(4))*sin(ur(5)); 0 1 sin(ur(4)); sin(ur(5)) 0 cos(ur(4))*cos(ur(5))]; 
f = [vic.vel(1:3)-Nv;...
     G\(vic.vel(4:6)-bg-Ng)]; %  6*1
 
% measurement model
C = [eye(3) zeros(3,3);zeros(3,3) eye(3)];
% Z = C*ur

% prediction
t_step = vic.t-t;
t = vic.t;
up = ur+t_step*f;

A = findjacobian1(vic.vel,Nv,Ng,bg,ur); %6*6 
F = eye(6) + t_step*A;
U = eye(6); % ???
V = U*t_step; 
cp = F*cr*F'+ V*Q*V';

if ~isempty(sensor.id) % update
    K = varargin{1};
    H_cr = varargin{2};
    P = varargin{3}; 
    [p, q, ~, ~] = estimate_pose(sensor, K,H_cr,P);
    Z = [p;q];
    [yaw,roll,pitch] = quat2angle(q', 'ZXY');
    z = [p;roll;pitch;yaw]; 
    W = eye(6); % ???
    K = cp*C'/(C*cp*C'+W*R*W'); % cp*C'*inv(C*cp*C'+W*R*W');
    ur = up + K*(z-C*up);
    cr = cp - K*C*cp;
else
    ur = up;
    cr = cp;
    Z = zeros(7,1);
end
quat = angle2quat(ur(6), ur(4), ur(5), 'ZXY');
X = [ur(1:3);quat'];

end

end
