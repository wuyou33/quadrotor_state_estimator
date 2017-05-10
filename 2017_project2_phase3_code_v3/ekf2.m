function [X, Z] = ekf2(sensor, varargin)
% EKF2 Extended Kalman Filter with IMU as inputs
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
%   varargin - any variables you wish to pass into the function, could be
%              a data structure to represent the map or camera parameters,
%              your decision. But for the purpose of testing, since we don't
%              know what inputs you will use, you have to specify them in
%              init_script by doing
%              ekf1_handle = ...
%                  @(sensor) ekf2(sensor, your input arguments);
%
% OUTPUTS:
% X - nx1 state of the quadrotor, n should be greater or equal to 9
%     the state should be in the following order
%     [x; y; z; vx; vy; vz; qw; qx; qy; qz; other states you use]
%     we will only take the first 10 rows of X
% OPTIONAL OUTPUTS:
% Z - mx1 measurement of your pose estimator, m shoulb be greater or equal to 6
%     the measurement should be in the following order
%     [x; y; z; qw; qx; qy; qz; other measurement you use]
%     note that this output is optional, it's here in case you want to log your
%     measurement

persistent ur up cr cp t; % state real&predicted, corivance real&predicted, time
bg = 0*[1;1;1]; % gryo bias
ba = 0*[1;1;1]; % accel bias
Na = 0*[1;1;1]; 
Ng = 0*[1;1;1];
g = [0;0;-9.81]; % gravity
% til now best  good 1:1 1:10 bad 1:100 1:0.01
Q = 1; % IMU noise
R = 10; % VISION noise

% prior / initialization
if isempty(ur)
    if ~isempty(sensor.id)
        % load data
        K = varargin{1};
        H_cr = varargin{2};
        P = varargin{3}; 
        [p,q,v,~] = estimate_vel(sensor, K,H_cr,P); % may use estimate_vel here if fast enough
        X = [p;v;q];
        Z = [p;q;v];
        [yaw,roll,pitch] = quat2angle(q', 'ZXY');
        ur = [p;roll;pitch;yaw;v]; 
        cr = 0; % ???
        t = sensor.t;
    else
        X = zeros(10,1);
        Z = zeros(10,1);
%         ur = zeros(9,1); 
%         cr = 0; % ???
%         t = sensor.t;
    end
else

% process model VJ note (30)
G = [cos(ur(5)) 0 -cos(ur(4))*sin(ur(5)); 0 1 sin(ur(4)); sin(ur(5)) 0 cos(ur(4))*cos(ur(5))]; 
Rq = [cos(ur(6))*cos(ur(5))-sin(ur(4))*sin(ur(5))*sin(ur(6)) -cos(ur(4))*sin(ur(6)) cos(ur(6))*sin(ur(5))+cos(ur(5))*sin(ur(4))*sin(ur(6));...
     sin(ur(6))*cos(ur(5))+sin(ur(4))*sin(ur(5))*cos(ur(6))  cos(ur(4))*cos(ur(6)) sin(ur(6))*sin(ur(5))-cos(ur(5))*sin(ur(4))*cos(ur(6));...
     -cos(ur(4))*sin(ur(5))                                  sin(ur(4))            cos(ur(4))*cos(ur(5))];
f = [ur(7:9);...
     G\(sensor.omg-bg-Ng);...
     g+Rq*(sensor.acc-ba-Na)]; %  9*1
 
% measurement model
C = [eye(3) zeros(3,3) zeros(3,3);zeros(3,3) eye(3) zeros(3,3); zeros(3,3) zeros(3,3) eye(3)];
% Z = C*ur

% prediction
t_step = sensor.t-t;
t = sensor.t;
up = ur+t_step*f;

A = findjacobian2(sensor.omg,sensor.acc,g,Na,Ng,ba,bg,ur); %9*9 !!!
F = eye(9) + t_step*A;
U = eye(9); % ???
V = U*t_step; 
cp = F*cr*F'+ V*Q*V';

if ~isempty(sensor.id) % update
    K = varargin{1};
    H_cr = varargin{2};
    P = varargin{3}; 
    [p, q, ~, ~] = estimate_pose(sensor, K,H_cr,P);
    v = (p-ur(1:3))/t_step;
    Z = [p;q;v];
    [yaw,roll,pitch] = quat2angle(q', 'ZXY');
    z = [p;roll;pitch;yaw;v]; 
    W = eye(9); % ???
    K = cp*C'/(C*cp*C'+W*R*W'); 
    ur = up + K*(z-C*up);
    cr = cp - K*C*cp;
else
    ur = up;
    cr = cp;
    Z = zeros(10,1);
end
quat = angle2quat(ur(6), ur(4), ur(5), 'ZXY');
X = [ur(1:3);ur(7:9);quat'];

end


end
