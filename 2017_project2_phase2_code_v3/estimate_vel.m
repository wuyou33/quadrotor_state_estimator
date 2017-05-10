function [vel , omg] = estimate_vel(sensor, varargin)
%ESTIMATE_VEL 6DOF velocity estimator
%   sensor - struct stored in provided dataset, fields include
%          - is_ready: logical, indicates whether sensor data is valid
%          - t: timestamp
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
%              estimate_vel_handle = ...
%                  @(sensor) estimate_vel(sensor, your personal input arguments);

%   vel - 3x1 velocity of the quadrotor in world frame
%   omg - 3x1 angular velocity of the quadrotor

persistent pts t td tracker output; % corner position in image, time, time difference

if isempty(sensor.id) % return empty for empty packets
    vel =[];
    omg =[];
    t = sensor.t;
    
else
    if isempty(pts) % 1st non-empty frame
    pts = detectFASTFeatures(sensor.img,'MinQuality',0.8); % detect corners, more properties
    pts = pts.selectStrongest(50); % select 100 strongest corners
    pts = pts.Location;
    PointTracker = vision.PointTracker('MaxBidirectionalError',0.5);
    initialize(PointTracker,pts,sensor.img); 
    tracker = PointTracker; % pass the initialization
        if isempty(t)
           t=0;
        end
    td = sensor.t-t;
    t = sensor.t;
    vel =NaN(3,1); % 
    omg =NaN(3,1);
    
    else % later frames
    points_prev = pts; 
    t_prev = t;
    td_prev = td;
    [points, ~, scores] = step(tracker,sensor.img);
    inliners_id = find(scores>=0.9985);
    m = length(inliners_id); % num of valid corners with high confidence score (not valid, score=0)
    
    % detect new corners and initialize new tracker
    pts = detectFASTFeatures(sensor.img,'MinQuality',0.8); 
    pts = pts.selectStrongest(50);
    pts = pts.Location;
    PointTracker = vision.PointTracker('MaxBidirectionalError',0.5); % new tracker
    initialize(PointTracker,pts,sensor.img); 
    tracker = PointTracker;
    t = sensor.t;
    
    K = varargin{1};
    H_cr = varargin{2};
    P = varargin{3};
    [~, ~, H, H_wr] = estimate_pose(sensor,K,H_cr,P); % homography
    C_prev = (K \ [points_prev(inliners_id,1)';points_prev(inliners_id,2)';ones(1,m)])'; % only valid points will be counted
    C = (K \ [points(inliners_id,1)';points(inliners_id,2)';ones(1,m)])'; % m*3  convert from image frame to camera frame
    W = H \ C'; % 3*m Z*C=H*W
    Z = 1./W(3,:)'; % m*1 corner depth 1/lambda
    
    % low-pass filter on t
    alpha = 0.17; % tuning later 0.2
    td = alpha*(t-t_prev)+(1-alpha)*td_prev; % assume at a constant rate
    V = (C-C_prev)./td; % m*3 velocity in camera frame
    
    % RANSAC
    % X = ransac(C,Z,V,3,0.999999,0.02,0.8); % need tuning, mainly on dist and ratio (last two together)
    % 0.999999,0.012,0.5
    
    F(1:m,:) = [-1./Z , zeros(m,1) , C(:,1)./Z , C(:,1).*C(:,2) , -(1+C(:,1).^2) , C(:,2)];
    F(m+1:2*m,:) = [zeros(m,1), -1./Z,  C(:,2)./Z,  1+C(:,2).^2, -C(:,1).*C(:,2), -C(:,1)];
    X = F \ [V(:,1);V(:,2)]; 
    
    % convert to quadrotor frame
%     omg = (H_cr(1:3,1:3))'*X(4:6);
%     vel = (H_cr(1:3,1:3))'*X(1:3);
%     r = H_wr*[-(H_cr(1:3,1:3))'*H_cr(1:3,4);1];
%     r = r(1:3);
%     vel = X(1:3)-cross(omg,r);
    X_quadrotor = [(H_cr(1:3,1:3))' -(H_cr(1:3,1:3))'*[0 -H_cr(3,4) H_cr(2,4);H_cr(3,4) 0 -H_cr(1,4);-H_cr(2,4) H_cr(1,4) 0]; zeros(3,3) (H_cr(1:3,1:3))']*X;
    
    % convert to world frame
    vel0 = H_wr(1:3,1:3)*X_quadrotor(1:3);
    omg0 = H_wr(1:3,1:3)*X_quadrotor(4:6);
    
    % filter spikes
    if isempty(output) % 1st vel&omg, take it whatever
        vel = vel0;
        omg = omg0;
        output = [vel;omg];
    else
        output_prev = output;
        beta1 = 0.35; % low pass filter for vel&omg
        beta2 = 0.98;
        beta3 = 0.2;
        beta4 = 0.7;
        if norm(vel0-output_prev(1:3),inf) < 0.2
            vel = (1-beta1)*output_prev(1:3)+ beta1 * vel0;
        else
            vel = (1-beta3)*output_prev(1:3)+ beta3 * vel0;
        end
        if norm(omg0-output_prev(4:6),inf) < 0.4
            omg = (1-beta2)*output_prev(4:6)+ beta2 * omg0;
        else
            omg = (1-beta4)*output_prev(4:6)+ beta4 * omg0;
        end
        output = [vel;omg];
        % 0.45;0.95;0.1;0.7...0.2;0.4
        
    end
    
    end
end

end