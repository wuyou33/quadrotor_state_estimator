function [X] = ransac(C,Z,V,num,rate,dist,ratio)
% X: vel;omg 6*1
% C: 2*m/2*n dataset with n points before/after RANSAC
% Z: corner depth m*1
% V: velocity 3*m
% num: num of points needed for random sampling
% rate: success rate of RANSAC, which decide the iteration
% dist: threshold dist between inliners and fitting line
% ratio: threshold of ratio, inliners to all points

m = size(C,2); % num of all points
iter = round(log(1-rate)/log(1-ratio^num)); % too small here???
max_inliners_num = 0; 

points_c = C';
F(1:m,:) = [-1./Z , zeros(m,1) , points_c(:,1)./Z , points_c(:,1).*points_c(:,2) , -(1+points_c(:,1).^2) , points_c(:,2)];
F(m+1:2*m,:) = [zeros(m,1), -1./Z,  points_c(:,2)./Z,  1+points_c(:,2).^2, -points_c(:,1).*points_c(:,2), -points_c(:,1)];

for i = 1:iter
    % randomly select num points for sampling and solve velocities
    random = randperm(m,num); 
    sample_F(1:num,:) = F(random,:);
    sample_F(num+1:2*num,:) = F(random+m,:);
    sample_V = V(:,random);
    sample_X = sample_F\[sample_V(1,:)';sample_V(2,:)']; 
    
    % compute the distance between all points's real optical velocity to
    % estimated velocity from X
    V_estimate = F*sample_X; % 2m*1
    error = V(1:2,:)-[V_estimate(1:m) V_estimate(m+1:2*m)]';% 2*m
    distance = sqrt(error(1,:).^2+error(2,:).^2); % 1*m might be problem here
    
    % threshold the inliers
    inliners_id = find(distance<=dist);
    inliners_num = length(inliners_id);
    
    % Update inliers and the model if a better model is found     
    if inliners_num > max_inliners_num
         max_inliners_num = inliners_num;
         best_inliners_id = inliners_id;
    end
    if max_inliners_num >= round((ratio+0.1)*m) % save time
        break
    end
end

optimized_F(1:max_inliners_num,:) = F(best_inliners_id,:);
optimized_F(max_inliners_num+1:2*max_inliners_num,:) = F(best_inliners_id+m,:);
optimized_V = V(:,best_inliners_id);
X = optimized_F \ [optimized_V(1,:)';optimized_V(2,:)']; 
   
end