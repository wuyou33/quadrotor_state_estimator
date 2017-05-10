function test()
init_script;
load('data/studentdata4.mat');
n = size(data,2);
velocity = zeros(3,n);
omega = zeros(3,n);
dtime = zeros(1,n);
for i = 1:n
    if isempty(data(i).id)
        velocity(:,i) =[];
        omega(:,i) =[];
        dtime(i) = [];
    else
        [velocity(:,i) , omega(:,i)]= estimate_vel(data(i),K,H_cr,P);
        dtime(i) = data(i).t;
    end
end

vtime = time;
vvelocity = vicon(7:9,:);
vomega = vicon(10:12,:);

figure
title('X linear velocity');
plot(dtime,velocity(1,:),'g',vtime,vvelocity(1,:),'r');
figure
title('Y linear velocity');
plot(dtime,velocity(2,:),'g',vtime,vvelocity(2,:),'r');
figure
title('Z linear velocity');
plot(dtime,velocity(3,:),'g',vtime,vvelocity(3,:),'r');
figure
title('X angular velocity');
plot(dtime,omega(1,:),'g',vtime,vomega(1,:),'r');
figure
title('Y angular velocity');
plot(dtime,omega(2,:),'g',vtime,vomega(2,:),'r');
figure
title('Z angular velocity');
plot(dtime,omega(3,:),'g',vtime,vomega(3,:),'r');

end