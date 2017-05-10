function H = CalculateHomography(u,v,X,Y)
%  Compute the homography matrix from source(X,Y) to destination(u,v)
%
%    u,v are coordinates of destination points
%    X,Y are coordinates of source points
%    X/Y/x/y , each is a vector of nx1,n>=4
%    Wudao Ling

n = size(X,1); % num of correspondence
A = zeros(n*2,9);

A(1:n,1:3) = [X Y ones(n,1)];
A(1:n,7:9) = -A(1:n,1:3).*(u*[1 1 1]);
A(n+1:2*n,4:6) =  A(1:n,1:3);
A(n+1:2*n,7:9) = -A(1:n,1:3).*(v*[1 1 1]);

% for i = 1:n
%  a = [X(i),Y(i),1];
%  b = [u(i),v(i)];
%  c = zeros(1,3);
%  A((i-1)*2+1:(i-1)*2+2,:) = [a c -b(1)*a;c a -b(2)*a];
% end

[~,~,V] = svd(A,'econ'); % economy svd
h = V(:,end);
H = [h(1:3)';h(4:6)';h(7:9)'];

H = H./H(3,3); % for calibration H33=1