function [v_m, w_m, z, x_true] = rws(v_true,w_true,LM,d_max,dt)

N = size(LM,1);
std_d = 0.01;
std_theta = 0.01;

% Preallocate memory
z = zeros(2,N); 
% Odometry measurements
v_m = v_true + 0.01*v_true .* randn(1,N-1);
w_m = w_true + 0.04*v_true .* randn(1,N-1);

x_true = zeros(3,N);
for k=2:N
    x_true(1,k) = x_true(1,k-1)+v_true(1,k-1)*dt*cos(x_true(3,k-1));
    x_true(2,k) = x_true(2,k-1)+v_true(1,k-1)*dt*sin(x_true(3,k-1));
    x_true(3,k) = x_true(3,k-1)+w_true(1,k-1)*dt;
end
% measurements for each landmark
for i = 1:N   
    % distance measurement
    distance = sqrt((LM(i,1)-x_true(1,i))^2+(LM(i,2)-x_true(2,i))^2)+std_d*randn(1,1);
    % To consider landmarks in a given range
    if(distance<d_max) 
        z(1,i) = distance;
        z(2,i)= atan2(LM(i,2)-x_true(2,i), LM(i,1)-x_true(1,i)) - x_true(3,i) + std_theta*randn(1,1);
    else
        z(1,i) = -1;
        z(2,i) = -1;
    end
       
end
end
    




