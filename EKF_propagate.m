function[x_hat_min,P_min] =EKF_propagate(p_plus,sigma_g,x_k,v_m,omega_m,dt,M)

% To set the state in the propagation step
% The land mark state does not change in the propogation step.
% M is the number of times we want to propagate before udate

N = size(x_k,1); % number of rows

x_inter = zeros(3,M+1);
x_inter(1,1) = x_k(1,1);
x_inter(2,1) = x_k(2,1);
x_inter(3,1) = x_k(3,1);

P_k = eye(3);
G_k = zeros(3,2);

P_min = p_plus;
P_inter = eye(3);

for i=1:M
    x_inter(1,i+1) = x_inter(1,i)+(v_m*dt*cos(x_inter(3,i)));
    x_inter(2,i+1) = x_inter(2,i)+(v_m*dt*sin(x_inter(3,i)));
    x_inter(3,i+1) = x_inter(3,i)+(omega_m*dt);

    % To find the new covaraince matrix

    P_k(1,3) = -v_m*dt*sin(x_inter(3,i));
    P_k(2,3) = v_m*dt*cos(x_inter(3,i));

    G_k(1,1) = dt*cos(x_k(3,i));
    G_k(2,1) = dt*sin(x_k(3,i));
    G_k(3,2) = dt;

    P_min(1:3,1:3) = (P_k*p_plus(1:3,1:3)*P_k')+(G_k*sigma_g*G_k');
    P_inter = P_inter*P_k;
end

% Final propagation calculation
x_hat_min(1,1) = x_inter(1,M+1);
x_hat_min(2,1) = x_inter(2,M+1);
x_hat_min(3,1) = x_inter(3,M+1);

P_min(1:3,4:N) = P_inter*p_plus(1:3,4:N);
P_min(4:N,1:3) = p_inter*p_plus(4:N,1:3);




  
