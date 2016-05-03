function check_the_function()
P_min = ones(3)*0.01;
%x_hat_min = [1;2;0.5;0.9;2;0;1];
x_hat_min = [1;2;0];
z_g1 = 0.1;
z_g2 = pi/2;
[x_hat_min,P_min,~,~] = EKF_update_dist_bear(x_hat_min, P_min, z_g1,z_g2);
z_g1 = 0.15;
z_g2 = pi/2;
[x_hat_min,P_min,~,~] = EKF_update_dist_bear(x_hat_min, P_min, z_g1,z_g2);
z_g1 = 0.14;
z_g2 = pi/4;
[x_hat_min,P_min,~,~] = EKF_update_dist_bear(x_hat_min, P_min, z_g1,z_g2);
z_g1 = 0.099;
z_g2 = pi/2;
[x_hat_min,P_min,~,~] = EKF_update_dist_bear(x_hat_min, P_min, z_g1,z_g2);
z_g1 = 0.1005;
z_g2 = pi/2;
EKF_update_dist_bear(x_hat_min, P_min, z_g1,z_g2)