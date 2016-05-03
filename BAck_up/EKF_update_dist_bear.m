function [x_hat_plus,P_plus,res,S] = EKF_update_dist_bear(x_hat_min, P_min, z_g1,z_g2)

% Let us assume that zg1 contains the distance measurement and
% zg2 has the bearing measurement
N = size(x_hat_min,1);
no_of_lm = (N-3)/2;

% to give the mahalobis distance range (can change) 
epsilon_1 = 0.95; %This value was specified in a presentation
epsilon_2 = 1.5;

% to get the position from the measured value same as given the known
% measurement
x_value = z_g1*cos(z_g2);
y_value = z_g1*sin(z_g2);
% Initializing the covariance matrix for noise
sigma_d = 0.01;
sigma_theta = 0.01;
R = zeros(2);
R(1,1) = sigma_d^2;
R(2,2) = sigma_theta^2;
% Changing R to suit the position measurement problem
G = [cos(z_g2) -z_g1*sin(z_g2);
     sin(z_g2)  z_g1*cos(z_g2)];
R_new = G*R*G';

% Update process. 
C = [cos(x_hat_min(3,1)) -sin(x_hat_min(3,1));
     sin(x_hat_min(3,1))  cos(x_hat_min(3,1))];
J = [0 -1;1 0];
% Initializing first landmark
if(no_of_lm == 0)
   [x_hat_plus,P_plus,res,S] = Initialize_Landmark(x_hat_min, P_min,x_value,y_value,no_of_lm,R_new);
else
   j = 1;
   g = zeros(1,no_of_lm);
   for i=4:2:N
       H = zeros(2,N);
       m_dis = [x_hat_min(i,1)-x_hat_min(1,1);x_hat_min(i+1,1)-x_hat_min(2,1)];
       z_hat = C*m_dis;
       res = [x_value;y_value] - z_hat;
       H(1:2,1:2) = -C';
       H(1:2,3) = -C'*J*m_dis;
       H(1:2,i:i+1) = C';
       S = H*P_min*H'+R_new;
       g(1,j) = res'*S*res;
       j = j+1;
   end
   [g_min,value] = min(g(1,:));
   % Mahalanobis Distance Test 
   if(g_min<epsilon_1)
      % apply update
      [x_hat_plus,P_plus,res,S] = Update_Landmark(x_hat_min, P_min,x_value,y_value,value,R_new);
   elseif(g_min>epsilon_2)
      % apply initialization
      [x_hat_plus,P_plus,res,S] = Initialize_Landmark(x_hat_min, P_min,x_value,y_value,no_of_lm,R_new);
   else
       % don't do anything
   end
end
   
  

