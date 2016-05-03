function [x_hat_plus,P_plus,res,S] = Update_Landmark(x_hat_min, P_min,x_value,y_value,value,R_new)

N = size(x_hat_min,1);
C = [cos(x_hat_min(3,1)) -sin(x_hat_min(3,1));
     sin(x_hat_min(3,1))  cos(x_hat_min(3,1))];
J = [0 -1;1 0];
ind = 3+(value-1)*2; 
H = zeros(2,N);

m_dis = [x_hat_min(ind+1,1)-x_hat_min(1,1);x_hat_min(ind+2,1)-x_hat_min(2,1)];
H(1:2,1:2) = -C';
H(1:2,3) = -C'*J*m_dis;
H(1:2,ind+1:ind+2) = C';
z_hat = C*m_dis; 
res = [x_value;y_value] - z_hat;
S = H*P_min*H'+R_new;  
S_inv = inv(S);
K = P_min*H'*S_inv;
x_hat_plus = x_hat_min + K*res;
P_plus = P_min+ K*S*K';
