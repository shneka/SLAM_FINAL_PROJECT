function [x_hat_plus,P_plus,res,S] = Initialize_Landmark(x_hat_min, P_min,x_value,y_value,no_of_lm,R_new)

C = [cos(x_hat_min(3,1)) -sin(x_hat_min(3,1));
     sin(x_hat_min(3,1))  cos(x_hat_min(3,1))];
J = [0 -1;1 0];
ind = 3+(no_of_lm*2);

B = zeros(2,ind);
I = eye(2);
large = 1000000;

P_inter = [P_min B';B large*I];
x_hat_inter = [x_hat_min;0;0];  
m_dis = [-x_hat_min(1,1);-x_hat_min(2,1)];
H(1:2,1:2) = -C';
H(1:2,3) = -C'*J*m_dis;
H(1:2,ind+1:ind+2) = C';
z_hat = C*m_dis; 
res = [x_value;y_value] - z_hat;
S = H*P_inter*H'+R_new;  
S_inv = inv(S);
K = P_inter*H'*S_inv;
x_hat_plus = x_hat_inter + K*res;
P_plus = P_inter+ K*S*K';

