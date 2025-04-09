function j = z_axis_mpc(K,dt,p_0,v_0,a_0,pt,vt,at)
w1 = 100;
w2 = 1;
w3 = 1;
w4 = 1;
w5 = 1e4;

%% Constraints for x-y Velocity, Acceleration, & Jerk
v_max = 6;
v_min = -1;
a_max = 3;
a_min = -1;
jerk_max = 2;
jerk_min = -2;

%% Construct the prediction matrix
[Tp, Tv, Ta, Bp, Bv, Ba] = getPredictionMatrix(K,dt,p_0,v_0,a_0);

%% Construct the optimization problem
H = blkdiag(w4*eye(K)+w1*(Tp'*Tp)+w2*(Tv'*Tv)+w3*(Ta'*Ta),w5*eye(K));
F = [w1*(Bp-pt)'*Tp+w2*(Bv-vt)'*Tv+w3*(Ba-at)'*Ta, zeros(1,K)];

%% Construct the constraint matrices (A and b)
% Velocity constraints
A_v = [Tv, zeros(K); -Tv, -eye(K)];
b_v = [v_max * ones(K, 1) - Bv; -v_min * ones(K, 1) + Bv];
    
% Acceleration constraints
A_a = [Ta, zeros(K); -Ta, zeros(K)];
b_a = [a_max * ones(K, 1) - Ba; -a_min * ones(K, 1) + Ba];
    
% Jerk constraints
A_j = [zeros(size(Ta)), -eye(K)];
b_j = [zeros(K,1)];
    
% Combine all constraints
A = [A_v; A_a; A_j];
b = [b_v; b_a; b_j];

%A = [Tv, zeros(K);-Tv, -eye(K);Ta, zeros(K);-Ta, zeros(K); zeros(size(Ta)), -eye(K)];
%b = [6*ones(20,1)-Bv;-1*ones(20,1)+Bv;3*ones(20,1)-Ba;-1*ones(20,1)+Ba; zeros(K,1)];

%% Hard Constraints for jerk
lower_bound = jerk_min * ones(K, 1);
upper_bound = jerk_max * ones(K, 1);

%% Solve the optimization problem
J = quadprog(H,F,A,b,[],[],lower_bound,upper_bound);

%% Apply the control
j = J(1); % first jerk value to apply
end