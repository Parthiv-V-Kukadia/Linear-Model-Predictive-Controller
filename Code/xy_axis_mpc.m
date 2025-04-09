function j = xy_axis_mpc(K,dt,p_0,v_0,a_0,pt,vt,at)
    %% Cost function weight
    w1 = 100; % tracking error
    w2 = 1;   % velocity
    w3 = 1;   % acceleration
    w4 = 1;   % jerk
    w5 = 1e4;   % soft constraint

    %% Constraints for x-y Velocity, Acceleration, & Jerk
    v_max = 6;
    v_min = -6;
    a_max = 3;
    a_min = -3;
    jerk_max = 3;
    jerk_min = -3;

    %% Construct the prediction matrix
    [Tp, Tv, Ta, Bp, Bv, Ba] = getPredictionMatrix(K,dt,p_0,v_0,a_0);
    
    %% Construct the optimization problem
    H = blkdiag(w4*eye(K)+w1*(Tp'*Tp)+w2*(Tv'*Tv)+w3*(Ta'*Ta),w5*eye(K));
    F = [w1*(Bp-pt)'*Tp+w2*(Bv-vt)'*Tv+w3*(Ba-at)'*Ta, zeros(1,K)];
    
    %% Construct the soft constraint matrices (A and b)
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

    %% Hard Constraints for jerk
    lower_bound = jerk_min * ones(2*K, 1);
    upper_bound = jerk_max * ones(2*K, 1);
    
    %% Solve the optimization problem using quadprog
    J = quadprog(H, F, A, b, [], [], lower_bound, upper_bound);
    
    %% Apply the control
    j = J(1);
end