function [acc,sum] = plan_traj(x0,r,T,N,visualise,graph)
A = [1 0 0 T 0 0;
    0 1 0 0 T 0;
    0 0 1 0 0 T;
    0 0 0 1 0 0;
    0 0 0 0 1 0;
    0 0 0 0 0 1];

B = [0 0 0;
    0 0 0;
    0 0 0;
    T 0 0;
    0 T 0;
    0 0 T];

M = size(x0,1); % Number of agents

nx = 6; %state vector dimension
nu = 3; %number of inputs
Nd = 3; %number of dimensions

Q = eye(Nd)*10; 
Q = blkdiag(Q,zeros(nx-Nd));
R = eye(nu)*13;

%% ADMM Vectors
N_j = cell(M,1);

for i = 1:M
    N_j{i} = find(graph(i,:));
end

rho = 40; %weighting of constraint violation

lambda = zeros(Nd,1); %Lagrangian variable
lambda_to_ = zeros(Nd,1); %used to construct lagrangians for each agent
lambda_from_ = zeros(Nd,1);

w = zeros(Nd,1); %Will effectively be the proposed trajectory by one drone for another
x = zeros(nx,1);
w_from_ = zeros(Nd,1);
w_to_ = zeros(Nd,1);

lambda = repmat(lambda, N+1, M);

lambda_to_j = cell(M,1);
lambda_from_j = cell(M,1);
w_to_j = cell(M,1);
w_from_j = cell(M,1);

for i = 1:M
    lambda_to_j{i} = repmat(lambda_to_, N+1,size(N_j{i},2));
    lambda_from_j{i} = repmat(lambda_from_, N+1, size(N_j{i},2));
    w_from_j{i} = repmat(w_from_, N+1, size(N_j{i},2));
    w_to_j{i} = repmat(w_to_, N+1, size(N_j{i},2));
end

w = repmat(w, N+1, M);
x = repmat(x, N+1, M);


delta = 0.5; % Inter-agent distance

%% Useful matrices

posM = blkdiag(eye(Nd),zeros(nx-Nd)); % Matrix to take only position from the state vector x (zeros rest)
d = [eye(Nd),zeros(Nd,nx-Nd)]; % Matrix to take only position from the state vector x (only returns pos)
posMN = kron(eye(N+1),d); %returns pos across time steps

rhoM = kron(speye((N+1)),rho/2*eye(nu)); % matrix for the quadratic objective formualation

% Transformation matrix H creation
% Multiplies state vector to return a vector consisting of difference in
% positions between agents
H = cell(M,1);
Hw = cell(M,1);
for i = 1:M
    H{i} = kron(eye(N+1),repmat(d, size(N_j{i},2), 1));
    Hw{i} = kron(eye(N+1),repmat(eye(nu), size(N_j{i},2), 1));
    for j = 1:size(N_j{i},2)
        v = zeros(size(N_j{i},2),1);
        v(j) = 1;
        H_M = kron(eye(N+1),kron(v, -1*d));
        H_Mw = kron(eye(N+1),kron(v, -1*eye(nu)));

        H{i} =  [H{i}, H_M];
        Hw{i} = [Hw{i},H_Mw];
    end
end
%% The MPC Problem
% Cast MPC problem to a QP: x = (x(0),x(1),...,x(N),u(0),...,u(N-1))

% - quadratic objective for prediction
P = blkdiag( kron(speye(N), Q ), Q, kron(speye(N), R) );
P_new = cell(M,1);
for i = 1:M
    P_new{i} = blkdiag( kron(speye(N), Q+ (size(N_j{i},2)+1)*rho/2*posM ), Q+(size(N_j{i},2)+1)*rho/2*posM, kron(speye(N), R) );
end
% - linear dynamics
Ax = kron(speye(N+1), -speye(nx)) + kron(sparse(diag(ones(N, 1), -1)), A);
Bu = kron([sparse(1, N); speye(N)], B);
Aeq = [Ax, Bu];
% quadratic objective for coordination
Pc = cell(M,1);
for i = 1:M
    Pc{i} = kron(eye(1+size(N_j{i},2)),rhoM);
end
u_bound = zeros((N+1)*nx+N*nu,M);
l_bound = zeros((N+1)*nx+N*nu,M);
b_eq = zeros((N+1)*nx,M);
q = zeros((N+1)*nx+N*nu,M);
res = zeros((N+1)*nx+N*nu,M);
x_bar = cell(M,1);

qp_opts = optimoptions('quadprog','Display','off');
for i = 1:M
    q(:,i) = [repmat(-Q*r(i,:)', N, 1); -Q*r(i,:)'; zeros(N*nu, 1)];
    % input constraints (here just between 1 & -1)
    umin = ones(nu,1)*-1;
    umax = ones(nu,1)*1;
    
    min_input = repmat(umin,N,1);
    max_input = repmat(umax,N,1);
    % - linear dynamics
    
    u_bound(:,i) = [inf((N+1)*nx,1);max_input];
    l_bound(:,i) = [-inf((N+1)*nx,1);min_input];
    
    b_eq(:,i) = [-x0(i,:)'; zeros(N*nx, 1)];
    res(:,i) = quadprog(P_new{i},q(:,i),[],[], Aeq,b_eq(:,i),l_bound(:,i),u_bound(:,i),[],qp_opts);
    x(:,i) = res(1:nx*(N+1),i); %initial solution
end

%% optimisation
qc = cell(M,1);
resc = cell(M,1);
l_ineq = cell(M,1);
for i = 1:M
    qc{i} = zeros(1+size(N_j{i},2)*(N+1)*Nd,1);
    A_ineq = cell(M,1);
    l_ineq{i} = zeros(size(N_j{i},2)*N,1);
    resc{i} = zeros(1+size(N_j{i},2)*(N+1)*Nd,1);
end
for i =1:M
   x_bar{i}(:,i) = x(:,i);
   for j = 1:size(N_j{i},2)
      x_bar{i}(:,N_j{i}(j)) = x(:,N_j{i}(j)); 
   end
end
addpath Funz;

for nit = 1:45
    
    %% Prediction
    for i = 1:M
        
        q(:,i) = prediction_linear(lambda(:,i),lambda_from_j{i},w(:,i),w_from_j{i},rho,r(i,:)',Q,Q,N,nu,M,posMN);

        res(:,i) = quadprog(P_new{i},q(:,i),[],[], Aeq,b_eq(:,i),l_bound(:,i),u_bound(:,i),[],qp_opts);
        x(:,i) = res(1:nx*(N+1),i);
        x_bar{i}(:,i) = x(:,i);
    end
    
    %% Coordination
    for i = 1:M    
        %- linear objective for coordination
        qc{i} = coordination_linear(i,lambda(:,i),lambda_to_j{i},rho,N_j{i},M,x,posMN);
        
        % Update matrices
        [A_ineq{i},l_ineq{i}] = communicate(i,x_bar{i},N,N_j{i},M,H{i},Hw{i},delta,nu); %shares x with all neighbours
        resc{i} = quadprog(Pc{i},qc{i},-A_ineq{i},-l_ineq{i},[],[],[],[],[],qp_opts);
        w(:,i) = resc{i}(1:Nd*(N+1),1);
        for j = 1:size(N_j{i},2)
            w_to_j{i}(:,j) = resc{i}(j*Nd*(N+1)+1:(j+1)*Nd*(N+1),1);
        end
    end
    %% Mediation & communication
    
    for i = 1:M

        lambda(:,i) = lambda(:,i) + rho * (posMN * x(:,i)-w(:,i));
        for j = 1:size(N_j{i},2)
            lambda_to_j{i}(:,j) = lambda_to_j{i}(:,j) + rho * (posMN*x(:,N_j{i}(j)) - w_to_j{i}(:,j));
        end
        for j = 1:size(N_j{i},2)
           x_bar{i}(:,N_j{i}(j)) = x(:,N_j{i}(j)); 
        end
        
    end
    %% Final Communication
    
    for i = 1:M
        for j = 1:size(N_j{i},2)
            lambda_from_j{i}(:,j) = lambda_to_j{N_j{i}(j)}(:,(N_j{N_j{i}(j)}==i));
            w_from_j{i}(:,j) = w_to_j{N_j{i}(j)}(:,(N_j{N_j{i}(j)}==i));
        end
        
    end
    
end

%% Objective calculation and command extraction

sum = 0;
acc = zeros(nu*N,M);
for i = 1:M
    finx = x(nx+1:end,i);
    finu = res((N+1)*nx+1:end,i);
    finQ = blkdiag(kron(speye(N-1), Q),Q);
    finR = kron(speye(N), R);
    finr = repmat(r(i,:)', N, 1);
    acc(:,i) = finu;
    
    sum = sum+(finx-finr)'*finQ*(finx-finr)+finu'*finR*finu;
end


%% visualisation

if visualise
    visualise_drones_3d(r,x,N,T)
end
end