function acc = plan_traj(x0,r,T,N,visualise,graph)
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
model = cell(M,1);
for i = 1:M
    model{i}.A = A;
    model{i}.B = B;
    model{i}.umax = 100;
    model{i}.umin = -100;
end

nx = size(model{1}.A,1); %state vector dimension
nu = size(model{1}.B,2); %number of inputs
Nd = 3; %number of dimensions

Objective = cell(M,1);
for i = 1:M
    Q = eye(Nd)*10;
    Objective{i}.Q = blkdiag(Q,zeros(nx-Nd));
    Objective{i}.R = eye(nu)*13;
    Objective{i}.N = N;
end

constraints = cell(M,1);
for i = 1:M
    constraints{i}.r = r(i,:);
    constraints{i}.x0 = x0(i,:);
    constraints{i}.N_j = find(graph(i,:));
    constraints{i}.delta = 0.5;
end
%% ADMM Vectors

ADMM = cell(M,1);
for i = 1:M
    ADMM{i}.rho = 40; %weighting of constraint violation
    
    lambda = zeros(Nd,1); %Lagrangian variable
    lambda_to_ = zeros(Nd,1); %used to construct lagrangians for each agent
    lambda_from_ = zeros(Nd,1);
    
    ADMM{i}.w = zeros(Nd*(N+1),1); %Will effectively be the proposed trajectory by one drone for another
    ADMM{i}.x = zeros(nx*(N+1),1);
    w_from_ = zeros(Nd,1);
    w_to_ = zeros(Nd,1);
    
    ADMM{i}.lambda = repmat(lambda, N+1, 1);
    
    ADMM{i}.lambda_to_j = repmat(lambda_to_, N+1,size(constraints{i}.N_j,2));
    ADMM{i}.lambda_from_j = repmat(lambda_from_, N+1, size(constraints{i}.N_j,2));
    ADMM{i}.w_from_j = repmat(w_from_, N+1, size(constraints{i}.N_j,2));
    ADMM{i}.w_to_j = repmat(w_to_, N+1, size(constraints{i}.N_j,2));
end


x_init = zeros(nx*(N+1),M);
u_init = zeros(nu*N,M);
for i = 1:M
    [x_init(:,i),u_init(:,i)] = init_traj(ADMM{i}.rho,Objective{i},constraints{i},model{i}); %initial solution
    ADMM{i}.x = x_init(:,i);
    ADMM{i}.u = u_init(:,i);
end

%% optimisation

for i =1:M
    ADMM{i}.x_bar(:,1) = x_init(:,i);
    for j = 1:size(constraints{i}.N_j,2)
        ADMM{i}.x_bar(:,j+1) = x_init(:,constraints{i}.N_j(j));
    end
end

addpath Funz;

for nit = 1:45
    
    %% Prediction
    for i = 1:M
        ADMM{i} = predict_step(ADMM{i},Objective{i},constraints{i},model{i});
    end
    
    
    %% Coordination & communication
    for i = 1:M
        ADMM{i}.x_bar(:,1) = ADMM{i}.x;
        for j = 1:size(constraints{i}.N_j,2)
            ADMM{i}.x_bar(:,j+1) = ADMM{constraints{i}.N_j(j)}.x;
        end
    end
    
    for i = 1:M
        ADMM{i} = coordinate_step(ADMM{i},Objective{i},constraints{i});
    end
    %% Mediation
    
    for i = 1:M
        
        ADMM{i} = mediate_step(ADMM{i},constraints{i},Objective{i}.N);
        
    end
    %% Final Communication
    
    for i = 1:M
        for j = 1:size(constraints{i}.N_j,2)
            curr_comm = constraints{i}.N_j(j);
            ADMM{i}.lambda_from_j(:,j) = ADMM{curr_comm}.lambda_to_j(:,(constraints{curr_comm}.N_j==i));
            ADMM{i}.w_from_j(:,j) = ADMM{curr_comm}.w_to_j(:,(constraints{curr_comm}.N_j==i));
        end
        
    end
    
end

%% Command extraction

acc = zeros(nu*N,M);
x = zeros(nx*(N+1),M);
for i = 1:M
    acc(:,i) = ADMM{i}.u;
    x(:,i) = ADMM{i}.x;
end


%% visualisation

if visualise
    visualise_drones_3d(r,x,N,T)
end
end