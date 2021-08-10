clear
N = 100;
r = [1000.0,1.4,0,0,0,0; 0.6,0.4,0,0,0,0; 0,0.9,0,0,0,0; 0,0,1,0,0,0];
x0 = [0,0.5,0,0,0,0;  0.2,0.9,0,0,0,0; 1.0,0.3,0,0,0,0; 0,0,0,0,0,0];

T = 0.1;
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
graph = ones(M) - eye(M);
model.A = A;
model.B = B;
model.umax = 100;
model.umin = -100;

nx = size(model.A,1); %state vector dimension
nu = size(model.B,2); %number of inputs
Nd = 3; %number of dimensions

Q = eye(Nd)*10;
Objective.Q = blkdiag(Q,zeros(nx-Nd));
Objective.R = eye(nu)*13;
Objective.N = N;

constraints_struct = [];
for i = 1:M
    constraints.r = r(i,:);
    constraints.x0 = x0(i,:);
    constraints.delta = 0.5;
    constraints.N_j = find(graph(i,:));
    constraints_struct = [constraints_struct;constraints];
end




%% ADMM Vectors
ADMM_vec = [];
for i = 1:M
    ADMM_struct.rho = 40; %weighting of constraint violation

    lambda = zeros(Nd,1); %Lagrangian variable
    lambda_to_ = zeros(Nd,1); %used to construct lagrangians for each agent
    lambda_from_ = zeros(Nd,1);

    ADMM_struct.w = zeros(Nd*(N+1),1); %Will effectively be the proposed trajectory by one drone for another
    ADMM_struct.x = zeros(nx*(N+1),1);
    ADMM_struct.u = zeros(nu*N,1);

    w_from_ = zeros(Nd,1);
    w_to_ = zeros(Nd,1);

    ADMM_struct.lambda = repmat(lambda, N+1, 1);

    ADMM_struct.lambda_to_j = repmat(lambda_to_, N+1,size(constraints_struct(i).N_j,2));
    ADMM_struct.lambda_from_j = repmat(lambda_from_, N+1, size(constraints_struct(i).N_j,2));
    ADMM_struct.w_from_j = repmat(w_from_, N+1, size(constraints_struct(i).N_j,2));
    ADMM_struct.w_to_j = repmat(w_to_, N+1, size(constraints_struct(i).N_j,2));
    ADMM_struct.x_bar = zeros(nx*(N+1),M);
    ADMM_vec = [ADMM_vec;ADMM_struct];
end
x_init = zeros(nx*(N+1),M);

for i = 1:M
    x_init(:,i) = init_traj(ADMM_struct.rho,Objective,constraints_struct(i),model); %initial solution
end
for i =1:M
    ADMM_vec(i).x_bar(:,1) = x_init(:,i);
    for j = 1:size(constraints_struct(i).N_j,2)
        ADMM_vec(i).x_bar(:,j+1) = x_init(:,constraints_struct(i).N_j(j));
    end
end
ADMM = Simulink.Bus;


fn = fieldnames(ADMM_struct);
for k = 1:numel(fn)
   elems(k) = Simulink.BusElement;
   elems(k).Name = fn{k};
   elems(k).Dimensions = size(ADMM_struct.(fn{k}));
end

ADMM.Elements = elems;

constraints = Simulink.Bus;
fn = fieldnames(constraints_struct(1));

for k = 1:numel(fn)
   con_elems(k) = Simulink.BusElement;
   con_elems(k).Name = fn{k};
   con_elems(k).Dimensions = size(constraints_struct(1).(fn{k}));
end

constraints.Elements = con_elems;