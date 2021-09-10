function ADMM = predict_step(ADMM,Objective,constraints,model)

Q = Objective.Q;
R = Objective.R;
N = Objective.N;

Nd = 3;
nx = size(Q,1);
nu = size(R,1);

A = model.A;
B = model.B;
umax = model.umax;
umin = model.umin;


N_j = constraints.N_j;
r = constraints.r;
x0 = constraints.x0;

rho = ADMM.rho;
lambda = ADMM.lambda;
lambda_from_j = ADMM.lambda_from_j;
w = ADMM.w;
w_from_j = ADMM.w_from_j;

min_input = repmat(ones(nu,1)*umin,N,1);
max_input = repmat(ones(nu,1)*umax,N,1);
% - linear dynamics

u_bound = [inf((N+1)*nx,1);max_input];
l_bound = [-inf((N+1)*nx,1);min_input];


posM = blkdiag(eye(Nd),zeros(nx-Nd)); % Matrix to take only position from the state vector x (zeros rest)
d = [eye(Nd),zeros(Nd,nx-Nd)]; % Matrix to take only position from the state vector x (only returns pos)
posMN = kron(eye(N+1),d); %returns pos across time steps

Ax = kron(speye(N+1), -speye(nx)) + kron(sparse(diag(ones(N, 1), -1)), A);
Bu = kron([sparse(1, N); speye(N)], B);
Aeq = [Ax, Bu];
b_eq = [-x0'; zeros(N*nx, 1)];
P_new = blkdiag( kron(speye(N), Q+ (size(N_j,2)+1)*rho/2*posM ), Q+(size(N_j,2)+1)*rho/2*posM, kron(speye(N), R) );

q = prediction_linear(lambda,lambda_from_j,w,w_from_j,rho,r',Q,N,nu,posMN);

qp_opts = optimoptions('quadprog','Display','off');
% res = quadprog(P_new,q,[],[], Aeq,b_eq,l_bound,u_bound,[],qp_opts);
res = qp_grad(P_new,q,[],[], Aeq,b_eq,l_bound,u_bound,[ADMM.x;ADMM.u],1e-2);
ADMM.x = res(1:nx*(N+1));
ADMM.u = res((N+1)*nx+1:end);

end