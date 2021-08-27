function [x_init,u_init] = init_traj(rho,Objective,constraints,model)

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

min_input = repmat(ones(nu,1)*umin,N,1);
max_input = repmat(ones(nu,1)*umax,N,1);
% - linear dynamics

u_bound = [inf((N+1)*nx,1);max_input];
l_bound = [-inf((N+1)*nx,1);min_input];


posM = blkdiag(eye(Nd),zeros(nx-Nd)); % Matrix to take only position from the state vector x (zeros rest)

Ax = kron(speye(N+1), -speye(nx)) + kron(sparse(diag(ones(N, 1), -1)), A);
Bu = kron([sparse(1, N); speye(N)], B);
Aeq = [Ax, Bu];
b_eq = [-x0'; zeros(N*nx, 1)];
P_new = blkdiag( kron(eye(N), Q+ (size(N_j,2)+1)*rho/2*posM ), Q+(size(N_j,2)+1)*rho/2*posM, kron(eye(N), R) );

q = [repmat(-Q*r', N, 1); -Q*r'; zeros(N*nu, 1)];

init = [repmat(x0',N+1,1);zeros(nu*N,1)];
qp_opts = optimoptions('quadprog','Display','off','Algorithm','Active-set');
res = quadprog(P_new,q,[],[], full(Aeq),b_eq,l_bound,u_bound,init,qp_opts);
x_init = res(1:nx*(N+1));
u_init = res(nx*(N+1)+1:end);

end