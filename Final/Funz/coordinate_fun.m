function ADMM = coordinate_fun(ADMM,Objective, constraints)

Nd = 3;
nx = size(Objective.Q,1);
nu = size(Objective.R,1);

N = Objective.N;

d = [eye(Nd),zeros(Nd,nx-Nd)]; % Matrix to take only position from the state vector x (only returns pos)
posMN = kron(eye(N+1),d); %returns pos across time steps


rhoM = kron(speye((N+1)),ADMM.rho/2*eye(nu)); % matrix for the quadratic objective formualation
Pc = kron(eye(1+size(constraints.N_j,2)),rhoM);

H = kron(eye(N+1),repmat(d, size(constraints.N_j,2), 1));
Hw = kron(eye(N+1),repmat(eye(nu), size(constraints.N_j,2), 1));
for j = 1:size(constraints.N_j,2)
    v = zeros(size(constraints.N_j,2),1);
    v(j) = 1;
    H_M = kron(eye(N+1),kron(v, -1*d));
    H_Mw = kron(eye(N+1),kron(v, -1*eye(nu)));
    
    H =  [H, H_M];
    Hw = [Hw,H_Mw];
end
qc = coordination_linear(ADMM.lambda,ADMM.lambda_to_j,ADMM.rho,ADMM.x_bar,posMN);

% Update matrices

qp_opts = optimoptions('quadprog','Display','off','Algorithm','active-set');
init = ADMM.w;
for j = 1:size(constraints.N_j,2)
   init = [init;ADMM.w_to_j(:,j)]; 
end
[A_ineq,l_ineq] = communicate(ADMM.x_bar,Objective.N,constraints.N_j,H,Hw,constraints.delta,nu);
resc = quadprog(full(Pc),qc,-A_ineq,-l_ineq,[],[],[],[],init,qp_opts);


ADMM.w = resc(1:Nd*(N+1),1);
for j = 1:size(constraints.N_j,2)
    ADMM.w_to_j(:,j) = resc(j*Nd*(N+1)+1:(j+1)*Nd*(N+1),1);
end

end