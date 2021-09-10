function ADMM = mediate_step(ADMM,constraints,N)

Nd = 3;
nx = size(ADMM.x,1)/(N+1);

d = [eye(Nd),zeros(Nd,nx-Nd)]; % Matrix to take only position from the state vector x (only returns pos)
posMN = kron(eye(N+1),d); %returns pos across time steps


ADMM.lambda = ADMM.lambda + ADMM.rho * (posMN * ADMM.x-ADMM.w);
for j = 1:size(constraints.N_j,2)
    ADMM.lambda_to_j(:,j) = ADMM.lambda_to_j(:,j) + ADMM.rho * (posMN*ADMM.x_bar(:,j+1) - ADMM.w_to_j(:,j));
end

end