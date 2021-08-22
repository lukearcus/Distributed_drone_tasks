function [A_ineq,l_ineq] = communicate(x,N,N_j,H,Hw,delta,nu,i)

% placeholders
A_ineq = zeros(N*nnz(N_j),(N+1)*nu*(nnz(N_j)+1));
l_ineq = zeros(N*nnz(N_j),1);

x_bar = x(:,i);
nonzero_N_j = nonzeros(N_j);
for j = 1:nnz(N_j)
    x_bar = [x_bar;x(:,nonzero_N_j(j))];
end

delta_x_bar = H * x_bar;

for k = 2:N+1
    
    eta_M_k = zeros(nnz(N_j),nu*nnz(N_j)); % placeholder
    delta_x_k = delta_x_bar((k-1)*nu*nnz(N_j)+1:k*nu*nnz(N_j)); % get k-th delta_x
    
    for m = 1:nnz(N_j)
        
        x_bar_norm = norm(delta_x_k((m-1)*nu+1:m*nu)); % get the 2 norm of x_bar
        eta_ij_k = delta_x_k((m-1)*nu+1:m*nu)' * 1/x_bar_norm; % formulate eta ij k
        eta_M_k(m,(m-1)*nu+1:(m-1)*nu+nu) = eta_ij_k; % populate matrix eta_M
        
        l_ineq((k-2)*nnz(N_j)+m) = delta + eta_ij_k * delta_x_k((m-1)*nu+1:m*nu) - x_bar_norm; % fill in l_ij
        
    end
    
    A_ineq( (k-2)*nnz(N_j)+1 : (k-1)*nnz(N_j) , :) = eta_M_k * Hw((k-1)*nnz(N_j)*nu+1:(k)*nnz(N_j)*nu,:);
    
end

% A_ineq(isnan(A_ineq)) = 0;
% l_ineq(isnan(l_ineq)) = 0;

end
