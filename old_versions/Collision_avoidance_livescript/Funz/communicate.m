function [A_ineq,l_ineq] = communicate(i,x,N,N_j,M,H,Hw,delta,nu)

% placeholders
A_ineq = zeros(N*(M-1),(N+1)*nu*M);
l_ineq = zeros(N*(M-1),1);

x_bar = x(:,i);

for j = 1:(M-1)
    x_bar = [x_bar;x(:,N_j(j))];
end

delta_x_bar = H * x_bar;

for k = 2:N+1
    
    eta_M_k = zeros((M-1),nu*(M-1)); % placeholder
    delta_x_k = delta_x_bar((k-1)*nu*(M-1)+1:k*nu*(M-1)); % get k-th delta_x
    
    for m = 1:(M-1)
        
        x_bar_norm = norm(delta_x_k((m-1)*nu+1:m*nu)); % get the 2 norm of x_bar
        eta_ij_k = delta_x_k((m-1)*nu+1:m*nu)' * 1/x_bar_norm; % formulate eta ij k
        eta_M_k(m,(m-1)*nu+1:(m-1)*nu+nu) = eta_ij_k; % populate matrix eta_M
        
        l_ineq((k-2)*2+m) = delta + eta_ij_k * delta_x_k((m-1)*nu+1:m*nu) - x_bar_norm; % fill in l_ij
        
    end
    
    A_ineq( (k-2)*(M-1)+1 : (k-1)*(M-1) , :) = eta_M_k * Hw((k-1)*(M-1)*nu+1:(k)*(M-1)*nu,:);
    
end


end
