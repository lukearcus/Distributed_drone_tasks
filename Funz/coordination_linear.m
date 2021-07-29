function qc = coordination_linear(i,lambda,lambda_to_j,rho,N_j,M,x,V)


qc = -0.5 * lambda - rho/2 * V * x(:,i);


for j = 1:(M-1)
    
    qc = [qc;( -0.5 * lambda_to_j(:,j) - rho/2 * V * x(:,N_j(j)))];
    
end

end