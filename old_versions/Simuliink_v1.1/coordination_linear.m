function qc = coordination_linear(lambda,lambda_to_j,rho,x,V,N_j,i)


qc = -0.5 * lambda - rho/2 * V * x(:,i);

N_j = nonzeros(N_j);
for j = 1:nnz(N_j)
    
    qc = [qc;( -0.5 * lambda_to_j(:,N_j(j)) - rho/2 * V * x(:,N_j(j)))];
    
end

end