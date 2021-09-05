function qc = coordination_linear(lambda,lambda_to_j,rho,x,V)


qc = -0.5 * lambda - rho/2 * V * x(:,1);


for j = 1:size(lambda_to_j,2)
    
    qc = [qc;( -0.5 * lambda_to_j(:,j) - rho/2 * V * x(:,j+1))];
    
end

end