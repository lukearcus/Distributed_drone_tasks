function q = prediction_linear(lambda,lambda_from_j,w,w_from_j,rho,r,Q,QN,N,nu,M,V)


q = -(rho/2 * w' * V)' + 0.5*(lambda'*V)';

sigma_j = 0;
for j = 1:(M-1)
    sigma_j = sigma_j + 0.5 * (lambda_from_j(:,j)' * V)' - (rho/2 * (w_from_j(:,j))' * V)';
end

q=q+[repmat(-Q*r, N, 1); -QN*r] + sigma_j;
q = [q;zeros(N*nu, 1)];

end