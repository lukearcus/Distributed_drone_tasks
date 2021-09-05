function q = prediction_linear(lambda,lambda_from_j,w,w_from_j,rho,r,Q,N,nu,V)


q = -(rho/2 * w' * V)' + 0.5*(lambda'*V)';

sigma_j = zeros(size(q));
for j = 1:size(lambda_from_j,2)    % TODO: can we vectorise this computation?
    sigma_j = sigma_j + 0.5 * (lambda_from_j(:,j)' * V)' - (rho/2 * (w_from_j(:,j))' * V)';
end

q=q+[repmat(-Q*r, N, 1); -Q*r] + sigma_j;
q = [q;zeros(N*nu, 1)];

end