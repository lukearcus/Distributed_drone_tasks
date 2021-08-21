function result = qp_grad(P,q,A_ineq,b_ineq,A_eq, b_eq, l_bound, u_bound,init)
% stepsize = 0.1;
% Grad = P*init + q;
% new = init - stepsize*Grad; 
% opts = optimoptions('lsqlin');
% opts.Algorithm = 'active-set';
% opts.Display = 'none';
%  opts.Diagnostics = 'off'; %projection doesn't work since solves a QP again :(
% result = lsqlin(eye(length(new)),new,A_ineq,b_ineq,A_eq,b_eq,l_bound,u_bound,init,opts);

result = init;
max_steps = 100;
start = 0.1;
if isempty(q)
    q = zeros(size(P,1),1);
end
Grad = P*init + q;
thresh = 1e-3;

if isempty(A_eq)
    Aq = [];
else
    Aq = A_eq;
end
for i = 1:size(A_ineq,1)
    if (A_ineq(i,:)*init == b_ineq(i,:))
        Aq = [Aq;A_ineq(i,:)];
    end
end

for i = 1:size(l_bound,1)
    row = zeros(1,size(l_bound,1));
    if (init(i,:) == l_bound(i,:))
        row(1,i) = -1;
        Aq = [Aq;row];
    end
end

for i = 1:size(u_bound,1)
   row = zeros(1,size(u_bound,1));
   if (init(i,:) == u_bound(i,:))
        row(1,i) = 1;
        Aq = [Aq;row];
   end
end
fin = false;
while ~fin
    if isempty(Aq)
        d = -Grad;
    else
        Ap = Aq'*inv(Aq*Aq')*Aq;
        P = eye(size(Ap)) - Ap;
        d = -P*Grad;
    end
    
    if max(d ~= 0)
        step = start;
        x = init;
        for i = 1:max_steps
            test = x + step*d;
            if ~isempty(A_ineq)
                 check = max(A_ineq*test > b_ineq);
            else
                check = false;
            end
            if ~isempty(A_eq)
                soft_eq = max(A_eq*test > b_eq + thresh) | max(A_eq*test < b_eq - thresh);
                check = check | soft_eq;
            end
            if ~isempty(l_bound)
                check = check | max(test < l_bound-thresh);
            end
            if ~isempty(u_bound)
                check = check | max(test > u_bound+thresh);
            end
            if (check)
                break;
            else
                step = step+start;
            end
        end
        alpha1 = step;
        minimum = 0.5*init'*P*init + q'*init;
        steps = linspace(0,alpha1,max_steps);
        steps(1) = [];
        steps = [steps,alpha1];
        for i = 1:max_steps
            test = x + steps(i)*d;
            test_val = 0.5*test'*P*test + q'*test;
            if (test_val < minimum)
                result = test;
                minimum =  test_val;
            end
        end
        
        fin = true;
    else
        if isempty(Aq)
            fin = true;
        else
            lambda = -inv(Aq*Aq')*Aq*Grad;
            lambda_ineq = lambda(size(A_eq,1)+1:end);
            if min(lambda_ineq < 0)
                [~,loc] = min(lambda_ineq);
                Aq(loc+size(A_eq),:) = [];
            else
                result = init;
                fin = true;
            end
        end
    end
end

end