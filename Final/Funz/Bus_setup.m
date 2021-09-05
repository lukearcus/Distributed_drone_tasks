constraints_struct = [];
for i = 1:M
    constraints_setup.r = r(i,:);
    constraints_setup.x0 = x0(i,:);
    constraints_setup.delta = delta;
    constraints_setup.N_j = zeros(1,M);
    constraints_struct = [constraints_struct;constraints_setup];
end




%% ADMM Vectors
ADMM_vec = [];
for i = 1:M
    ADMM_struct.rho = 40; %weighting of constraint violation

    lambda = zeros(Nd,1); %Lagrangian variable
    lambda_to_ = zeros(Nd,1); %used to construct lagrangians for each agent
    lambda_from_ = zeros(Nd,1);

    ADMM_struct.w = zeros(Nd*(N+1),1); %Will effectively be the proposed trajectory by one drone for another
    ADMM_struct.x = zeros(nx*(N+1),1);
    ADMM_struct.u = zeros(nu*N,1);

    w_from_ = zeros(Nd,1);
    w_to_ = zeros(Nd,1);

    ADMM_struct.lambda = repmat(lambda, N+1, 1);

    ADMM_struct.lambda_to_j = repmat(lambda_to_, N+1,M);
    ADMM_struct.lambda_from_j = repmat(lambda_from_, N+1, M);
    ADMM_struct.w_from_j = repmat(w_from_, N+1, M);
    ADMM_struct.w_to_j = repmat(w_to_, N+1, M);
    ADMM_struct.x_bar = zeros(nx*(N+1),M);
    ADMM_vec = [ADMM_vec;ADMM_struct];
end
for i =1:M
    ADMM_vec(i).x_bar = zeros(nx*(N+1),M);
end
ADMM = Simulink.Bus;


fn = fieldnames(ADMM_struct);
for k = 1:numel(fn)
   elems(k) = Simulink.BusElement;
   elems(k).Name = fn{k};
   elems(k).Dimensions = size(ADMM_struct.(fn{k}));
end

ADMM.Elements = elems;

constraints = Simulink.Bus;
fn = fieldnames(constraints_struct(1));

for k = 1:numel(fn)
   con_elems(k) = Simulink.BusElement;
   con_elems(k).Name = fn{k};
   con_elems(k).Dimensions = size(constraints_struct(1).(fn{k}));
end

constraints.Elements = con_elems;