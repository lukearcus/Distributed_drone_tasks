clear
addpath Funz

r = [1.0,1.4,0,0,0,0; 0.6,0.4,0,0,0,0; 0,0.9,0,0,0,0; 0,0,1,0,0,0];
x_0 = [0,0.5,0,0,0,0;  0.2,0.9,0,0,0,0; 1.0,0.3,0,0,0,0; 0,0,0,0,0,0];
M = size(r,1);
comm_graph = [0 1 1 1; 1 0 1 1; 1 1 0 1; 1 1 1 0];


% M = 20;
% r = [randn(M,3),zeros(M,3)];
% x_0 = [randn(M,3),zeros(M,3)];

for i = 1:M
   drone{i} = multirotor; 
   state{i} = drone{i}.state;
   input{i} = drone{i}.control;
end

e = environment(drone{1});

t_step = 0.1;
t_end = 10;
N = t_end/t_step;



tic
a = plan_traj(x_0,r,t_step,N,0,comm_graph);
toc

x_0 = x_0';

for i = 1:M
    state{i}(1:6) = x_0(:,i);
    init_z(i) = state{i}(3);
    state_out{i} = zeros(13,N+1);
    state_out{i}(:,1) = state{i};
    deriv_out{i} = zeros(13,N);
    hist_input{i} = zeros(3,N);
end


k=1;
for t = 0:t_step:t_end-t_step

    for i = 1:M
        acc{i} = a(3*(k-1)+1:3*k,i);
        hist_input{i}(:,k) = acc{i};
        [input{i}.Thrust,input{i}.Pitch,input{i}.Roll] = acc_to_input(acc{i},state{i}(7),drone{i}.Configuration.Mass);
        sdot{i} = derivative(drone{i},state{i},input{i},e);
        simOut = ode45(@(~,x)derivative(drone{i},x,input{i},e), [t t+t_step], state{i});
        state{i} = simOut.y(:,size(simOut.y(1,:),2));
        sdot{i} = derivative(drone{i},state{i},input{i},e);
        deriv_out{i}(:,k) = sdot{i};
        deriv_out{i}(3,k) = -deriv_out{i}(3,k);
        deriv_out{i}(6,k) = -deriv_out{i}(6,k);
    end
    
    k = k+1;
    
    for i = 1:M
    state_out{i}(:,k) = state{i};
    state_out{i}(3,k) = 2*init_z(i)-state_out{i}(3,k);
    end
end

s = zeros((N+1)*3,M);
for i = 1:M
   for k = 1:N+1
      s((k-1)*3+1:(k-1)*3+3,i) = state_out{i}(1:3,k);  
   end
end
visualise_drones_3d(r,s,N,t_step);


figure
for i = 1:3
    subplot(3,1,i)
    plot(deriv_out{i}(3+i,:)')
    hold on
    plot(hist_input{i}(i,:)')
    xlabel("Acceleration");
    ylabel("timestep")
    switch i
        case 1
            title("x")
        case 2
            title("y")
        case 3
            title("z")
    end
end