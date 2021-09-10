clear
addpath Funz

%% define inputs
r = [
    1.0,1.4,0,0,0,0; 
    0.6,0.4,0,0,0,0; 
    0,0.9,0,0,0,0; 
    0,0,1,0,0,0;
    ]; %Goal/ reference state

x_0 = [
    0,0.5,0,0,0,0;  
    0.2,0.9,0,0,0,0; 
    1.0,0.3,0,0,0,0; 
    0,0,0,0,0,0;
    ]; %initial state

M = size(r,1);
comm_graph = [0 1 1 1; 1 0 1 1; 1 1 0 1; 1 1 1 0]; %Communication graph

t_step = 0.1;
t_end = 10;
MPC_step = 30; %Number of steps to look ahead in plan_traj
MPC_update = 10; %how often to call plan_traj

%% setup

%setup multirotor models
for i = 1:M
   drone{i} = multirotor; 
   state{i} = drone{i}.state;
   input{i} = drone{i}.control;
end

e = environment(drone{1});


N = t_end/t_step;


a = plan_traj(x_0,r,t_step,MPC_step,0,comm_graph);


x_0 = x_0';
%sets up initial conditions
for i = 1:M
    state{i}(1:6) = x_0(:,i);
    state{i}(3) = -state{i}(3);
    state{i}(6) = -state{i}(6);
    init_z(i) = state{i}(3);
    state_out{i} = zeros(13,N+1);
    state_out{i}(:,1) = state{i};
    deriv_out{i} = zeros(13,N);
    hist_input{i} = zeros(3,N);
end

%% run simulation

x = zeros(M,6);
k=1;
MPC_k = 1;
for t = 0:t_step:t_end-t_step
    if rem(k,MPC_update) == 0
        for i=1:M
           x(i,:) = state{i}(1:6);
           x(i,3) = -x(i,3);
           x(i,6) = -x(i,6);
        end
        a = plan_traj(x,r,t_step,MPC_step,0,comm_graph); %updates planned trajectory when necessary
        MPC_k = 1;
    end
    
    for i = 1:M
        acc{i} = a(3*(MPC_k-1)+1:3*MPC_k,i);
        hist_input{i}(:,k) = acc{i};
        %Calculate control commands from 3D acceleration
        [input{i}.Thrust,input{i}.Pitch,input{i}.Roll] = acc_to_input(acc{i},state{i}(7),drone{i}.Configuration.Mass);
        
        sdot{i} = derivative(drone{i},state{i},input{i},e);
        simOut = ode45(@(~,x)derivative(drone{i},x,input{i},e), [t t+t_step], state{i}); %Simulate
        
        state{i} = simOut.y(:,size(simOut.y(1,:),2)); %update states and derivatives
        sdot{i} = derivative(drone{i},state{i},input{i},e);
        deriv_out{i}(:,k) = sdot{i};
        deriv_out{i}(3,k) = -deriv_out{i}(3,k);
        deriv_out{i}(6,k) = -deriv_out{i}(6,k);
    end
    
    MPC_k = MPC_k+1;
    k = k+1;
    
    for i = 1:M
        state_out{i}(:,k) = state{i}; %store state for afterwards
        state_out{i}(3,k) = 2*init_z(i)-state_out{i}(3,k);
    end
end

s = zeros((N+1)*3,M);
for i = 1:M
   for k = 1:N+1
      s((k-1)*3+1:(k-1)*3+3,i) = state_out{i}(1:3,k); %transform to state for visualisation
   end
end
visualise_drones_3d(r,s,N,t_step);

%% plot acceleration data and find maximum deviation
max_diff = zeros(M,1);
for k = 1:M
    figure
    for i = 1:3
        subplot(3,1,i)
        plot(deriv_out{k}(3+i,:)')
        hold on
        plot(hist_input{k}(i,:)')
        max_diff(k) = max(max_diff,max(abs(deriv_out{k}(3+i,:)'-hist_input{k}(i,:)')));
        ylabel("Acceleration");
        xlabel("timestep")
        switch i
            case 1
                title("x")
            case 2
                title("y")
            case 3
                title("z")
        end
    end
end