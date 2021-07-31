% *****************************************************************************
% *                                                                           *
% *		          Trajectory visualiser - OSQP	                              *
% *				Luke Rickard with thankst to Aren Karapetyan    		      *
% *	     Plots the simulated trajectories of three agents in 3D               *
% *                                                                           *
% *****************************************************************************

function visualise_drones_3d (r,x,N,T)

M = size(x,2);
plot_var = cell(M,3);
nx = size(x,1)/(N+1);

figure;


pause on

 
for k = 1:(N+1)
    hold off
    for i = 1:M
      
        px(i) = plot3(r(i,1),r(i,2),r(i,3),'x','MarkerSize',13);
        hold on
    end
    
    % initial point
    for i = 1:M
        po(i) = plot3(x(1,i),x(2,i),x(3,i),'o','MarkerSize',13,'Color',px(i).Color);
    end

    for n = 1:3
        for i = 1:M
            plot_var{i,n} = [plot_var{i,n}, x(n+(k-1)*nx,i)];   
        end
    end

    for i = 1:M
        p(i) = plot3(plot_var{i,1},plot_var{i,2},plot_var{i,3},'LineWidth',2,'Color',px(i).Color);
    end
    hold on
    
    for i = 1:M
        ps(i) = plot3(plot_var{i,1}(end),plot_var{i,2}(end),plot_var{i,3}(end),'*','MarkerSize',10,'Color',px(i).Color);
    end

    hold off
    
    legend('off')

    pause(T)
    
end


xlabel('x [m]') 
ylabel('y [m]') 
zlabel('z [m]')
labels = [];
for i = 1:M
    labels = [labels, "Agent "+i];
end
legend(p,labels);

end