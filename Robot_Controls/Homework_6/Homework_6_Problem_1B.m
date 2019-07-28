%% Subacz, RBE502, Homework 6, Problem 1B
% Note: The equations on page 509 of the book are placed in the incorrect
% order. The quastions should be swithed.

function []=Figure_11_15()
    clc, clear all, close all
    %Parameters - Pages found on page 509 in Siciliano, Book of Robotics
    Tf = 100; %Run time
    w_d = 1/15; % desired steering velocity 
    R_1 = 3; %radius 1
    R_2 = 3; %radius 2
    d_r = 0.7; % damping ration      
    n_f = 1; % natural frequency 
    k_1=2*d_r*n_f; % controller gain
    k_3=k_1; % controller gain
       
    Start_Pos(1,:)=[0, 3, 0];% Initial starting position - format [x, y, theta]
    End_Pos(1,:)=[1, 0, 0]; %Initial ending position
    Error = End_Pos-Start_Pos; % Initial error position
    x0=Error(1,:); % Initial ode45  input
    [T,X] = ode45(@(t,x)ODE45Controller(t,x),[0 Tf],x0);

    function [dx] = ODE45Controller(t,x)
        %Calculate velocity and gain     
        v_d = sqrt((R_2*w_d*cos(t*w_d))^2 + (2*R_1*w_d*cos(2*t*w_d))^2); % linear velocity
        k_2=(n_f^2-w_d^2)/v_d; % controller gain  

        %Calculate the closed loop linearized dynamics - page 506
        %Branch controller to avoid div/0 error!
        if x(3) ~= 0
            dx = [x(2)*(w_d-(-k_2 * v_d * (sin(x(3))/x(3)) * x(2) - k_3*x(3))) - k_1*x(1);
                v_d*sin(x(3)) - x(1)*(w_d - (-k_2 * v_d * (sin(x(3))/x(3)) * x(2) - k_3*x(3)));
                -k_2*v_d*sin(x(3))/x(3)*x(2) - k_3];
        else           
            dx = [x(2)*w_d - k_1*x(1);
                         - x(1)*(w_d);
                                 k_3];
        end
    end

    %Error vector format [x,y,theta]
    Error=[X(:,1), X(:,2), X(:,3)];
    
    %Calcalate the desired end position
    x_End_Pos = 3 + 3*sin((1/15)*T);
    y_End_Pos = -3 + 3*sin((2/15)*T);
    t_zeros = zeros(length(x_End_Pos),1);
    End_Pos=[x_End_Pos, y_End_Pos, t_zeros];
    
    %Calculate the position of robot
    Start_Pos=End_Pos - Error; 
    
    %Plot trajectories of controller
    figure
    plot(Start_Pos(:,1), Start_Pos(:,2));
    hold on
    plot(End_Pos(:,1), End_Pos(:,2));
    title('Circular Trajectory')
    xlabel('m')
    ylabel('m')
    legend('Controller Trajectory' ,'Calculated Trajectory')
    
    % Plot the norm error of the error
    figure
    norm_error = sqrt(Error(:,1).^2 + Error(:,2).^2);
    plot(norm_error);
    title('Norm of Error')
    xlabel('m')
    ylabel('m')
end