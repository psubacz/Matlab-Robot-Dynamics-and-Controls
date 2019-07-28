%% Subacz, RBE502, Homework 6, Problem 1C

function []=Figure_11_16()
    clc, clear all, close all
    %Parameters - Pages found on page 509 in Siciliano, Book of Robotics
    Tf=16; %Run time
    b = 0.75;
    k_1=2; % controller gain
    k_2=2; % controller gain
    
    Start_Pos=[1, 0, 0]; % Initial starting position - format [x, y, theta]
    End_Pos=[1, 0.5, 0]; % Initial ending position
    Error = End_Pos-Start_Pos; % Initial error position
    x0=Error(1,:); % Initial ode45  input
    
    [T,X] = ode45(@(t,x)ODE45Controller(t,x),[0 Tf],x0);

    function [dx ] = ODE45Controller(t,x)
        %Calculate the trajectory to follow
        [x_1,y_2,dx_1,dy_2] = SquareTrajectory(t);

        %Calculate the closed loop linearized dynamics - page 505
        dx = [dx_1 + k_1*(x_1 - x(1));
              dy_2 + k_2*(y_2 - x(2));
             (dx_1 + k_1*(x_1 - x(1))*cos(x(3)) - dy_2 + k_2*(y_2 - x(2))*sin(x(3)))/b];
    end

    %Error vector format [x,y,theta]
    Error=[X(:,1), X(:,2), X(:,3)];
    
    for i=1:size(T)
        %Calculate the trajectory to follow
        [x_1,y_1,~,~] = SquareTrajectory(T(i));
        End_Pos(i,:)=[x_1, y_1, 0];
    end

    figure
    plot(Error(:,1), Error(:,2));
    hold on
    plot(End_Pos(:,1), End_Pos(:,2));
    hold off
    title('Square Trajectory')
    xlabel('m')
    ylabel('m')
    
    %Calculate the positions/velocity of the model
    x_Pos=X(:,1); 
    y_Pos=X(:,2);
    th=X(:,3); 
    d_x = x_Pos(2:end) - x_Pos(1:end-1);
    d_y = y_Pos(2:end) - y_Pos(1:end-1);
    d_th = th(2:end) - th(1:end-1);
    d_t = T(2:end) - T(1:end-1);

    %Calculate driving velocity
    vd_x = d_x./d_t;
    vd_y = d_y./d_t;
    v_s = sqrt(vd_x.^2 + vd_y.^2);

    %calcalate steering velocity
    v_th = d_th./d_t; 
    
    figure
    plot(T,[0; v_s])
    title('Driving Velocity')
    xlabel('seconds')
    
    figure
    plot(T,[0; v_th])
    title('Steering Velocity')
    xlabel('seconds')
    hold off

    function [x_1,y_1,dx_1,dy_2]  = SquareTrajectory(t)
        %control system to follow a box
        if t<=4
            %first turn
            x_1 = 1 + t;
            y_1 = 0.5;
            dx_1 = 1;
            dy_2 = 0;
        elseif t>4 && t<=8
            %Second turn
            x_1 = 5;
            y_1 = 0.5 + (t-4);
            dx_1 = 0;
            dy_2 = 1;
        elseif t>8 && t<=12
            %thrid turn
            x_1 = 5 - (t-8);
            y_1 = 4.5;
            dx_1 = -1;
            dy_2 = 0;
        elseif t>12 && t<=16
            %fouth turn
            x_1=1;
            y_1 = 4.5 - (t-12);
            dx_1 = 0;
            dy_2 = -1;
        else
            x_1 = 1;
            y_1 = 0.5;
            dx_1 = 0;
            dy_2 = 0;
        end
    end
end