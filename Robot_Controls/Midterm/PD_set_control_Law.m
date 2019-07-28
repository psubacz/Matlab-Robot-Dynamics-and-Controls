%% Problem 4 - G,H
function [] = PD_set_control_Law(x_pos,y_pos,z_pos)
    clc,clear all,close all;
    %Takes a desired x-y coordinate positions and computes the inverse
    %kinematics of the angles. Inputs are expected to be in m
    
    x_pos = [.3,0]
    y_pos = [0,.3]
    z_pos = [.6,.6]
    a2 = 0.3;
    a3 = 0.3;
    %Solve for inverse kinematics - Starting Position
	r = sqrt(x_pos(1)^2 + y_pos(1)^2)
    startTheta1 = atan2(y_pos(1),x_pos(1))
    startTheta3 = acos((r^2 + x_pos(1)^2 - (a2^2 + a3^2))/(2*a2*a3))
    startTheta2 = atan2(x_pos(1),(z_pos(1) - atan2(a3*sin(startTheta3), (a2 + a3*cos(startTheta3)))))
     
    %Solve for inverse kinematics - Ending Position
	r = sqrt(x_pos(2)^2 + y_pos(2)^2)
    endTheta1 = atan2(y_pos(2),x_pos(2))
    endTheta3 = acos((r^2 + x_pos(2)^2 - (a2^2 + a3^2))/(2*a2*a3))
    endTheta2 = atan2(x_pos(2),(z_pos(2) - atan2(a3*sin(endTheta3), (a2 + a3*cos(endTheta3)))))
     
    %Set up the intial conditions using the following format
    % Initial Condition - Format:[theta1,theta2,dtheta1,dtheta2]
    x0= [startTheta1,startTheta2,startTheta3,0,0,0];
    tf=10;

    %% Solve the closed-loop system nonlinear differential equation (PlanarArmODE) via ode45
    %%ode45 solves the differential equation and returns X with respect to T.
    global torque
    torque=[];
    [T,X] = ode45(@(t,x)planarArmODE(t,x,endTheta1,endTheta2,endTheta3),[0 tf],x0);
    
    %% Plot Data
%     figure('Name','Theta_1 & Theta_2 under PD SetPoint Control');
%     subplot(2,1,1)
    plot(T, rad2deg(X(:,1)),'b-',T,rad2deg(X(:,2)),'g-',T,rad2deg(X(:,3)),'r-');
    title('Joint Angles versus time')
    ylabel('Deg')
    xlabel('Time')
    legend1 = legend('Joint Angle 1'...
    ,'Joint Angle 2'...
    ,'Joint Angle 3')
    snapnow
    
    plot(T,X(:,4),'b-',T,X(:,5),'g-',T,X(:,6),'r-');
    title('Joint Velocity versus time')
    ylabel('Deg')
    xlabel('Time')
    legend1 = legend('Joint Velocity 1'...
    ,'Joint Velocity 2'...
    ,'Joint Velocity 3')
    snapnow

    figure('Name','Input_PD control');
    subplot(3,1,1)
    plot(T, torque(1,1:size(T,1)),'b-');
    xlabel('Time')
    ylabel('Control Input')
    subplot(3,1,2)
    plot(T, torque(2,1:size(T,1)),'g-');
    xlabel('Time')
    ylabel('Control Input')
    subplot(3,1,3)
    plot(T, torque(3,1:size(T,1)),'r-');
    xlabel('Time')
    ylabel('Control Input')
    legend1 = legend('Control Input 1'...
    ,'Control Input 2'...
    ,'Control Input 3')
    snapnow  
    figure;


    %For the theta values returned form the solved equation, solve for the
    %foward kinematics to get the tip position as well as the link one
    %position
    link1 = zeros(length(T(:,1)),3);
    link2 = zeros(length(T(:,1)),3);
    link3 = zeros(length(T(:,1)),3);
    for i = 1:length(X(:,1))
        tipPostion = getRobot_Tip_Position(l1,l2,l3,X(i,1),X(i,2),X(i,3));
        link1(i,:) = [tipPostion(1,1), tipPostion(2,1),tipPostion(3,1)];
        link2(i,:) = [tipPostion(1,2), tipPostion(2,2),tipPostion(3,2)];
        link3(i,:) = [tipPostion(1,3), tipPostion(2,3),tipPostion(3,3)];
    end

  
    plot3(link1(:,1),link1(:,2),link1(:,3),...%Plot the location of link 1
         link2(:,1),link2(:,2),link2(:,3),...%Plot the location of link 2
         link3(:,1),link3(:,2),link3(:,3),...%Plot the location of link 3
         link2(1,1),link2(1,2),link2(1,3),'r-o',...%Plot the starting location of link 2
         link2(end,1),link2(end,2),link2(end,3),'b-o',...%plot the ending location of link 2
         link3(1,1),link3(1,2),link3(1,3),'r-o',...%plot the starting location of the end effector
         link3(end,1),link3(end,2),link3(end,3),'b-o',...%plot the ending location fo the end effoctor
         0,0,0);% normalize the plot to coord 0,0
    title('Link Trajectories.')
    legend1 = legend('Link_2 trajectory'...
        ,'Link_3 Trajectory'...
        ,'Link_3 Start'...
        ,'Link_3 Fnd'...
        ,'Link_2 Start'...
        ,'Link_2 End'...
        ,'Home Frame');
    
    figure;
    
    plot3([0,link1(1,1),link2(1,1),link3(1,1)],...
        [0,link1(1,2),link2(1,2),link3(1,2)],...
        [0,link1(1,3),link2(1,3),link3(1,3)],...
        'ro','markersize',10,'markerfacecolor','g');
    title('Planar Arm Stick Model - Starting Position')
    hold on
    rotate3d on
    grid on
    xlabel('x');
    ylabel('y');
    zlabel('z');
    lines = plot3([0,link1(1,1),link2(1,1),link3(1,1)],...
        [0,link1(1,2),link2(1,2),link3(1,2)],...
        [0,link1(1,3),link2(1,3),link3(1,3)],...
        'black','Linewidth',4);
    legend('Joints')
    snapnow
    
    figure;
    
    plot3([0,link1(end,1),link2(end,1),link3(end,1)],...
        [0,link1(end,2),link2(end,2),link3(end,2)],...
        [0,link1(end,3),link2(end,3),link3(end,3)],...
        'ro','markersize',10,'markerfacecolor','g');
    title('Planar Arm Stick Model - Ending Position')
    hold on
    rotate3d on
    grid on
    xlabel('x');
    ylabel('y');
    zlabel('z');
    lines = plot3([0,link1(end,1),link2(end,1),link3(end,1)],...
        [0,link1(end,2),link2(end,2),link3(end,2)],...
        [0,link1(end,3),link2(end,3),link3(end,3)],...
        'black','Linewidth',4);
    legend('Joints')
    snapnow
    
    figure; 
    plot(T,(x_pos(2)-link2(:,1)),...
        T,(y_pos(2)-link2(:,2)),...
        T,(z_pos(2)-link2(:,2))); 
    title('End Effector Error = Desired EE - Actual EE')
    legend('x_p error','y_p error','z_p error')
    xlabel('Time')
    ylabel('Error')
    snapnow  
    
    torque=[];
    disp('Finish.');
    
    function dx = planarArmODE(t,x,theta_1,theta_2,theta_3)
        theta_d = [theta_1; theta_2;theta_3];% Set-Point Position
        dtheta_d=[0;0;0];   % velocity (Derivative of theta_d)
        ddtheta_d=[0;0;0];
        theta= x(1:3,1);
        dtheta= x(4:6,1);
        l1 = 0.3;
        l2 = 0.3;
        l3 = 0.3;
        m1 = 0.5;
        m2 = 0.5;
        m3 = 0.5;
        g = 9.8;
        
        global Mmat Cmat Gmat

 
        Mmat =[ (l2^2*m2)/2 + (l2^2*m3)/2 + (l3^2*m3)/2 + (l2^2*m2*cos(2*theta(2)))/2 + (l2^2*m3*cos(2*theta(2)))/2 + (l3^2*m3*cos(2*theta(2) + 2*theta(3)))/2 + l2*l3*m3*cos(2*theta(2) + theta(3)) + l2*l3*m3*cos(theta(3)),0,0;
                0, l2^2*m2 + l2^2*m3 + l3^2*m3 + 2*l2*l3*m3*cos(theta(3)), l3*m3*(l3 + l2*cos(theta(3)));
                0,                          l3*m3*(l3 + l2*cos(theta(3))), l3^2*m3];

        Cmat = [ -dtheta(1)*(dtheta(2)*l3^2*m3*sin(2*theta(2) + 2*theta(3)) + dtheta(3)*l3^2*m3*sin(2*theta(2) + 2*theta(3)) + dtheta(2)*l2^2*m2*sin(2*theta(2)) + dtheta(2)*l2^2*m3*sin(2*theta(2)) + 2*dtheta(2)*l2*l3*m3*sin(2*theta(2) + theta(3)) + dtheta(3)*l2*l3*m3*sin(2*theta(2) + theta(3)) + dtheta(3)*l2*l3*m3*sin(theta(3)));
                                             (dtheta(1)^2*l2^2*m2*sin(2*theta(2)))/2 + (dtheta(1)^2*l2^2*m3*sin(2*theta(2)))/2 + (dtheta(1)^2*l3^2*m3*sin(2*theta(2) + 2*theta(3)))/2 + dtheta(1)^2*l2*l3*m3*sin(2*theta(2) + theta(3)) - dtheta(3)^2*l2*l3*m3*sin(theta(3)) - 2*dtheta(2)*dtheta(3)*l2*l3*m3*sin(theta(3));
                                                                                                                               (dtheta(1)^2*l3^2*m3*sin(2*theta(2) + 2*theta(3)))/2 + (dtheta(1)^2*l2*l3*m3*sin(2*theta(2) + theta(3)))/2 + (dtheta(1)^2*l2*l3*m3*sin(theta(3)))/2 + dtheta(2)^2*l2*l3*m3*sin(theta(3))];
        
        invM = inv(Mmat);
        invMC = invM*Cmat;

        tau = PDControl(theta_d,dtheta_d,ddtheta_d,theta,dtheta);
        
        torque =[torque, tau];
        dx=zeros(6,1);
        dx(1) = x(4); %dtheta1
        dx(2) = x(5); %dtheta2
        dx(3) = x(6); %dtheta1
        dx(4:6) = -invMC.'*x(4:6) + invM*tau; % because ddot theta = -M^{-1}(C \dot Theta) + M^{-1} tau
    end

    function tau = PDControl(theta_d,dtheta_d,~,theta,dtheta)
        Kp=[20,0,0;
            0,15,0;
            0,0,30];
        Kv=Kp;
        e=theta_d-theta; % position error
        de = dtheta_d - dtheta; % velocity error
        tau = Kp*e + Kv*de;
    end

    function [ RobotTipPosition ] = getRobot_Tip_Position(l_1,l_2,l_3,theta1,theta2,theta3)
        % Calculates forward kinematics, and returns the end effector position     
        theta = [ theta1+pi/2; theta2;theta3];
        d =     [    l_1;       0;    0];
        a =     [    0;      l_2; l_3]; %cm
        alpha = [    pi/2;        0;   0];

        dhTable = [theta(:),d(:),a(:),alpha(:)];
        %Create a dh table
        transformMatrix = Transform_Matrix(dhTable);
        %Calculate the forward kinematics of links 1,2,3
        partialforwardKinematics = Partial_Forward_Transforms(transformMatrix);
        %Get the tip position for links 1,2,3
        RobotTipPosition = [partialforwardKinematics(:,4,1),partialforwardKinematics(:,4,2),partialforwardKinematics(:,4,3)];
    end 

end

% 
%There may be some computational wierdness with the stick figures being
%slightly off, This is due to how matlab computes angles and is a rounding
%error. 
%


