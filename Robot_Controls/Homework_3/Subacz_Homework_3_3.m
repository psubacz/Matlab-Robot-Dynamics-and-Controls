%% Problem 4J
function [] = PD_set_control_Law(x_pos,y_pos)
    clc,clear all,close all;
    %Takes a desired x-y coordinate positions and computes the inverse
    %kinematics of the angles. Inputs are expected to be in m
    x_pos = [.3,-.3]
    y_pos = [.45,.45]
    I_1 = 1;
    I_2 = 1;
    m_1=0.5; %mass
    m_2=0.5; 
    a_1 = 0.3;
    a_2 = 0.3;
    l_1 = 0.3/2; %link length
    l_2 = 0.3/2;
    g = 9.8;

    %Solve for inverse kinematics - Starting Position
    %There are two different values of theta due to the + or - of the sqrt
    %therefore we need to calculate two different values of theta. 
    theta2_1 = atan2(sqrt(1-(((x_pos(1)^2 + y_pos(1)^2 - a_1^2 - a_2^2)...
        / (2*a_1*a_2))^2)),((x_pos(1)^2 + y_pos(1)^2 - a_1^2 - a_2^2)...
        / (2*a_1*a_2)));
    theta2_2 = atan2(-sqrt(1-(((x_pos(1)^2 + y_pos(1)^2 - a_1^2 - a_2^2)...
        / (2*a_1*a_2))^2)),((x_pos(1)^2 + y_pos(1)^2 - a_1^2 - a_2^2)...
        / (2*a_1*a_2)));
    theta1_1 = atan2(y_pos(1),x_pos(1)) - atan2(a_2*sin(theta2_1),...
        a_1+a_2*cos(theta2_1));
    theta1_2 = atan2(y_pos(1),x_pos(1)) - atan2(a_2*sin(theta2_2),...
        a_1+a_2*cos(theta2_2));
    startTheta1 = theta1_1
    startTheta2 = theta2_1

    %Solve for inverse kinematics - Ending Position
    %There are two different values of theta due to the + or - of the sqrt
    %therefore we need to calculate two different values of theta. 
    theta2_1 = atan2(sqrt(1-(((x_pos(2)^2 + y_pos(2)^2 - a_1^2 - a_2^2)...
        / (2*a_1*a_2))^2)),((x_pos(2)^2 + y_pos(2)^2 - a_1^2 - a_2^2) /...
        (2*a_1*a_2)));
    theta2_2 = atan2(-sqrt(1-(((x_pos(2)^2 + y_pos(2)^2 - a_1^2 - a_2^2)...
        / (2*a_1*a_2))^2)),((x_pos(2)^2 + y_pos(2)^2 - a_1^2 - a_2^2) /...
        (2*a_1*a_2)));
    theta1_1 = atan2(y_pos(2),x_pos(2)) - atan2(a_2*sin(theta2_1),...
        a_1+a_2*cos(theta2_1));
    theta1_2 = atan2(y_pos(2),x_pos(2)) - atan2(a_2*sin(theta2_2),...
        a_1+a_2*cos(theta2_2));
    endTheta1 = theta1_2
    endTheta2 = theta2_2
    
    %Set up the intial conditions using the following format
    % Initial Condition - Format:[theta1,theta2,dtheta1,dtheta2]
    x0= [startTheta1,startTheta2,0,0];
    tf=10;

    %% Solve the closed-loop system nonlinear differential equation (PlanarArmODE) via ode45
    %%ode45 solves the differential equation and returns X with respect to T.
    global torque
    torque=[];
    [T,X] = ode45(@(t,x)planarArmODE(t,x,endTheta1, endTheta2),[0 tf],x0);
    
    %% Plot Data
    figure('Name','Theta_1 & Theta_2 under PD SetPoint Control');
    subplot(2,1,1)
    plot(T, rad2deg(X(:,1)),'b-');
    title('Theta_1')
    ylabel('Deg')
    xlabel('Time')
    subplot(2,1,2)
    plot(T, rad2deg(X(:,2)),'b-');
    title('Theta_2')
    ylabel('Deg')
    xlabel('Time')
    snapnow

    figure('Name','Theta_1 & Theta_2 velocity under PD SetPoint Control');
    subplot(2,1,1)
    plot(T, X(:,3),'r--');
    title('dTheta_1')
    xlabel('time')
    subplot(2,1,2)
    plot(T, X(:,4),'r--');
    title('dTheta_2')
    xlabel('time')
    snapnow

    figure('Name','Input_PD control');
    plot(T, torque(1,1:size(T,1)),'-' );
    hold on
    plot(T, torque(2,1:size(T,1)),'r--');
    xlabel('Time')
    snapnow

    %first plot side view slice.
    figure;
    subplot(2,1,1)
    hold on;

    %For the theta values returned form the solved equation, solve for the
    %foward kinematics to get the tip position as well as the link one
    %position
    for i = 1:length(X(:,1))
        tipPostion = getRobot_Tip_Position(a_1,a_2,X(i,1),X(i,2));
        link1(i,:) = [tipPostion(1,1); tipPostion(2,1)];
        link2(i,:) = [tipPostion(1,2); tipPostion(2,2)];
    end
    
    plot(link1(:,1),link1(:,2),...%Plot the location of link 1
        link2(:,1),link2(:,2),...%Plot the location of link 2
        link1(1,1),link1(1,2),'r-o',...%Plot the starting location of link 1
        link1(end,1),link1(end,2),'b-o',...%plot the ending location of link 1
        link2(1,1),link2(1,2),'r-o',...%plot the starting location of the end effector
        link2(end,1),link2(end,2),'b-o',...%plot the ending location fo the end effoctor
        0,0);% normalize the plot to coord 0,0
    title('Position of Link_1 and Link_2.')
    legend1 = legend('Link_1 trajectory'...
        ,'Link_2 Trajectory'...
        ,'Link_2 Start'...
        ,'Link_2 Fnd'...
        ,'Link_1 Start'...
        ,'Link_1 End'...
        ,'Home Frame');
    snapnow
    figure;
    
    title('Planar Arm Stick Model')
    hold on
    plot([0 link1(1,1) link2(1,1)],...
        [0 link1(1,2) link2(1,2)],...
        link2(1,1),link2(1,2),'r-o',...
        link1(1,1),link1(1,2),'b-o')
       
    title('Link_2 Final Position')
    xlabel('m')
    ylabel('m')
    legend('Links','Link_2','Link_1')
    snapnow

    figure; 
    plot(T,x_pos(2) -link2(:,1),...
        T,y_pos(2) -link2(:,2)); 
    title('Link_2 error = Desired EE - Actual EE')
    legend('x_p error','y_p error')
    xlabel('Time')
    ylabel('Error')
    snapnow  
    
    torque=[];
    disp('Finish.');

    function dx = planarArmODE(t,x,theta_1,theta_2)
        theta_d = [theta_1; theta_2];% Set-Point Position
        dtheta_d=[0;0];   % velocity (Derivative of theta_d)
        ddtheta_d=[0;0];
        theta= x(1:2,1);
        dtheta= x(3:4,1);

        global Mmat Cmat
        %write m in terms of the states
        M_11 = m_2*a_1^2 + 2*m_2*cos(x(2))*a_1*l_2 + m_1*l_1^2 + m_2*l_2^2 + I_1 + I_2;
        M_12 = m_2*l_2^2 + a_1*m_2*cos(x(2))*l_2 + I_2;
        M_21 = m_2*l_2^2 + a_1*m_2*cos(x(2))*l_2 + I_2;
        M_22 = m_2*l_2^2 + I_2;

        Mmat = [M_11 M_12
                M_21 M_22];

        C_11 = -1*a_1*l_2*m_2*sin(x(2))*2; 
        C_12 =  x(4)^2;
        C_21 = a_1*l_2*m_2*sin(x(2))*x(3);
        C_22 = 0;
        Cmat = [C_11 C_12;
                C_21 C_22];
        invM = inv(Mmat);
        invMC = invM*Cmat;

        tau = PDControl(theta_d,dtheta_d,ddtheta_d,theta,dtheta);
        
        torque =[torque, tau];
        dx=zeros(4,1);
        dx(1) = x(3); %dtheta1
        dx(2) = x(4); %dtheta2
        dx(3:4) = -invMC* x(3:4) +invM*tau; % because ddot theta = -M^{-1}(C \dot Theta) + M^{-1} tau
    end

    function tau = PDControl(theta_d,dtheta_d,~,theta,dtheta)
        Kp=10*eye(2);
        Kv=10*eye(2);
        e=theta_d-theta; % position error
        de = dtheta_d - dtheta; % velocity error
        tau = Kp*e + Kv*de;
    end

    function [ RobotTipPosition ] = getRobot_Tip_Position(l_1,l_2,theta1,theta2 )
        % Calculates forward kinematics, and returns the end effector position     
        theta = [ theta1; theta2];
        d =     [    0;       0];
        a =     [  l_1;      l_2]; %cm
        alpha = [    0;        0];
        dhTable = [theta(:),d(:),a(:),alpha(:)];
        %Create a dh table
        transformMatrix = Transform_Matrix(dhTable);
        %Calculate the forward kinematics of links 1 and 2
        partialforwardKinematics = Partial_Forward_Transforms(transformMatrix);
        %Get the tip position for links 1 and 2
        RobotTipPosition = [partialforwardKinematics(:,4,1),partialforwardKinematics(:,4,2)];
    end 
end