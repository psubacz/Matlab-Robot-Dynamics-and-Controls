function [] = PD_set_control_Law(x_pos,y_pos)
    clc,clear all,close all;
    %Takes a desired x-y coordinate positions and computes the inverse
    %kinematics of the angles. Inputs are expected to be in m
    x_pos = 0.5
    y_pos = 0.5
    I_1 = 1;
    I_2 = 1;
    m_1=0.5; %mass
    m_2=0.5; 
    a_1 = 0.3;
    a_2 = 0.3;
    l_1 = 0.3/2; %link length
    l_2 = 0.3/2;
    g = 9.8

    %Solve for inverse kinematics
    l_1 = l_1+a_1;
    l_2 = l_2+a_2;
    temp_1 = sqrt(1- ((x_pos^2 + y_pos^2 - l_1^2 - l_2^2)/(2*l_1*l_2))^2);
    temp_2 = (x_pos^2 + y_pos^2 - l_1^2 - l_2^2)/(2*l_1*l_2);
    theta_2 = atan2(temp_1,temp_2)
    theta_1 = atan2(y_pos,x_pos) - atan2(l_2*sin(theta_2),l_1+l_2*cos(theta_2))

    x0= [-0.5,0.2,0.1,0.1]; % Initial Condition - Format:[theta1,theta2,dtheta1,dtheta2]

    tf=10;

    %% Solve the closed-loop system nonlinear differential equation (PlanarArmODE) via ode45
    %%ode45 solves the differential equation and returns X with respect to T.
    global torque
    torque=[];
    [T,X] = ode45(@(t,x)planarArmODE(t,x,theta_2,theta_1),[0 tf],x0);
    
    %% Plot Data
    figure('Name','Theta_1 under PD SetPoint Control');
    plot(T, X(:,1),'r-');
    hold on
    snapnow

    figure('Name','Theta_2 under PD SetPoint Control');
    plot(T, X(:,2),'r--');
    hold on
    snapnow

    figure('Name','Input_PD control');
    plot(T, torque(1,1:size(T,1)),'-' );
    hold on
    plot(T, torque(2,1:size(T,1)),'r--');
    snapnow

    torque=[];
    disp('Finish.');
%% Definging Functions

    function dx = planarArmODE(t,x,theta_2,theta_1)
        theta_d = [theta_1; theta_2];    % Desired Set-Point Position
        dtheta_d=[0;0]; % Desired velocity (Derivative of theta_d)
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
end