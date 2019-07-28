%% Subacz, RBE502, Homework 4, Problem 5A
function []= Problem3()

    clc, clear all, close all;

    %Parameters for the 2 link planar arm.
    I_1 = 1; I_2 = 1; m_1 = 0.05; m_2 = 0.05; l_1 = 0.30/2; l_2 = 0.30/2; 
    a_1 = 0.30; a_2 = 0.30; g = 9.8; w_1 = 4; w_2 = 3;
    
    %Parameters for the starting and ending position of the arm
    x_pos = [300/1000;-300/1000];
    y_pos = [450/1000;450/1000];

    %Use inverse kinematics to generate angle
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
    
    thetas = [startTheta1;
              startTheta2;
                endTheta1;
                endTheta2];
    
    %The total simulation time is 10 seconds, but the arm should be at
    %postion 2 with 5 seconds. 
    finalT = 5; %seconds
    tf = 10;

    %Calculate the trajectories using a quinctic method due to 6 
    % constraints in the problem space. 
    [q1,dq1,ddq1] = QuinticTrajectory(thetas(1),0,0,thetas(3),0,0,0,finalT)
    [q2,dq2,ddq2] = QuinticTrajectory(thetas(2),0,0,thetas(4),0,0,0,finalT)

    x0= [thetas(1),thetas(2),0,0]; % Initial Condition - Format:[theta1,theta2,dtheta1,dtheta2]

    % the options for ode - Optional!
    options = odeset('RelTol',1e-4,'AbsTol',[1e-4, 1e-4, 1e-4, 1e-4]);

    % Implement the computed Torque control.
    global torque
    torque=[];
    [T,X] = ode45(@(t,x)plannarArmODE(t,x),[0 tf],x0,options);

    %Plot the joint angles
    figure('Name','Theta_1+Theta_2');
    plot(T, X(:,1), 'r-');
    hold on
    plot(T, X(:,2), 'b-');
    title('Theta_1 and Theta_2')
    xlabel('time')
    ylabel('rads')
    snapnow

    % Plot joint Velocities
    figure('Name','Theta_1 & Theta_2 velocity vs time');
    plot(T, X(:,3), 'r-');
    hold on
    plot(T, X(:,4), 'b-');
    legend('theta1','theta2')
    ylabel('rad/s')
    xlabel('time')

    % Plot torques
    figure('Name','Control Input vs time');
    plot(T, torque(1,1:size(T,1)),'-' );
    hold on
    plot(T, torque(2,1:size(T,1)),'r--');
    ylabel('torque')
    xlabel('time')

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
    
    %plot a stick figure of the planar arm
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
    plot(T,x_pos(2)-link2(:,1),...
        T,y_pos(2)-link2(:,2)); 
    title('Link_2 error = Desired EE - Actual EE')
    legend('x_p error','y_p error')
    xlabel('Time')
    ylabel('Error')
    snapnow  

    % Show an animation of the end effector.
    figure; 
    h = animatedline();
    axis([-1,1,0,1])
    title('Animation of Position of end effector')
    for k = 1:length(link1)
        addpoints(h,link2(k,1),link2(k,2));
        drawnow
    end
    torque=[];

%% Functions
    function [dx] = plannarArmODE(t,x)
        %This branch point is needed because the simulation is only
        %computed for 5 seconds, but the simulation runs for 10 seconds. if
        %the simulation continues, the postions of theta diverage to + and
        %- inf. as time increases. 
        if t<=finalT
            theta_d= [feval(q1,t);
                      feval(q2,t)];
            dtheta_d =[feval(dq1,t);
                       feval(dq2,t)];
            ddtheta_d = [feval(ddq1,t);
                         feval(ddq2,t)];
        else
            theta_d= [feval(q1,finalT);
                      feval(q2,finalT)];
            dtheta_d =[0;
                       0];
            ddtheta_d = [0;
                         0];
        end

        theta= x(1:2,1);
        dtheta= x(3:4,1);
        
        global Mmat Cmat Gmat Mmatd Cmatd Gmatd
        M_11 = m_2*a_1^2 + 2*m_2*cos(x(2))*a_1*l_2 + m_1*l_1^2 +...
            m_2*l_2^2 + I_1 + I_2;
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
            
        G_11 = g*l_2*m_2*cos(x(1) + x(2)) + a_1*g*m_2*cos(x(1)) + g*l_1*m_1*cos(x(1));
        G_21 = g*l_2*m_2*cos(x(1) + x(2));
        Gmat = [G_11;
                G_21];   
        
        M_11 = m_2*a_1^2 + 2*m_2*cos(theta_d(2))*a_1*l_2 + m_1*l_1^2 + m_2*l_2^2 + I_1 + I_2;
        M_12 = m_2*l_2^2 + a_1*m_2*cos(theta_d(2))*l_2 + I_2;
        M_21 = m_2*l_2^2 + a_1*m_2*cos(theta_d(2))*l_2 + I_2;
        M_22 = m_2*l_2^2 + I_2;
        Mmatd = [M_11 M_12;
                 M_21 M_22];

        C_11 = -1*a_1*l_2*m_2*sin(theta_d(2))*2;
        C_12 =  dtheta_d(2)^2;
        C_21 = a_1*l_2*m_2*sin(theta_d(2))*dtheta_d(1);
        C_22 = 0;
        Cmatd = [C_11 C_12;
                 C_21 C_22];
        
        G_11 = g*l_2*m_2*cos(theta_d(1) + theta_d(2)) + a_1*g*m_2*cos(theta_d(1)) + g*l_1*m_1*cos(theta_d(1));
        G_21 = g*l_2*m_2*cos(theta_d(1) + theta_d(2));
        Gmatd = [G_11;
                 G_21];
        
        invM = inv(Mmat);
        invMC = invM*Cmat;
  
        tau = PDPlusFeedforwardControl(theta_d, dtheta_d, ddtheta_d,...
            theta, dtheta) 
        
        torque =[torque, tau];
        dx=zeros(4,1);
        dx(1) = x(3);
        dx(2) = x(4);
        dx(3:4) = -invMC* x(3:4) +invM*tau; % because ddot theta = -M^{-1}(C \dot Theta) + M^{-1} tau
    end

    function tau =PDPlusFeedforwardControl(theta_d, dtheta_d,ddtheta_d, theta, dtheta)
        global Mmatd Cmatd Gmatd
        %Self choosen weights, these can probably be tuned. 
        Kp=[10 0;
            0 10];
        Kv=[5 0;
            0 5];
        e=theta_d-theta; % position error
        de = dtheta_d - dtheta; % velocity error
        tau= (Kp*e + Kv*de) + Cmatd*dtheta_d + Mmatd*ddtheta_d + Gmatd;
    end
    function [qd,vd,ad] = QuinticTrajectory(q0,v0,ac0,q1,v1,ac1,t0,tf)
        % These equations comes from Spong, M. W., Hutchinson, S., & 
        % Vidyasagar, M. (2006). Robot Modeling and Control. and can be
        % found starting on page 191.

        t = linspace(t0,tf,100*(tf-t0));
        c = 1;
        M = [ 1 t0 t0^2 t0^3 t0^4 t0^5;
              0 1 2*t0 3*t0^2 4*t0^3 5*t0^4;
              0 0 2 6*t0 12*t0^2 20*t0^3;
              1 tf tf^2 tf^3 tf^4 tf^5;
              0 1 2*tf 3*tf^2 4*tf^3 5*tf^4;
              0 0 2 6*tf 12*tf^2 20*tf^3];
        
        b=[q0; v0; ac0; q1; v1; ac1];
        a = inv(M)*b;
        
        syms t
        qd = matlabFunction(a(1).*c + a(2).*t +a(3).*t.^2 + a(4).*t.^3 +a(5).*t.^4 + a(6).*t.^5);
        vd = matlabFunction(a(2).*c +2*a(3).*t +3*a(4).*t.^2 +4*a(5).*t.^3 +5*a(6).*t.^4);
        ad = matlabFunction(2*a(3).*c + 6*a(4).*t +12*a(5).*t.^2 +20*a(6).*t.^3);
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

    disp('Finish.');

end
