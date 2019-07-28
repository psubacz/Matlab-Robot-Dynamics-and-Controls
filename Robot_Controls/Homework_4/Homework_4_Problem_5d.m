%% Subacz, RBE502, Homework 4, Problem 5D
function []= Problem5D()

    clc, clear all, close all;

    %Parameters for the 2 link planar arm.
    I_1 = 1; I_2 = 1; m_1 = 0.05; m_2 = 0.05; l_1 = 0.30/2; l_2 = 0.30/2; 
    a_1 = 0.30; a_2 = 0.30; g = 9.8;
    
    %Parameters for the starting and ending position of the arm
    x_pos = [300/1000;-300/1000];
    y_pos = [450/1000;450/1000];

    %Use inverse kinematics to generate angles	
    theta2_1 = atan2(sqrt(1-(((x_pos(1)^2 + y_pos(1)^2 - a_1^2 - a_2^2)...
        / (2*a_1*a_2))^2)),((x_pos(1)^2 + y_pos(1)^2 - a_1^2 - a_2^2)...
        / (2*a_1*a_2)));
    t1 = atan2(y_pos(1),x_pos(1)) - atan2(a_2*sin(theta2_1),...
        a_1+a_2*cos(theta2_1));
    thetsa =[t1;
            theta2_1];
    
    prevTheta = thetsa;
    dPrevTheta = [0;0];     
    ddPrevTheta = dPrevTheta;
    
    %The total simulation time is 10 seconds, but the arm should be at
    %postion 2 with 5 seconds. 
    finalT = 5; %seconds
    tf = 10;
    prevT = 0;
    
    x0= [thetsa(1),thetsa(2),0,0]; % Initial Condition - Format:[theta1,theta2,dtheta1,dtheta2]

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
    set(legend1,...
    'Position',[0.7 0.3 0.2 0.15]);
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
    
    x_pos = [300/1000;-300/1000];
    y_pos = [450/1000;450/1000];
    figure; 
    plot(T,(300 - 120*T)-link2(:,1),...
         T,y_pos(2)-link2(:,2)); 
    title('Link_2 error = Desired EE - Actual EE')
    legend('x_p error','y_p error')
    xlabel('Time')
    ylabel('Error')
    snapnow

%% Functions
    function [dx] = plannarArmODE(t,x)
        %This branch point is needed because the simulation is only
        %computed for 5 seconds, but the simulation runs for 10 seconds. if
        %the simulation continues, the postions of theta diverage to + and
        %- inf. as time increases. 
        
        %THE VARIABLE t DOESNT WORK HOW YOU THINK IT WOULD!
        
        if t<finalT && t<=finalT && t> prevT           
            %Equation for a line to follow
            x_pos = (300 - 120*t)/1000;
            y_pos = 450/1000;
            [theta_d,dtheta_d,ddtheta_d] = LineTrajectory(x_pos,y_pos,t,prevT,prevTheta,dPrevTheta);
            prevTheta = theta_d;
            dPrevTheta = dtheta_d;
            ddPrevTheta = ddtheta_d;
            
        elseif t>finalT
            %end
            x_pos = -300/1000;
            y_pos = 450/1000;
            [theta_d,~,~] = LineTrajectory(x_pos,y_pos,t,prevT,prevTheta,dPrevTheta);
            theta_d = theta_d;
            dtheta_d =[0;
                       0];
            ddtheta_d = [0;
                         0];     
        else
            %Log  old values          
            theta_d = prevTheta;
            dtheta_d = dPrevTheta;
            ddtheta_d = ddPrevTheta;
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
            theta, dtheta);
        
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

    function [ntheta,ndtheta_d,nddtheta_d] = LineTrajectory(x_pos,y_pos,t,prevT,prevTheta,dPrevTheta)
        
        %Use inverse kinematics to generate angles	
        theta1_1 = atan2(sqrt(1-(((x_pos^2 + y_pos^2 - a_1^2 - a_2^2)...
            / (2*a_1*a_2))^2)),((x_pos^2 + y_pos^2 - a_1^2 - a_2^2)...
            / (2*a_1*a_2)));
        theta1_2 = atan2(-sqrt(1-(((x_pos^2 + y_pos^2 - a_1^2 - a_2^2)...
            / (2*a_1*a_2))^2)),((x_pos^2 + y_pos^2 - a_1^2 - a_2^2)...
            / (2*a_1*a_2)));
        theta1 = atan2(y_pos,x_pos) - atan2(a_2*sin(theta1_1),...
            a_1+a_2*cos(theta1_1));
        theta2 = atan2(y_pos,x_pos) - atan2(a_2*sin(theta1_2),...
            a_1+a_2*cos(theta1_2));
        
        ntheta =[theta1;theta1_1];
        
        %Avoid a divid by zero error
        if (t-prevT)==0
            ndtheta_d = [0;
                         0];
            nddtheta_d = [0;
                          0];
        else
            %Calculate the velcities 
            ndtheta_d = [((ntheta(1)-prevTheta(1))/(t-prevT));
                         ((ntheta(2)-prevTheta(2))/(t-prevT))];
            %Calculate the acceleration
            nddtheta_d = [((ndtheta_d(1)-dPrevTheta(1))/(t-prevT));
                          ((ndtheta_d(2)-dPrevTheta(2))/(t-prevT))];
        end

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
