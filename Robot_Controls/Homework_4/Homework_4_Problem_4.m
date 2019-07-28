%% Subacz, RBE502, Homework 4, Problem 5

function []= Problem4()
clc, clear all, close all;

%parameters for the arm
%Importing the parameters from table 5.1, page 115 of kelly's book
I_1 = 0.1213; I_2 = 0.0116; m_1 = 6.5225; l_c1 = .0983; m_2 = 2.0458;
l_c2 = .0229; l_1 = 0.26; l_2 = 0.26; g = 9.81; w_1 = 4; w_2 = 3;

x0= [0,0,0,0]; % Initial Condition - Format:[theta1,theta2,dtheta1,dtheta2]

tf=10; 

%Generate trajectories to follow, Using the parameters and equations from
%chapter 5.4 of kellys book, page 128.

b_1 = pi/4; c_1 = pi/9; b_2 = pi/3; c_2 = pi/6;

global torque
torque=[];

global err

[T,X] = ode45(@(t,x)plannarArmODE(t,x),[0 tf],x0);

figure('Name','Theta_1+Theta_2');
plot(T, X(:,1), 'r-');
hold on
plot(T, X(:,2), 'b-');
title('Theta_1 and Theta_2')
xlabel('time')
ylabel('rads')
snapnow

%recalcalate the actual positions of q_1 and q_2
q_plot_1 = b_1*(1-exp(-2*(T.^3))) + c_1*(1- exp(-2*(T.^3))).*sin(w_1.*T);
q_plot_2 = b_2*(1-exp(-2*(T.^3))) + c_2*(1- exp(-2*(T.^3))).*sin(w_2.*T);

figure('Name','Error Theta_1+Theta_2 under PD SetPoint Control');
plot(T,(q_plot_1(:,1)- X(:,1)),'r-')
hold on
plot(T,(q_plot_2(:,1))- X(:,2),'b-');
title('Theta_1 and Theta_2 error')
xlabel('time')
ylabel('err')
axis([-0 10 -1 1])

figure('Name','Input_Computed Torque Control');
plot(T, torque(1,1:size(T,1)),'r-');
hold on
plot(T, torque(2,1:size(T,1)),'b-');
xlabel('time')
ylabel('torque')
title('input computed torque control')
torque=[];

%% Functions

    function [dx] = plannarArmODE(t,x)
        %Generate trajectories to follow, Using the parameters and equations from
        %chapter 5.4 of kellys book, page 128.
        
        %Calculate the positons of q_1 and q_2
        q_1 = b_1*(1-exp(-2*(t.^3))) + c_1*(1- exp(-2*(t.^3))).*sin(w_1.*t);
        q_2 = b_2*(1-exp(-2*(t.^3))) + c_2*(1- exp(-2*(t.^3))).*sin(w_2.*t);
        theta_d= [q_1; q_2];
              
        %Calculate the velocities of dq_1 and dq_2
        dq_1 = 6*b_1*t^2*exp(-2*t^3) - c_1*w_1*cos(t*w_1)*(exp(-2*t^3) - 1) + 6*c_1*t^2*exp(-2*t^3)*sin(t*w_1);;
        dq_2 = 6*b_2*t^2*exp(-2*t^3) - c_2*w_2*cos(t*w_2)*(exp(-2*t^3) - 1) + 6*c_2*t^2*exp(-2*t^3)*sin(t*w_2);;
        dtheta_d =[dq_1; dq_2];    
               
        %Calculate the acceleration of ddq_1 and ddq_2
        ddq_1 = 12*b_1*t*exp(-2*t^3) - 36*b_1*t^4*exp(-2*t^3) + c_1*w_1^2*sin(t*w_1)*(exp(-2*t^3) - 1) + 12*c_1*t*exp(-2*t^3)*sin(t*w_1) - 36*c_1*t^4*exp(-2*t^3)*sin(t*w_1) + 12*c_1*t^2*w_1*exp(-2*t^3)*cos(t*w_1);
        ddq_2 = 12*b_2*t*exp(-2*t^3) - 36*b_2*t^4*exp(-2*t^3) + c_2*w_2^2*sin(t*w_2)*(exp(-2*t^3) - 1) + 12*c_2*t*exp(-2*t^3)*sin(t*w_2) - 36*c_2*t^4*exp(-2*t^3)*sin(t*w_2) + 12*c_2*t^2*w_2*exp(-2*t^3)*cos(t*w_2);
        ddtheta_d = [ddq_1; ddq_2];
        
        theta= x(1:2,1);
        dtheta= x(3:4,1);
        
        global Mmat Cmat Gmat Mmatd Cmatd Gmatd
        a = I_1+I_2+m_1*l_c1^2+ m_2*(l_1^2+ l_c2^2);
        b = m_2*l_1*l_c2;
        d = I_2+ m_2*l_c2^2;

        Mmat = [a+2*b*cos(x(2)), d+b*cos(x(2));  d+b*cos(x(2)), d];
        Cmat = [-b*sin(x(2))*x(4), -b*sin(x(2))*(x(3)+x(4)); b*sin(x(2))*x(3),0];
        Gmat = [(m_1*l_c1 + m_2*l_1)*g*sin(x(1)) + m_2*l_c2*g*sin(x(1) + x(2)); m_2*l_c2*g*sin(x(1) + x(2))];
        
        Mmatd = [a+2*b*cos(theta_d(2)), d+b*cos(theta_d(2));  d+b*cos(theta_d(2)), d];
        Cmatd = [-b*sin(theta_d(2))*dtheta_d(2), -b*sin(theta_d(2))*(dtheta_d(1)+dtheta_d(2)); b*sin(theta_d(2))*dtheta_d(1),0];
        Gmatd = [(m_1*l_c1 + m_2*l_c2)*g*sin(theta_d(1)) + m_2*l_c2*g*sin(theta_d(1) + theta_d(2));m_2*l_c2*g*sin(theta_d(1) + theta_d(2))];
        
        invM = inv(Mmat);
        invMC = invM*Cmat;
        invMG = invM*Gmat;
        tau = FeedforwardControl(theta_d, dtheta_d, ddtheta_d, theta, dtheta); %
        torque =[torque, tau];
        dx=zeros(4,1);
        dx(1) = x(3);
        dx(2) = x(4);
        invMG = invM*Gmat;
        dx(3:4) = -invMC* x(3:4) - invMG + invM*tau;
    end

    function tau = FeedforwardControl(theta_d, dtheta_d,ddtheta_d, theta, dtheta)
        global Mmatd Cmatd Gmatd
        tau= Cmatd*dtheta_d + Mmatd*ddtheta_d + Gmatd;
    end

disp('Finish.');

end

%% Discussion
% On page 268-269 of kelly's book, a graphic on feed forward control is
% shown in an oscillatory postion. As stated in the book, feed forward
% contol has many equilibria, therefore it is possible to create a stable
% control system using feedward. However, this is counter acted by the fact
% that the equilibrium points are inflexible, meaning that a designer has
% no control over the points. The control of the equilibrium points are
% derived by the inherint nature of the controller. With a feedforward
% controller a design would need to hand tailor each dynamic model to
% support the desired equilibrium points. 

% In this example, the planar arm from chapter 5 was computed and the
% system is not in a natural equilibrium point in the derived dynamic
% model. figure 1 shows theta_2 trending to positive inf. while theta_1 is
% contant zero. In figure 2, the system instability is show with the
% erractic behavior in the calculated error positions. Lastly, figure 3
% show addition eratic behaviour and does not converage as the figures did 
% in problems 1,2, and 3.
