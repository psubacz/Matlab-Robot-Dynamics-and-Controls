
function []= PID_set_control_Law()
%This function calculates the PID controller for a planar arm using preset
%conditions. 

clc, clear all, close all;

%Importing the parameters from table 5.1, page 115 of kelly's book
I_1 = 0.1213; 
I_2 = 0.0116;
m_1 = 6.5225;
r_1 = .0983;
m_2 = 2.0458;
r_2 = .0229;
l_1 = 0.26;
l_2 = 0.26; 
g = 9.81;

x0= [0,0,0,0]; % Initial Condition - Format:[theta1,theta2,dtheta1,dtheta2]

tf=10; 
endThetas = [pi/45; pi/10];
endVelocities = [0;0];

%% Solve the closed-loop system nonlinear differential equation (PlanarArmODE) via ode45
%%ode45 solves the differential equation and returns X with respect to T.
global torque
torque=[];

%record the integral as a global, I had errors making this a local variable
global intgrl
intgrl = [0;0];

%Record the timestep instances.
global lt
lt = 0;

[T,X] = ode45(@(t,x)planarArmODE(t,x,endThetas,endVelocities),[0 tf],x0);

%% Plot Data
figure('Name','Theta_1+Theta_2');
plot(T, X(:,1),'r-');
hold on
plot(T,X(:,2),'b-');
title('Theta_1 and Theta_2')
xlabel('time')
ylabel('rads')
snapnow

figure('Name','Error Theta_1+Theta_2 under PID SetPoint Control');
plot(T, endThetas(1) - X(:,1),'r-');
hold on
plot(T, endThetas(2) - X(:,2),'b-');
title('Theta_1 and Theta_2 error')
xlabel('Time')
ylabel('Rad')
hold off
snapnow

figure('Name','Input_PID control');
plot(T, torque(1,1:size(T,1)),'-' );
hold on
plot(T, torque(2,1:size(T,1)),'r--');
hold off
xlabel('time')
ylabel('torque')
snapnow

torque=[];

%% Defining functions
    function dx = planarArmODE(t,x,td,dtd)
        theta= x(1:2,1);
        dtheta= x(3:4,1);
        theta_d=td; % Desired Set-Point Position
        dtheta_d=dtd; % Desired velocity (Derivative of theta_d)        
        
        global Mmat Cmat Gmat
        
        a = I_1+I_2+m_1*r_1^2+ m_2*(l_1^2+ r_2^2);
        b = m_2*l_1*r_2;
        d = I_2+ m_2*r_2^2;

        Mmat = [a+2*b*cos(x(2)), d+b*cos(x(2));  d+b*cos(x(2)), d];
        Cmat = [-b*sin(x(2))*x(4), -b*sin(x(2))*(x(3)+x(4)); b*sin(x(2))*x(3),0];
        Gmat = [(m_1*r_1 + m_2*l_1)*g*sin(x(1)) + m_2*r_2*g*sin(x(1) + x(2)); m_2*r_2*g*sin(x(1) + x(2))];
        
        invM = inv(Mmat);
        invMC = invM*Cmat;
        invMG = invM*Gmat;        
        tau = PIDControl(theta_d,dtheta_d,theta,dtheta,t);        
        
        torque =[torque, tau];
        dx=zeros(4,1);
        dx(1) = x(3); %dtheta1
        dx(2) = x(4); %dtheta2        
        dx(3:4) = -invMC* x(3:4) - invMG + invM*tau;     
    end

    function tau = PIDControl(theta_d,dtheta_d,theta,dtheta,t)
        %Importing the gains from figure 9.5 page 216
        Kp=[30 0;0 30];
        Kv=[7 0;0 3];
        Ki=[70 0;0 100];
        e=theta_d-theta; % position error        
        de = dtheta_d - dtheta; % velocity error
        timechange = t-lt;
        intgrl = intgrl + (e*timechange); %integral error
        lt = t;
        tau = Kp*e + Kv*de + Ki*intgrl;
    end

disp('Finish.');

end
