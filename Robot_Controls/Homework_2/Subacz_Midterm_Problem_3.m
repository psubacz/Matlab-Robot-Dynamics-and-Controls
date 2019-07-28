%% Problem 3B
% DESCRIPTIVE TEXT

clc, clear all, close all 
syms L m N k_m k_b d_th r_a g x_1 u x_2 k_1 k_2 lambda

%Import the state space equations derivieted in part A 
x_1dot = x_2
x_2dot = g*sin(x_1)/L+(u*N*k_m-k_m*k_b*N^2*x_2)*(r_a*m*L^2)^-1

xdots = [x_1dot;
         x_2dot]

%Take the jacobian of the state equations with respect to x_1,x_2
A = [diff(xdots(1),x_1),diff(xdots(1),x_2);
     diff(xdots(2),x_1),diff(xdots(2),x_2)]
    
% Take the jacobian of state equations with respect to U
B = [diff(xdots(1),u);
    diff(xdots(2),u)]

%Linearize around the point x=0 and u = 0
A = subs(A,[x_1, x_2,u],[0,0,0])
B = subs(B,[x_1, x_2,u],[0,0,0])

%% Problem 3C
% Test to see if the linearized system is stable
% Solve using char. eqn. shown in class. Calculate the X_dot=AX+BU->X(A+BK)
K = [k_1,k_2]
M = A-(B*K)
%Calculate lambda, needs to be a 2x1 
lambdaI = lambda*eye(2)
det_z = det(M-lambdaI)

e = eig(M-lambdaI)
s = subs(e,[k_b,k_m,m,N,r_a],[0.1,0.1,1,10,1])

X = real(s)

%% Problem 3D
% Test for system controllability
t1 = [B,A*B,A^2*B]
t2 = [B,A*B,A^2*B]

if rank(t1) == rank(t2)
    disp('System is Controlable.')
else
    disp('System is not Controlable.')
end

%% Problem 3E
A_r = double(subs(A,[k_b,k_m,m,N,r_a,g,L],[0.1,0.1,1,10,1,9.8,1]))
B_r = double(subs(B,[N,k_m,r_a,L,m],[10,0.1,1,1,1]))

K = [k_1,k_2];

%Solve using char. eqn. shown in class. Calculate the X_dot=AX+BU->X(A+BK)
equ= A_r-(B_r*K)

%Calculate lambda, needs to be a 2x1 
lambdaI = lambda*eye(2)
M=equ-lambdaI
det_z = det(M)

%Solve for poles -1,-2
eqn1 = subs(det_z,[lambda],[-1]) == 0
eqn2 = subs(det_z,[lambda],[-2]) == 0

% %This generates a system of equations which can be used to solve for the ks
[C_r,D_r] = equationsToMatrix([eqn1,eqn2], [k_1,k_2])
k_12 = double(linsolve(C_r,D_r))
C_r = double(C_r)
D_r = double(D_r)

%% Problem 4d

t_1 =0
t_2 =1
t_1=linspace(t_1,t_2,60);

State_Space = ss(A_r,B_r,C_r,D_r);
[~,t1,states] = step(State_Space,t_1);

figure;
subplot(2,1,1)
plot(t_1,states(:,2))
title('Robot Arm Angle')
ylabel('angle')
