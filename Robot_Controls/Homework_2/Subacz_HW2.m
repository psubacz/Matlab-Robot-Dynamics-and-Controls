%%Peter Subacz
% RBE502 - Homework 2

%% Problem 1
syms l1 l2 a1 a2 I1 I2 theta1(t) theta2(t) m1 m2 t1r t2r td1r td2r t g
th1 = theta1;
th2 = theta2;

x1 = l1*cos(th1);
y1 = l1*sin(th1);
X1 = [x1;y1]

x2 = (a1*cos(th1))+(l2*cos(th1+th2));
y2 = (a1*sin(th1))+(l2*sin(th1+th2));
X2 = [x2;y2]

XD1 = diff(X1,t)
XD2 = diff(X2,t)

v1s = simplify(XD1.'*XD1)
v2s = simplify(XD2.'*XD2)

k1 = (m1*v1s*0.5)+(I1*(diff(th1,t))^2*0.5);
k2 = (m2*v2s*0.5)+(I2*(diff(th1,t)+diff(th2,t))^2*0.5);
KE = simplify(k1+k2)

p1 = m1*g*y1;
p2 = m2*g*y2;
PE = simplify(p1+p2)

L = KE-PE;
L = simplify(L)

aP = {theta1, theta2, diff(theta1(t),t), diff(theta2(t),t)};
sP = {t1r, t2r td1r, td2r};

torq1 = diff(subs(diff(subs(L,aP,sP),td1r),sP,aP),t)-subs(diff(subs(L,aP,sP),t1r),sP,aP);
torq2 = diff(subs(diff(subs(L,aP,sP),td2r),sP,aP),t)-subs(diff(subs(L,aP,sP),t2r),sP,aP);

torq = [simplify(torq1);simplify(torq2)]

%place into  MCG form
torq = torq(t);
M11 = simplify(torq(1)-subs(torq(1),diff(theta1(t),t,t),0))/diff(theta1(t),t,t);
M12 = simplify(torq(1)-subs(torq(1),diff(theta2(t),t,t),0))/diff(theta2(t),t,t);
M21 = simplify(torq(2)-subs(torq(2),diff(theta1(t),t,t),0))/diff(theta1(t),t,t);
M22 = simplify(torq(2)-subs(torq(2),diff(theta2(t),t,t),0))/diff(theta2(t),t,t);

M = [M11,M12;
    M21,M22]

G = subs(torq, {diff(theta1(t),t,t),diff(theta2(t),t,t),diff(theta1(t),t),diff(theta2(t),t)},{0,0,0,0})

C1 = simplify(torq(1)-(M(1,:)*[diff(theta1(t),t,t),diff(theta2(t),t,t)].'+G(1)));
C2 = simplify(torq(2)-(M(2,:)*[diff(theta1(t),t,t),diff(theta2(t),t,t)].'+G(2)));
C = [C1;C2]

%%
% 
% <<P1.PNG>>
% 

%% Problem 2A
syms a b c d

%Import problem matrix
A = [-1, 1, 0, 0, 0;
      0,-1, 1, 0, 0;
      0, 0,-1, 0, 0;
      0, 0, 0,-2, 1;
      0, 0, 0, 0,-2]
  
B = [0,0;
     0,0;
     a,b;
     0,0;
     c,d]

%Calculate the Reachability
% A is a 5x5 matrix and B is a 5x2 matrix. For this problem, u_i is
% expanded as follows : [B,A*B,...A^4*B]. This is then compare to the rows
% of matrix A to check for reachability.

u_i = [B,A*B,A^2*B,A^3*B,A^4*B]

if rank(u_i) == size(A,1)
    disp('System is reachable.')
else
    disp('System is not reachable.')
end

%Calculate the Controlability
% A is a 5x5 matrix and B is a 5x2 matrix. For this problem, t1 is
% expanded as follows : [B,A*B,...A^4*B] and t2 is expanded as follows : 
% [B,A*B,...A^4*B, A5]. The ranks of these matrixes are compared to check 
% for controllability.

t1 = [B,A*B,A^2*B,A^3*B,A^4*B]
t2 = [B,A*B,A^2*B,A^3*B,A^4*B,A^5]

if rank(t1) == rank(t2)
    disp('System is Controlable.')
else
    disp('System is not Controlable.')
end

%% Problem 2B
%Import problem matrix
A = [-1, 1, 0, 0, 0;
      0,-1, 1, 0, 0;
      0, 0,-1, 0, 0;
      0, 0, 0,-1, 1;
      0, 0, 0, 0,-1]
  
B = [0,0;
     0,0;
     a,b;
     0,0;
     c,d]

%Calculate the Reachability
% A is a 5x5 matrix and B is a 5x2 matrix. For this problem, u_i is
% expanded as follows : [B,A*B,...A^4*B]. This is then compare to the rows
% of matrix A to check for reachability.

u_i = [B,A*B,A^2*B,A^3*B,A^4*B]

if rank(u_i) == size(A,1)
    disp('System is reachable.')
else
    disp('System is not reachable.')
end

%Calculate the Controlability
% A is a 5x5 matrix and B is a 5x2 matrix. For this problem, t1 is
% expanded as follows : [B,A*B,...A^4*B] and t2 is expanded as follows : 
% [B,A*B,...A^4*B, A5]. The ranks of these matrixes are compared to check 
% for controllability.

t1 = [B,A*B,A^2*B,A^3*B,A^4*B]
t2 = [B,A*B,A^2*B,A^3*B,A^4*B,A^5]

if rank(t1) == rank(t2)
    disp('System is Controlable.')
else
    disp('System is not Controlable.')
end

%% Problem 3
syms x1 x2 lambda

%Setup initial conditions

%% Problem 3
clc,clear all, close all
syms k_1 k_2 lambda

%Setup initial conditions
A = [0,1;
     1,0]
B = [0;
     1]
U = [2]
K = [k_1,k_2]

%Solve using char. eqn. shown in class. Calculate the X_dot=AX+BU->X(A+BK)
M= A-(B*K)
%Calculate lambda, needs to be a 2x1 
lambdaI = lambda*eye(2)
det_z = det(M-lambdaI)

%Solve for poles -2,-3
eqn1 = subs(det_z,[lambda],[-2])
eqn2 = subs(det_z,[lambda],[-3])

%This generates a system of equations which can be used to solve for the ks
[C,D] = equationsToMatrix([eqn1, eqn2], [k_1,k_2])
k_12 = linsolve(C,D)

%The feedback has constant input of u=2 therefore Ax + BU where u =-kx ->
%x=u/-k
X = U./(-double(k_12))

%% Problem 4a

%%
% 
% <<P4_1.PNG>>
% 

%%
% 
% <<P4_2.PNG>>
% 

%%
% 
% <<P4_3.PNG>>
% 


%% Problem 4b
clc, clear all, close all 
syms x_1 x_2 x_3 x_4 m_1 m_2 g u l k_1 k_2 k_3 k_4 lambda

%Import the state space equations derivieted in part A 
x_1dot = x_2
x_2dot = (-m_2*g*cos(x_3)*sin(x_3)+m_2*l*x_4^2*sin(x_3)+u)/((m_1+m_2)-m_2*cos(x_3)^2)
x_3dot = x_4
x_4dot = (-(m_1+m_2)*g*sin(x_3)+m_2*l*x_4^2*sin(x_3)*cos(x_3)+u*cos(x_3))/((m_1+m_2)*l-m_2*l*cos(x_3)^2)

xdots = [x_1dot;
    x_2dot;
    x_3dot;
    x_4dot]

%Take the jacobian of the state equations with respect to x_1,x_2,x_3, and
%x_4
A = [diff(xdots(1),x_1),diff(xdots(1),x_2),diff(xdots(1),x_3),diff(xdots(1),x_4);
     diff(xdots(2),x_1),diff(xdots(2),x_2),diff(xdots(2),x_3),diff(xdots(2),x_4);
     diff(xdots(3),x_1),diff(xdots(3),x_2),diff(xdots(3),x_3),diff(xdots(3),x_4);
     diff(xdots(4),x_1),diff(xdots(4),x_2),diff(xdots(4),x_3),diff(xdots(4),x_4)]
    
% Take the jacobian of state equations with respect to U
B = [diff(xdots(1),u);
    diff(xdots(2),u);
    diff(xdots(3),u);
    diff(xdots(4),u)]

%Linearize around the point x=0 and u = 0
A = subs(A,[x_1, x_2, x_3, x_4,u],[0,0,0,0,0])
B = subs(B,[x_1, x_2, x_3, x_4,u],[0,0,0,0,0])


%% Problem 4c
%Substute in actual values, M = m_1+m_2 = 1.1
A_r = subs(A,[l,m_2,m_1,g],[1,0.1,1,-9.8])
B_r = subs(B,[l,m_2,m_1],[1,0.1,1])

K = [k_1,k_2,k_3,k_4];

%Solve using char. eqn. shown in class. Calculate the X_dot=AX+BU->X(A+BK)
equ= A_r-(B_r*K)

%Calculate lambda, needs to be a 2x1 
lambdaI = lambda*eye(4)
det_z = det(equ-lambdaI)

%Solve for poles -1,-2,-1+-j
eqn1 = subs(det_z,[lambda],[-1]) == 0;
eqn2 = subs(det_z,[lambda],[-2]) == 0;
eqn3 = subs(det_z,[lambda],[-1+j]) == 0;
eqn4 = subs(det_z,[lambda],[-1-j]) == 0;
% %This generates a system of equations which can be used to solve for the ks
[C_r,D_r] = equationsToMatrix([eqn1,eqn2,eqn3,eqn4], [k_1,k_2,k_3,k_4])
k_1234 = linsolve(C_r,D_r)
% 
% %The feedback has constant input of u=2 therefore Ax + BU where u =-kx ->
% %x=u/-k
% X = U./(-double(k_12))

%% Problem 4d
%
% For some reason when ever i tried to reuse code from above the method SS
% would error out. A_s and B_s should be the same as above.
%
A_s = [0,1,0,0;
    0,0,(-0.1*9.8)/1,0;
    0,0,0,1;
    0,0,((1.1*9.8)/1),0];

B_s = [0;
    1;
    0;
    -1]

C = [1,0,0,0;
    0,0,1,0]
%Ignored
D = [0,0].'

t_1 =0
t_2 =1
t_1=linspace(t_1,t_2,60);

State_Space = ss(A_s,B_s,C,D);
[~,t1,states] = step(State_Space,t_1);

figure;
subplot(2,1,1)
plot(t_1,states(:,3))
title('Pendulum Angle')
ylabel('angle')

subplot(2,1,2)
plot(t_1,states(:,1))
title('Vehicle Pos')
ylabel('Position')



