%% Subacz Problem 4
% Solution to problem 4

%% Problem 4A
%%
% 
% <<4A.PNG>>
% 



clc, clear all, close all

syms l1 l2 l3 theta1(t) theta2(t) theta3(t) m1 m2 m3 t1r t2r t3r ...
    td1r td2r td3r t g

th1 = theta1;
th2 = theta2;
th3 = theta3;

x1 = 0;
y1 = 0;
z1 = l1;
X1 = [x1;
      y1;
      z1]
  
x2 = l2*cos(th2)*cos(th1);
y2 = l2*cos(th2)*sin(th1);
z2 = l1+(l2*sin(th2));
X2 = [x2;
      y2;
      z2]

x3 = ((l2*cos(th2))+(l3*cos(th2+th3)))*cos(th1);
y3 = ((l2*cos(th2))+(l3*cos(th2+th3)))*sin(th1);
z3 = l1+(l2*sin(th2))+(l3*sin(th2+th3));
X3 = [x3;
      y3;
      z3]

XD1 = diff(X1,t)
XD2 = diff(X2,t)
XD3 = diff(X3,t)

v1s = XD1.'*XD1;
v2s = simplify(XD2.'*XD2);
v3s = simplify(XD3.'*XD3);

k1 = m1*v1s*0.5;
k2 = m2*v2s*0.5;
k3 = m3*v3s*0.5;
KE = simplify(k1+k2+k3)

p1 = m1*g*z1;
p2 = m2*g*z2;
p3 = m3*g*z3;
PE = simplify(p1+p2+p3)

L=simplify(KE-PE)

aP = {theta1, theta2, theta3, diff(theta1(t),t), diff(theta2(t),t),diff(theta3(t),t)};
sP = {t1r, t2r,t3r, td1r, td2r,td3r};

torq1 = diff(subs(diff(subs(L,aP,sP),td1r),sP,aP),t)-subs(diff(subs(L,aP,sP),t1r),sP,aP);
torq2 = diff(subs(diff(subs(L,aP,sP),td2r),sP,aP),t)-subs(diff(subs(L,aP,sP),t2r),sP,aP);
torq3 = diff(subs(diff(subs(L,aP,sP),td3r),sP,aP),t)-subs(diff(subs(L,aP,sP),t3r),sP,aP);

torq = [simplify(torq1);simplify(torq2);simplify(torq3)]

torq = torq(t);
M11 = simplify(torq(1)-subs(torq(1),diff(theta1(t),t,t),0))/diff(theta1(t),t,t);
M12 = simplify(torq(1)-subs(torq(1),diff(theta2(t),t,t),0))/diff(theta2(t),t,t);
M13 = simplify(torq(1)-subs(torq(1),diff(theta3(t),t,t),0))/diff(theta3(t),t,t);

M21 = simplify(torq(2)-subs(torq(2),diff(theta1(t),t,t),0))/diff(theta1(t),t,t);
M22 = simplify(torq(2)-subs(torq(2),diff(theta2(t),t,t),0))/diff(theta2(t),t,t);
M23 = simplify(torq(2)-subs(torq(2),diff(theta3(t),t,t),0))/diff(theta3(t),t,t);

M31 = simplify(torq(3)-subs(torq(3),diff(theta1(t),t,t),0))/diff(theta1(t),t,t);
M32 = simplify(torq(3)-subs(torq(3),diff(theta2(t),t,t),0))/diff(theta2(t),t,t);
M33 = simplify(torq(3)-subs(torq(3),diff(theta3(t),t,t),0))/diff(theta3(t),t,t);
M = [M11,M12,M13;
    M21,M22,M23;
    M31,M32,M33]

G = subs(torq, {diff(theta1(t),t,t),diff(theta2(t),t,t),diff(theta3(t),t,t),diff(theta1(t),t),diff(theta2(t),t),diff(theta3(t),t)},{0,0,0,0,0,0})

C1 = simplify(torq(1)-(M(1,:)*[diff(theta1(t),t,t),diff(theta2(t),t,t),diff(theta3(t),t,t)].'+G(1)));
C2 = simplify(torq(2)-(M(2,:)*[diff(theta1(t),t,t),diff(theta2(t),t,t),diff(theta3(t),t,t)].'+G(2)));
C3 = simplify(torq(3)-(M(3,:)*[diff(theta1(t),t,t),diff(theta2(t),t,t),diff(theta3(t),t,t)].'+G(3)));
C = [C1;C2;C3]

syms dtheta1 dtheta2 dtheta3
q ={dtheta1 dtheta2 dtheta3}
qq = {diff(theta1(t), t),diff(theta2(t), t),diff(theta3(t), t)}

m = subs(M,qq,q)
c = subs(C,qq,q)
g = subs(G,qq,q)

%% Problem 4B
%
disp("I have considered the PD set point control law u=k_pe+k_ve+q(q), :)")
% The gravity term should drop out due to the control law compensating for
% gravity.

%% Problem 4C
%%
% 
% <<4C.PNG>>
% 

%% Problem 4D,E,F
%%
% 
% <<4DEF.PNG>>
% 
%% Problem 4G
%See PD_Set Control_Law Script