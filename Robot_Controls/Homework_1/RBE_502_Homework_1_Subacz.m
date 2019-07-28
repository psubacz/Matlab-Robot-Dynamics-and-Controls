
%% RBE 502 - Homework 1 - Peter Subacz - 9/7/18
% This is homework 1 of RBE 502 - Robot Control

%% Problem 1 
% - No submission

%% Problem 2 
% Derive the dynamical model for the three-link RRR elbow manipulator
% shown in the below figure. Consider the links to be massless, that is, 
% consider point masses at the end of each link similar to what we had in
% the lecture for the two-link arm.
%
%%
% 
% <<F:\Graduate School\RBE502 - Robot Control\Homework\HW1\f2.PNG>>
% 
%
% Numeric Solution
% q = [0;0;0];
% l = [30;30;30]; %cm
% q = d2r(q); %CHANGE THE DEGREES TO RADIAN
% 
% theta = [ q(1);   q(2);   q(3)];
% d =     [ l(1);      0;      0];
% a =     [    0;   l(2);   l(3)]; %cm
% alpha = [ pi/2;      0;      0];
% dhTable = [theta(:),d(:),a(:),alpha(:)];
% dhTable = double(dhTable);
% 
% transformMatrix = Transform_Matrix(dhTable);
% forwardKinematics = Forward_Kinematics(transformMatrix);
% pTM = Partial_Forward_Transforms(transformMatrix);
% Stickmodel(pTM, '3 Joint Arm',1)

clc,clear,close all

syms q1 q2 q3 th1 th2 th3 al1 L1 L2 L3 m1 m2 m3 mL g

%Establish a DH-Table
q = [q1;q2;q3];
theta = [  q(1); q(2); q(3)];
d =     [    L1;    0;    0];
a =     [     0;   L2;   L3];
alpha = [   al1;    0;    0];
dhTable = [theta(:),d(:),a(:),alpha(:)]

%Calculate Forward Kinematics
transformMatrix = Transform_Matrix(dhTable);
forwardKinematics = Forward_Kinematics(transformMatrix);
forwardKinematics = simplify(forwardKinematics);
pTM = Partial_Forward_Transforms(transformMatrix);
pTM = simplify(pTM) %matrix that holds partial forward kinematics(T01, T12, T23...)


%Grab the tip position
tipPOS = [pTM(1,4,3);
       pTM(2,4,3);
       pTM(3,4,3)];
   
%Calculate the forward velocity kinematics (6-DOF (6x3) Jacobian)

%Calculate 3 translational velocities Jacobian of the end effector
Jv = simplify([diff(tipPOS(1),q1), diff(tipPOS(1),q2), diff(tipPOS(1),q3);
                 diff(tipPOS(2),q1), diff(tipPOS(2),q2), diff(tipPOS(2),q3);
                 diff(tipPOS(3),q1), diff(tipPOS(3),q2), diff(tipPOS(3),q3)]);

%Calculate 3 angular velocities Jacobian of the end effector

Jw = [pTM(1,3,3), pTM(1,3,3), pTM(1,3,3);
        pTM(1,3,3), pTM(1,3,3), pTM(1,3,3);
        pTM(1,3,3), pTM(1,3,3), pTM(1,3,3)];

%Combining to make a full Jacobian
JM = [Jv ; Jw]

% Calculate Torque at each mass (Torque = J^T * FTip)(Force propagation)
syms Fx Fy Fz N1 N2 N3
FTip = [Fx; Fy; Fz; N1; N2; N3];
Tau = JM.' * FTip
disp('in N/m')

% Calculate the Lagrangian Dynamics
syms q1 q2 q3 dq1 dq2 dq3 ddq1 ddq2 ddq3 th1(t) th2(t) th3(t) t real;
syms L1 L2 L3 m1 m2 m3 mL g real;

%The dynamical model is derived using the Euler-Langrange approach 

%Calculating the kinematic and potential energies of all these link masses
%using the velocities found in the Jacobian.
J_L1 = jacobian(pTM(1:3,4,1),[q1]);
J_L2 = jacobian(pTM(1:3,4,2),[q1 q2]);
J_L3 = jacobian(pTM(1:3,4,3),[q1 q2 q3]);
J_L1 = simplify(J_L1);
J_L2 = simplify(J_L2);
J_L3 = simplify(J_L3);

v_mL1 = J_L1*dq1;
v_mL2 = J_L2*[dq1 ; dq2];
v_mL3 = J_L3*[dq1 ; dq2 ; dq3];
v_mL1 = simplify(v_mL1);
v_mL2 = simplify(v_mL2);
v_mL3 = simplify(v_mL3);

%Calculate the kinetic energy
K1 = 0.5 * m1 * (v_mL1.' * v_mL1);
K2 = 0.5 * m2 * (v_mL2.' * v_mL2);
K3 = 0.5 * m3 * (v_mL3.' * v_mL3);
%Calculate the potential energy
P1 = m1 * g * pTM(3,4,1);
P2 = m2 * g * pTM(3,4,2);
P3 = m3 * g * pTM(3,4,3);

K = K1 + K2 + K3;
P = P1 + P2 + P3;

%Calculate the langrangian 
L = simplify( K - P )

syms th1(t) th2(t) th3(t);
A1 = diff(L,dq1);
B1 = diff(L,q1);

A1t = subs(A1,[q1 q2 q3 dq1 dq2 dq3], [th1 th2 th3 diff(th1(t),t) diff(th2(t),t) diff(th3(t), t)]);
dA1t = diff(A1t, t);
A1 = subs(dA1t,[th1 th2 th3 diff(th1(t),t) diff(th2(t),t) diff(th3(t), t) ...
diff(th1(t),t,t) diff(th2(t),t,t) diff(th3(t), t,t)],[q1 q2 q3 dq1 dq2 dq3 ddq1 ddq2 ddq3]);
A2 = diff(L,dq2);
A2t = subs(A2,[q1 q2 q3 dq1 dq2 dq3], [th1 th2 th3 diff(th1(t),t) diff(th2(t),t) diff(th3(t), t)]);
dA2t = diff(A2t, t);
A2 = subs(dA2t,[th1 th2 th3 diff(th1(t),t) diff(th2(t),t) diff(th3(t), t) ...
diff(th1(t),t,t) diff(th2(t),t,t) diff(th3(t), t,t)],[q1 q2 q3 dq1 dq2 dq3 ddq1 ddq2 ddq3]);
B2 = diff(L,q2);
A3 = diff(L,dq3);
A3t = subs(A3,[q1 q2 q3 dq1 dq2 dq3], [th1 th2 th3 diff(th1(t),t) diff(th2(t),t) diff(th3(t), t)]);
dA3t = diff(A3t, t);
A3 = subs(dA3t,[th1 th2 th3 diff(th1(t),t) diff(th2(t),t) diff(th3(t), t) ...
diff(th1(t),t,t) diff(th2(t),t,t) diff(th3(t), t,t)],[q1 q2 q3 dq1 dq2 dq3 ddq1 ddq2 ddq3]);
B3 = diff(L,q3);

Tau_1 = A1 - B1;
Tau_2 = A2 - B2;
Tau_3 = A3 - B3;

%Calculate the mass matrix
M11 = simplify(Tau_1 - subs(Tau_1,ddq1,0)) /ddq1;
M12 = simplify(Tau_1 - subs(Tau_1,ddq2,0)) /ddq2;
M13 = simplify(Tau_1 - subs(Tau_1,ddq3,0)) /ddq3;
M21 = simplify(Tau_2 - subs(Tau_2,ddq1,0)) /ddq1;
M22 = simplify(Tau_2 - subs(Tau_2,ddq2,0)) /ddq2;
M23 = simplify(Tau_2 - subs(Tau_2,ddq3,0)) /ddq3;
M31 = simplify(Tau_3 - subs(Tau_3,ddq1,0)) /ddq1;
M32 = simplify(Tau_3 - subs(Tau_3,ddq2,0)) /ddq2;
M33 = simplify(Tau_3 - subs(Tau_3,ddq3,0)) /ddq3;

M = [M11 M12 M13;
M21 M22 M23;
M31 M32 M33];

%Calculate the gravity matrix
G11 = subs(Tau_1, [dq1 dq2 dq3 ddq1 ddq2 ddq3], [0 0 0 0 0 0]);
G21 = subs(Tau_2, [dq1 dq2 dq3 ddq1 ddq2 ddq3], [0 0 0 0 0 0]);
G31 = subs(Tau_3, [dq1 dq2 dq3 ddq1 ddq2 ddq3], [0 0 0 0 0 0]);
G = simplify([G11;
              G21;
              G31]);

%Calculate the coriolis matrix
C11 = Tau_1 - (M(1,:) * [ddq1 ddq2 ddq3].' + G11);
C21 = Tau_2 - (M(2,:) * [ddq1 ddq2 ddq3].' + G21);
C31 = Tau_3 - (M(3,:) * [ddq1 ddq2 ddq3].' + G31);

C = simplify([C11;
              C21;
              C31]);

syms ddtheta1 ddtheta2 ddtheta3
TDDot = [ddtheta1;ddtheta2;ddtheta3];

%The standard form should be
% Tau = [Mass Inertial]*[ThetaDoubleDot]+[coriolis]*[Gravity]
% 3x3 3x1 3x1 3x1
General_Tau = M*TDDot+C+Tau



%% Problem 3
% - 
% For the 2-link arm shown below, let ?, ? be the distances of the centers
% of mass of the two links from the respective joint axes and ?, ? be the 
% length of the two links. Also let ? , ? be the masses of the two links. 
% Finally, let ?, ? be the moments of inertia relative to the centers of
% mass of the two links, respectively. Derive the dynamical model of the
% robot using Lagrange’s equation and finally write a MATLAB code that 
% calculates the joint torques for any given configuration/state of the 
% robot. Consider L_i = a_i/2 for simplicity. 
%
%%
% <<F:\Graduate School\RBE502 - Robot Control\Homework\HW1\f3.PNG>>
% 
% clc,clear,close all
% q = [0;0;0];
% l = [30;30;30]; %cm
% q = d2r(q); %CHANGE THE DEGREES TO RADIAN
% 
% theta = [ q(1);   q(2)];
% d =     [ 0;      0];
% a =     [    l(1);   l(2)]; %cm
% alpha = [ 0;      0];
% dhTable = [theta(:),d(:),a(:),alpha(:)];
% dhTable = double(dhTable);
% 
% transformMatrix = Transform_Matrix(dhTable);
% forwardKinematics = Forward_Kinematics(transformMatrix);
% pTM = Partial_Forward_Transforms(transformMatrix);
% Stickmodel(pTM, '3 Joint Arm',1)

clc,clear,close all
syms q1 q2 th1 th2 al1 L1 L2 m1 m2 m3 mL g

%Establish a DH-Table
q = [q1;q2];
theta = [  q(1); q(2)];
d =     [     0;    0];
a =     [    L1;   L2];
alpha = [   al1;    0];
dhTable = [theta(:),d(:),a(:),alpha(:)]

%Calculate Forward Kinematics
transformMatrix = Transform_Matrix(dhTable);
forwardKinematics = Forward_Kinematics(transformMatrix);
forwardKinematics = simplify(forwardKinematics);
pTM = Partial_Forward_Transforms(transformMatrix);
pTM = simplify(pTM)

%Grab the tip position
tipPOS = [pTM(1,4,2);
          pTM(2,4,2);
          pTM(3,4,2)];
          
%Calculate the forward velocity kinematics (6-DOF (6x3) Jacobian)

%Calculate 3 translational velocities Jacobian of the end effector
Jv = simplify([diff(tipPOS(1),q1), diff(tipPOS(1),q2);
               diff(tipPOS(2),q1), diff(tipPOS(2),q2);
               diff(tipPOS(3),q1), diff(tipPOS(3),q2)]);

%Calculate 3 angular velocities Jacobian of the end effector
Jw = [pTM(1,3,2), pTM(1,3,2);
      pTM(1,3,2), pTM(1,3,2);
      pTM(1,3,2), pTM(1,3,2)];
 
%Combining to make a full Jacobian   
JM = [Jv ; Jw]

% Calculate Torque at each mass (Torque = J^T * FTip)(Force propagation)
syms Fx Fy Fz N1 N2 N3
FTip = [Fx; Fy; Fz; N1; N2; N3];
Tau = JM.' * FTip
disp('in N/m')

% Calculate the Lagrangian Dynamics
syms q1 q2 dq1 dq2  ddq1 ddq2  th1(t) th2(t) t real;
syms L1 L2 m1 m2 m3 mL g real;

%The dynamical model is derived using the Euler-Langrange approach 

%Calculating the kinematic and potential energies of all these link masses
%using the velocities found in the Jacobian.
J_L1 = jacobian(pTM(1:3,4,1),[q1]);
J_L2 = jacobian(pTM(1:3,4,2),[q1 q2]);
J_L1 = simplify(J_L1);
J_L2 = simplify(J_L2);

v_mL1 = subs(J_L1,L1,L1/2) * dq1;
v_mL2 = subs(J_L2,[L1 L2],[L1 L2/2]) * [dq1 ; dq2];

v_mL1 = simplify(v_mL1);
v_mL2 = simplify(v_mL2);

%Calculate the kinetic energy
K1 = 0.5 * m1 * (v_mL1.' * v_mL1);
K2 = 0.5 * m2 * (v_mL2.' * v_mL2);

P1 = m1 * g * subs(pTM(3,4,1),L1,L1/2);
P2 = m2 * g * subs(pTM(3,4,2),L2,L2/2);

K = K1 + K2;
P = P1 + P2;

%Calculate the langrangian 
L = simplify( K - P )

syms th1(t) th2(t);
A1 = diff(L,dq1);
B1 = diff(L,q1);

A1t = subs(A1,[q1 q2 dq1 dq2], [th1 th2 diff(th1(t),t) diff(th2(t),t)]);
dA1t = diff(A1t, t);
A1 = subs(dA1t,[th1 th2  diff(th1(t),t) diff(th2(t),t) diff(th1(t),t,t)...
    diff(th2(t),t,t)],[q1 q2 dq1 dq2 ddq1 ddq2]);
A2 = diff(L,dq2);
A2t = subs(A2,[q1 q2 dq1 dq2], [th1 th2 diff(th1(t),t) diff(th2(t),t)]);
dA2t = diff(A2t, t);
A2 = subs(dA2t,[th1 th2 diff(th1(t),t) diff(th2(t),t) diff(th1(t),t,t)...
    diff(th2(t),t,t)],[q1 q2 dq1 dq2 ddq1 ddq2]);
B2 = diff(L,q2);

Tau_1 = A1 - B1;
Tau_2 = A2 - B2;

%Calculate the mass matrix
M11 = simplify(Tau_1 - subs(Tau_1,ddq1,0)) /ddq1;
M12 = simplify(Tau_1 - subs(Tau_1,ddq2,0)) /ddq2;
M21 = simplify(Tau_2 - subs(Tau_2,ddq1,0)) /ddq1;
M22 = simplify(Tau_2 - subs(Tau_2,ddq2,0)) /ddq2;
M = [M11 M12;
     M21 M22];
 
G11 = subs(Tau_1, [dq1 dq2 ddq1 ddq2], [0 0 0 0]);
G21 = subs(Tau_2, [dq1 dq2 ddq1 ddq2], [0 0 0 0]);
G = simplify([G11;
              G21]);

%Calculate the coroilis matrix        
C11 = Tau_1 - (M(1,:) * [ddq1 ddq2].' + G11);
C21 = Tau_2 - (M(2,:) * [ddq1 ddq2].' + G21);

%Mass Matrix
C = simplify([C11;
              C21]);
          
syms ddtheta1 ddtheta2
TDDot = [ddtheta1;ddtheta2];
%The standard form should be
% Tau = [Mass Inertial]*[ThetaDoubleDot]+[coriolis]*[Gravity]
% 3x3 3x1 3x1 3x1
General_Tau = M*TDDot+C+Tau