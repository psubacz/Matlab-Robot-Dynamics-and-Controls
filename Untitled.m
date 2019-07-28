clc, clear all, close all
q = [0;45;-45];
l = [30;30;30]; %cm
q = d2r(q); %CHANGE THE DEGREES TO RADIAN

theta = [ q(1);   q(2);   q(3)];
d =     [ l(1);      0;      0];
a =     [    0;   l(2);   l(3)]; %cm
alpha = [ pi/2;      0;      0];
dhTable = [theta(:),d(:),a(:),alpha(:)];
dhTable = double(dhTable);

transformMatrix = Robot_Kinematics.Transform_Matrix(dhTable);
pTM = Robot_Kinematics.Partial_Forward_Transforms(transformMatrix)
forwardKinematics = Robot_Kinematics.Forward_Kinematics(transformMatrix)
Robot_Kinematics.Stickmodel(pTM, '3 Joint Arm',1)

if forwardKinematics == pTM(:,:,3)
    disp('true')
end