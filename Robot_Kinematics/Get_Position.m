function [transformMatrix] = Get_Position()
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
%   Joint Torque Vector, 
% 
% %Joint Angle Acceleration Vector 3x1
% %Mass maxtrix is 3x3
% %Coriolis/Centripital Coupling Vector 3x1
% %Graity term 3x1
% 
% To get the point masses at the center of each link, 3 intermediate 
% links were introduced that divided the links in half

syms q1 q2 q3 th2 th3 th4 al1 l1 l2 l3 m1 m2 m3 mL g

%Establish a DH-Table
q = [q1;q2;q3];
l = [l1;l2;l3];
% q = degtorad(q);              %CHANGE THE DEGREES TO RADIANS
theta = [   0; 0+q(1);      0;  0+q(2);      0; 0+q(3);     0];
d =     [   0;      0;      0;       0;      0;      0;      0];
a =     [   0; l(1)/2; l(1)/2;  l(2)/2; l(2)/2; l(3)/2; l(3)/2]; %cm 
alpha = [pi/2;      0;      0;       0;      0;      0;      0];
dhTable = [theta(:),d(:),a(:),alpha(:)];

%Get the size of the DH matrix
dhSize = size(dhTable); %Get the size of the DH matrix

[numRows, numColumns] = size(dhTable);

for row = 1:numRows
    transformMatrix(:,:, row)= DH_parameters(dhTable(row,1),...
        dhTable(row,2), dhTable(row,3), dhTable(row,4));
end

end
