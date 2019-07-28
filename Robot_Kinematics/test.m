clc, clear all, close all
theta1 = 0
theta2 = 0+pi/6
theta3 = 0+pi/6
l_1 = 1
l_2 = 1
l_3 = 1

theta = [ theta1+pi/2; theta2;theta3];
d =     [    l_1;       0;    0];
a =     [    0;      l_2; l_3]; %cm
alpha = [    pi/2;        0;   0];

dhTable = [theta(:),d(:),a(:),alpha(:)];
%Create a dh table
transformMatrix = Transform_Matrix(dhTable);
%Calculate the forward kinematics of links 1 and 2
partialforwardKinematics = Partial_Forward_Transforms(transformMatrix);
%Get the tip position for links 1 and 2
RobotTipPosition = [partialforwardKinematics(:,4,1),partialforwardKinematics(:,4,2)];
    numDims = size(partialforwardKinematics,3);
    xPos = [0];
    yPos = [0];
    zPos = [0];
%Points at each link and joint
    for i = 1:numDims   
        xPos(:,i) = partialforwardKinematics(1,4,i);
        yPos(:,i) = partialforwardKinematics(2,4,i);
        zPos(:,i) = partialforwardKinematics(3,4,i);
    end
    xPos = cat(2,0,xPos);
    yPos = cat(2,0,yPos);
    zPos = cat(2,0,zPos);
% The green dots are the point masses and the red dots are the joints.
%%Setup a 3d plot
    plot3(xPos,yPos,zPos,'ro','markersize',10,'markerfacecolor','g');
    hold on
    rotate3d on
    grid on
    title('l')
    xlabel('x');
    ylabel('y');
    zlabel('z');
    lines = plot3(xPos,yPos,zPos,'black','Linewidth',4);
    xlim([-5,5])
    ylim([-5,5])
    zlim([0,10])
    legend('Joints')