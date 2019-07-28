function [] = Stickmodel( partialforwardKinematics,plotTitle,applyView)
%This function generate a plot of a X*n serial manipulator and plots the
%joints(green) and link lengths (black)
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
    title(plotTitle)
    xlabel('x');
    ylabel('y');
    zlabel('z');
    lines = plot3(xPos,yPos,zPos,'black','Linewidth',4);
    xlim([-150,150])
    ylim([-150,150])
    zlim([0,130])
    legend('Joints')
    if applyView == 1 %apply z-x view
        [az, el] = normalToAzimuthElevationDEG(0,90,0);
        view(az,el)
    elseif applyView == 2 %apply x-y view
        [az, el] = normalToAzimuthElevationDEG(0,0,90);
        view(az,el)
    elseif applyView == 3 %apply z-y view
        [az, el] = normalToAzimuthElevationDEG(90,0,0);
        view(az,el)
    end
end

