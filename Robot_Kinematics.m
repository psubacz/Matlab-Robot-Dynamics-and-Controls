classdef Robot_Kinematics
    %UNTITLED6 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        dhTable
        partialTransformMatrix
        plotTitle
        applyView
        Home_Frame
        dhTransform
        x
        y
        z
        theta
        d
        a
        alpha
        wow
    end
    
    methods(Static)
        function [ transformMatrix ] = Transform_Matrix(dhTable)
            %UNTITLED6 Summary of this function goes here
            %   This function takes the DH parameter and returns the n-dimensional
            %   transformation matrix Get the size of the dh table, should take the 
            %   form of n x 4[numRows, maybe optimise with size(A,1)? it should be 
            %   able to return the rows...
            numRows = size(dhTable,1);
            %   Construct a 4x4xn homogenuous transform matrix from DH table
            for row = 1:numRows
                transformMatrix(:,:, row)= DH_Parameters(dhTable(row,1),...
                dhTable(row,2), dhTable(row,3), dhTable(row,4));
            end
        end
        
        function [] = Stickmodel( partialTransformMatrix,plotTitle,applyView)
        %This function generate a plot of a X*n serial manipulator and plots the
        %joints(green) and link lengths (black)
            numDims = size(partialTransformMatrix,3);
            xPos = [0];
            yPos = [0];
            zPos = [0];
        %Points at each link and joint
            for i = 1:numDims   
                xPos(:,i) = partialTransformMatrix(1,4,i);
                yPos(:,i) = partialTransformMatrix(2,4,i);
                zPos(:,i) = partialTransformMatrix(3,4,i);
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
        
        function [Position_Vector] = Robot_Tip_Position(Home_Frame)
        %Returns the robot tip vector
        Position_Vector = [Home_Frame(1,4), Home_Frame(2,4), Home_Frame(3,4)].';
        end 
        
        function [ partialTransform ] = Partial_Forward_Transforms( dhTransform )
        %This fuction returns the partial transforms of a transformation matrix
            %Grab the size of the 3d matrix
            numTransforms = size(dhTransform,3);
            %Setup 4x4 Identity matrix
            transform = [1,0,0,0;
                         0,1,0,0;
                         0,0,1,0;
                         0,0,0,1];
            %calculate transforms and store them in a matrix
            for cf = 1:numTransforms
                transform = transform*dhTransform(:,:,cf);  
                partialTransform(:,:,cf) = transform;
            end
        end
        
        function [az,el]=normalToAzimuthElevationDEG(x,y,z)
            if length(x)>1
                v         = x;
                x=v(1);
                y=v(2);
                z=v(3);
            end
            if x==0 && y==0
                x =eps;
            end
            vNorm = sqrt(x^2+y^2+z^2);
            x=x/vNorm;
            y=y/vNorm;
            z=z/vNorm;
            az = 180/pi*asin(x/sqrt(x^2+y^2));
            el = 180/pi*asin(z);
        end   
        
        function [transform] = Forward_Kinematics(dhTransform)
        % Calculate the forward kinematics of a transformation matrix.
            numTransforms = size(dhTransform,3);

            transform = [1,0,0,0;
                         0,1,0,0;
                         0,0,1,0;
                         0,0,0,1];

            for cf = 1:numTransforms
             transform = transform*dhTransform(:,:,cf);   
            end
        end
        
        function [transMatrix] = DH_parameters(theta, d, a, alpha)
        %This method returns a the transformation of DH parameterized rotation.
        %where theta =  rotation around z, d is the offset around z, a is the
        %offest around x, and alpha is the rotation around x
        %   Detailed explanation goes here
        %
        % Theta is the rotation around Zn-1 that is required to make
        %   axis Xn-1 to match axis Xn. This includes rotations around the joint
        %
        % Alpha is the rotation around Xn that is required to make
        %  axis Zn-1 to match axis Zn. Not: the n-1 frame is rotated here.
        %
        % R(A) is the distance between the center of n-1 to center of n 
        %   in the frame Xn. Extend the axis of Xn to until it hits Zn-1.
        %
        % D is the distance between the center of frame n-1 to the center in n
        %   measauered in the Zn-1 direction. Note: extende the Zn-1 parameter.
            zAxisRot = [cos(theta), -sin(theta), 0, 0;
                        sin(theta), cos(theta), 0, 0;
                        0, 0, 1, 0;
                        0, 0, 0, 1];
            zAxisTrans = [1, 0, 0, 0;
                          0, 1, 0, 0;
                          0, 0, 1, d;
                          0, 0, 0, 1];
            xAxisTrans = [1, 0, 0, a;
                          0, 1, 0, 0;
                          0, 0, 1, 0;
                          0, 0, 0, 1];
             xAxisRot = [1, 0, 0, 0;
                         0, cos(alpha), -sin(alpha), 0;
                         0, sin(alpha), cos(alpha), 0;
                         0, 0, 0, 1];
            transMatrix = zAxisTrans*zAxisRot*xAxisTrans*xAxisRot;
        end

        function [ wow ] = d2r( wow )
        %UNTITLED11 Summary of this function goes here
        %   Detailed explanation goes here
            wow = wow./360.*(2.*pi());
        end
    end
end
        

