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