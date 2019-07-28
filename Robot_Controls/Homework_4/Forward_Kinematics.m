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