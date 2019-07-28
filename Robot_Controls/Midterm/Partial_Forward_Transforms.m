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
