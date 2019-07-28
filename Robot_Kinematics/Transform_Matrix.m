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

