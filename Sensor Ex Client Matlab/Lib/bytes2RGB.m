function [R G B] = bytes2RGB(bmpBytes, size)

h = size(1);
w = size(2);

pixelsData = reshape(uint8(bmpBytes), 3, w, h);

R = transpose(reshape(pixelsData(3, :, :), w, h));
G =  transpose(reshape(pixelsData(2, :, :), w, h));
B =  transpose(reshape(pixelsData(1, :, :), w, h));
 
end

