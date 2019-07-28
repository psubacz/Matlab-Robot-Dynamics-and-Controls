function  result = showRGB(R, G, B)

imgData = cat(3, ...
        R, ...
        G, ...
        B);

imshow(imgData);

result = true;

end

