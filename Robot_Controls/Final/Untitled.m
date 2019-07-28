clc,close all, clear all
x = 0.1;
y = -0.3;
for t = 0:0.01:15
    if t<10
        x1 = 0.1 - t*2*0.01;
        y1 = -0.3;
    elseif t>10 && t<11 
        %p1
        disp(t)
        x1 = -0.1;
        y1 = -0.3+(t-10)*0.075
    elseif t>11 && t<12 
        %p2
        x1 = -0.1+(t-11)*0.066;
        y1 = -.225+(t-11)*0.075;
    elseif t>12 && t<13 
        %p3 
        x1 = -0.034+(t-12)*0.066;
        y1 = -0.15;
    elseif t>13 && t<14 
        x1 = 0.0320+(t-13)*0.066;
        y1 = -0.15-(t-13)*0.075;
    elseif t>14 && t<15 
        x1 = 0.1;
        y1 = -0.2250-(t-14)*0.075;
    end
    x = [x,x1];
    y = [y,y1];
end

plot(x,y)
axis([-0.4 0.4 -0.4 0.2])