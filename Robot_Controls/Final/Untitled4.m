clc, clear all,close all

t = 11:0.01:12;
dt = (t-11);
a = -1;
b = 0;

% y1 =(a*(dt).^3)
y1 =(a*(dt*.415).^3)-0.15
plot(dt,y1)

