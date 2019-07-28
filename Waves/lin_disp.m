function [L,k] = lin_disp(T,h)
%Calculates teh linear displace of wave
g=9.81;
w=(2*pi)/T;
tol=1.0*10^-5; %%tolarance
%% Wavelegth
 k_old=(2*pi)/((g*T^2)/(2*pi));
% k_old=(2*pi)/T 
my_diff=1;
while (my_diff>tol) % creating the iteration and finding k 
    k=(w^2)/(g*tanh(k_old*h));
    my_diff=abs(k-k_old);
    k_old=k;
end;
L=(2*pi)/k; % creating L from k
end