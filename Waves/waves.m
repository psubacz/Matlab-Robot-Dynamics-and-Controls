%%Peter Subacz
%%Dr. Weaver - Costal Structures
%%Homework 1 - Waves Code
%%9/2/2015

clear all; close all; clc;
%%PART B
%%Depths(m)
% prompt='Please input a water depth ';
% h=input(prompt)
d_1=10;
h=d_1;
%%Wave Hieght(m)
H_1=2; 
%%Wave_angle
A_1=20;
%%Bottom slope
Tan_b=1/50;
%%Depth at position 2
d_2=5;
%%Depth below SWL at position 2
z_1=-2.5;
%%Period(s)
T=10;
%%Density
tol=1.0*10^-5;
w=(2*pi)/T;
rho=1025;
%%Gravity
g=9.81;
z=2;
%%Script Start

%%Part B

[L,k]=lin_disp(T,h) %%L
%%Relative Depth
Rd=d_1/L
%%If a shallow water wave  
if(Rd<=0.05)   
   disp('Checking to see if this is a shallow water wave')
   C_1=sqrt(g*d_1) %%m/s
   Cg_1=C_1 %%m/s
end
%If a intermediate water wave
 if(Rd>0.05)&&(Rd<0.5) 
     disp('This is a intermediate water wave')
  C_1=sqrt((g/k)*tanh(k*d_1))%%m/s
   Cg_1=(C_1/2)*(1+((4*pi*d_1)/L)/(sinh((4*pi*d_1)/L)))%%m/s
 end  
% % %If a deep water wave
 if(Rd>=0.5)
   disp('This is a deep water wave')
   C_1=sqrt((g*L)/(2*pi))%%m/s
   Cg_1=C_1/2%%m/s
 end
 
 %% Part C
 %%Deepwater WaveLength
 L_0=((g*T^2)/(2*pi))%%m
 %%DeepWater Wavespeed
 C_0=((g*T)/(2*pi))%%m/s
 %%Deepwater Groupspeed
 Cg_0=C_0/2%%m/s
 %%Snells Law
 A_0= asind((C_0/C_1)*sind(A_1)) %%Degrees
 %%Deepwater Wave Hieght
 H_0 = H_1*sqrt(Cg_1/Cg_0)*(sqrt(cosd(A_1)/cosd(A_0)))%%M

 %% Unrefracted deepwater wave height
 K_r=(sqrt(cosd(A_0)/cosd(A_1)));
 H_0_prime=K_r*H_0 %m
 
%%PART D
%Wave hieght at breaking
H_b= 0.56*((H_0_prime/L_0)^(-1/5))*H_0;%m

%%Breaking at depth
A_b=(43.8*(1-exp(-19*(Tan_b))));
B_b=(1.56/(1+exp(-19.5*(Tan_b))));
Gamma_b=B_b-(A_b*H_b/(g*T^2));
d_b=H_b/Gamma_b%m

%%Part E
if d_2<d_b
    disp ('Wave breaks before location 2(d_2)... Ending Program')
else
H_2=2.23
d_2=5
end
%% Wavelegth
 k_old=(2*pi)/((g*T^2)/(2*pi));
% k_old=(2*pi)/T 
my_diff=1;
while (my_diff>tol) % creating the iteration and finding k 
    k=(w^2)/(g*tanh(k_old*d_2));
    my_diff=abs(k-k_old);
    k_old=k;
end
L_2=(2*pi)/k % creating L from k
A_2=14.5
H_2=2.23
d_2=5
%% Wavelegth
 k_old=(2*pi)/((g*T^2)/(2*pi));
% k_old=(2*pi)/T 
my_diff=1;
while (my_diff>tol) % creating the iteration and finding k 
    k=(w^2)/(g*tanh(k_old*d_2));
    my_diff=abs(k-k_old);
    k_old=k;
end
L_2=(2*pi)/k % creating L from k
%%Relative Depth
Rd_1=d_2/L_2
%If a shallow water wave  
if(Rd_1<=0.05)   
   disp('Checking to see if this is a shallow water wave')
   %%Shallow_Rd(Rd,L0,L,h,g)
   C_2=sqrt(g*d_2)%%m/s
   Cg_2=C_2%%m/s
end
%If a intermediate water wave
 if(Rd>0.05)&&(Rd<0.5) 
     disp('This is a intermediate water wave')
  %%Intermediate_Rd(Rd,L0,L,g,h,k)
  C_2=sqrt((g/k)*tanh(k*d_2))%%m/s
   Cg_2=(C_2/2)*(1+((4*pi*d_2)/L_2)/(sinh((4*pi*d_2)/L_2)))%%m/s
% % %If a deep water wave
 if(Rd>=0.5)
   disp('This is a deep water wave')
   %%Deep_Rd(Rd,L0,L,T,g)
   C_2=sqrt((g*L)/(2*pi))%%m/s
   Cg_2=C_2/2%%m/s
 end
  %%Snells Law
 A_2= asind((C_2/C_0)*sind(A_0))%degrees
 H_2 = H_1*sqrt(Cg_1/Cg_2)*(sqrt(cosd(A_1)/cosd(A_2))) %m
 
%  H_0_prime_1=
 a=H_2/2 %m
 u= (a*w)*((cosh(k*z_1+k*d_2))/(sinh(k*d_2)))%%m/s
 w= (a*w)*((sinh(k*z_1+k*d_2))/(sinh(k*d_2)))%%m/s
 a_x=(a*g*k)*((cosh(k*z_1+k*d_2)/(cosh(k*d_2))))%%m/s^2
 a_y= (a*g*k)*((sinh(k*z_1+k*d_2))/(cosh(k*d_2)))%%m/s^2
 E= 1/8*rho*g*H_2^2*L_2 %J/m
 P=E*Cg_2 %W/M
end
 

