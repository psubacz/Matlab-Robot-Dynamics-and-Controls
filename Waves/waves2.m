%%Peter Subacz
%%Dr. Weaver - Costal Structures
%%Homework 1 - Waves Code
%%9/2/2015

%%clear all; close all; clc;

%%Depths(m)
prompt='Please input a water depth ';
h=input(prompt)
%%Wave Hieght(m)
H=1.5;
%%Period(s)
T=12;
%%Density
rho=1025;
%%Gravity
g=9.81;
w=(2*pi)/T;
tol=1.0*10^-5; %%tolarance
C=0;
Cg=0;
z=2;
%% Wavelegth
 k_old=(2*pi)/((g*T^2)/(2*pi));
% k_old=(2*pi)/T 
my_diff=1;
while (my_diff>tol) % creating the iteration and finding k 
    k=(w^2)/(g*tanh(k_old*h));
    my_diff=abs(k-k_old);
    k_old=k;
end
L0=(2*pi)/k % creating L from k
%%Relative Depth
Rd=h/L0
%%If a shallow water wave  
if(Rd<0.05)   
   disp('Checking to see if this is a shallow water wave')
   L=T*sqrt(g*h)
   Shallow_Rd(Rd,L0,L,h,g)
   C=sqrt(g*h)
   Cg=C
end
%If a intermediate water wave
 if(Rd>=0.05)&&(Rd<=0.5) 
     disp('This is a intermediate water wave')
 L=((g*T^2)/(2*pi))*tanh((2*pi*h)/L0)
   Celerity = L/T
  Intermediate_Rd(Rd,L0,L,g,h,k)
  C=sqrt((g/k)*tanh(k*h))
   Cg=(C/2)*(1+((4*pi*h)/L)/(sinh((4*pi*h)/L)))
 end  
% % %If a deep water wave
 if(Rd>0.5)
   disp('This is a deep water wave')
   L=((g*T^2)/(2*pi)) 
   Deep_Rd(Rd,L0,L,T,g)
   C=sqrt((g*L)/(2*pi))
   Cg=C/2
end  
% % % Energy for 1 wave with 100m wave crest width
E = 0.5*rho*g*H^2*L

%% Wave power for 1 wave with 100m wave crest width
Wave_Power = E*Cg

%% Maximum wave height possible for this wave period ad water depth
Theta = 0;
H_max = (1/((g^1/5)*(k^(4/5))))*(((H^2*C*cos(Theta))^(2/5))/(4^1/5))
%% Depth limited or Wave Angle Limited
Depth_limited = H_max/0.78
%% if the wave angle is greator than 120 degrees or H/L is greater than 1/7 the wave will break
Wave_angle = (H/L)

if (Wave_angle > 0.14)
    disp ('The wave is limited by its wave angle')
else
    disp ('the wave is limited by its Depth angle') 
end


%% Ursell parameter
Ursell_Parameter = ((H/h)*(L/h)^2)

%%Hydrostatic pressure

%%Hydrostatic pressure
%% hydrostatic pressure + ATM pressure
z=0;
Hydrostatic_pressure = rho*g*(h-z)

%%Max Dynamic(wave induced pressure)
x=0; %% this value should be what ever makes the oscillation term 0 to make cos(0) = 1

P_Dynamic = (((rho*g*(h-z))/2)*(cosh(k*(h+z)))/(cosh(k*h)))*cos(k*x-w*T)

%%Max total Gage pressure Under crest
Pressure_Under_Crest = Hydrostatic_pressure + P_Dynamic

%%Max total gage pressure under trough
Pressure_Under_Trough = Hydrostatic_pressure - P_Dynamic

%%Max Horizontal Water Particle Velocity
a=1;
u_velocity= (a*w)*((cosh(k*z+k*h))/(sinh(k*h)))*cos(k*x-w*T)

%%Max Vertical Water Particle Velocity
w_velocity= (a*w)*((sinh(k*z+k*h))/(sinh(k*h)))*sin(k*x-w*T)

%%Max Horizontal Water Particle Acceleration
Acc_x=(a*g*k)*((cosh(k*z+k*h)/(cosh(k*h))))*sin(k*x-w*T)

%%Max Vertical Water Particle Acceleration
Acc_y= -a*g*k*((sinh(k*z+k*h))/(cosh(k*h)))*cos(k*x-w*T)

%%Max Horizontal Water Particle Displacement
x_displacement = ((H/2)*((cos(k*(h+z)))/sinh(k*h)))*sin(k*x-w*T)

%%Max Horizontal Water Particle Displacement
y_displacement = ((H/2)*((sinh(k*(h+z)))/sinh(k*h)))*cos(k*x-w*T)