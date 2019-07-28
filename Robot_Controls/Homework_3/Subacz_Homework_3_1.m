%% Subacz Homework 3
%% Problem 1
%%
% 
% <<Problem_1.PNG>>
% 
%%
% 
% <<P1.PNG>>
% 

%% Problem 2
%%
% 
% <<Problem_2.PNG>>
% 
%%
% 
% <<P2.PNG>>
% 

%% Problem 3
%%
% 
% <<Problem_3.PNG>>
% 
%%
% 
% <<P3.PNG>>
% 
syms l_1 l_2 l_c1 l_c2 m_1 m_2 I_1 I_2 g q_1 q_2 I_1 I_2 t

%Import the solution for the mass and corolis matrics from section 5.3.2
M_11 = m_1*l_c1^2 + m_2*(l_1^2 + l_c2^2 + (2*l_1*l_c2 * cos(q_2))) + I_1 + I_2;
M_12 = m_2*(l_c2^2 + l_1*l_c2*cos(q_2)) + I_2;
M_21 = m_2*(l_c2^2 + l_1*l_c2*cos(q_2)) + I_2;
M_22 = m_2*l_c2^2 + I_2;

C_11 = -1*m_2*l_1*l_c2*sin(q_2)*diff(q_2);
C_12 = -1*m_2*l_1*l_c2*sin(q_2)*(diff(q_1) + diff(q_2));
C_21 = m_2*l_1*l_c2*sin(q_2)*diff(q_1);
C_22 = 0;

%Contruct the M and C matrices
M_q = [M_11 M_12
    M_21 M_22];

C = [C_11 C_12
    C_21 C_22];

%Next take derivative of M.
d_Mq = diff(M_q);

%Skew symmetric satisfies the condition A^T = ?A. In terms of the entries
%of the matrix, if aij denotes the entry in the i?th row and j?th column;
%i.e., A = (aij), then the skew-symmetric condition is aji = ?aij. 
%Source: https://en.wikipedia.org/wiki/Skew-symmetric_matrix

%Construct the A matrix
A = (d_Mq.*0.5 - C);

%Finally, check for eqaulity to satify skew syemtric matrix conditions
%explained above. 
isSymetric = isequaln(simplify(transpose(A)), simplify(-A));

if isSymetric
    disp('The matrix is skew symmetric')
else
    disp('The matrix is not skew symmetric')
end
%% Problem 4-A,B,C,D
%%
% 
% <<P4.PNG>>
% 
%% Problem 4-E
% I forgot to write down the anwser before scanning... but:
% The equations developed in parts A,B,C,D show a linear relationship for
% the state eqautions. As seen in the lectures and the developed equations
% the gravity term contained the non linear terms. Since these  terms have 
% been elimated from the equation, the function now becomes linear. 

%% Problem 4-F,G,H
%%
% 
% <<P5.PNG>>
% 

