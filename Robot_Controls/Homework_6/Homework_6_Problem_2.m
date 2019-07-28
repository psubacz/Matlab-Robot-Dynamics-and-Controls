%% Subacz, RBE502, Homework 6, Problem 2
function Compliance_Control()
    %Assumptions: The force applied by to the end effector is a constant.
    %the tip forces springs to 0 for a few seconds, this is due to the tip
    %boun
    
    clc,close all,clear all
    Tf = [0,15] ; %Run time
     

    %Calculate the vo
    
    %Starting Coordinates
    x = 0.3;
    y = 0.45;

    %Calculate the inverse kinematics
    a_1 = 0.3;
    a_2 = 0.3;
    q_2 = acos((x^2+y^2-a_1^2-a_2^2)/(2*a_1^2));
    q_1 = atan2(y,x)-atan2((a_2*sin(q_2)),(a_1+a_2*cos(q_2)));
    q_d = [q_1;
           q_2];
    
    %Initial ode45  input format        
    X0 =[x; %%joint angle1
         y; %%joint angle2
         q_1; %joint velocity1
         q_2; %joint velocity2
         0; %Computed joint velocity1
         0; %Computed joint velocity1
         0; %torque 1
         0; %torque 2
         0; %joint error 1 
         0; %%joint error 2
         0; %%position error 1
         0
         ];%%position error 2
    
    global tip_forces
    tip_forces = [0 0];

    [T,X] = ode45(@(t,x)plannarArmODE(t,x),Tf,X0) ;

    %Plot - Joint velocities vs time.
    figure;
    plot(T,X(:,5),T,X(:,6));
    title('Joint Velocities vs Time')
    xlabel('Time')
    ylabel('dTheta');
    legend('q_1','q_2');

    %Plot - control input vs time.
    %Calculate t approximate derivative
    T1 = [0;diff(X(:,7))./diff(T)];
    T2 = [0;diff(X(:,8))./diff(T)];
    figure;
    plot(T,T1,T,T2) ;
    title('Control Input vs Time')
    xlabel('Time')
    ylabel('Torque')
    legend('tau_1' ,'tau_2')

    %Calculate the tip position over time
    figure;
    x_tip_pos = a_1*cos(X(:,3)) + a_2*cos(X(:,3)+X(:,4));
    y_tip_pos = a_1*sin(X(:,3)) + a_2*sin(X(:,3)+X(:,4));
    plot(T,x_tip_pos,T,y_tip_pos) ;
    title('End Effector Position')
    xlabel('Time')
    ylabel('Position')
    
    %end effector error over time
    figure;
    plot(T,X(:,11),T,X(:,12));
    title('Position Error')
    xlabel('Time')
    ylabel('Position Error')
    legend('Error_x','Error_y')

    %end-effector force as function of time
    figure;
    x = length(tip_forces(:,2))
    z = 15/x
    A = [z:z:15]; %generate time function cause t is wierd.
    plot(A,tip_forces(:,1),A,tip_forces(:,2))
    title('End Effector Force vs Time')
    xlabel('Time')
    ylabel('Force')
    legend('Pos_x','Pos_y')
    
    %Stickmodel animation - code from hw solution 4
    figure;
    subplot(2,1,1);
    plot(T,x_tip_pos,T,y_tip_pos);
    hh1(1)= line(T(1),x_tip_pos(1,1),'Marker','.','MarkerSize',20, ...
        'Color','b');
    hh1(2)= line(T(1),y_tip_pos(1,1),'Marker','.','MarkerSize',20, ...
        'Color',[0 .5 0]);
    xlabel('time(sec)'); ylabel('Position');    
    
    subplot(2,1,2);
    hh2 = line ([0,1*cos(X(1,1))],[0,1*sin(X(1,1))]);
    axis([-0.5 0.5 -0 0.5]);
    ht = title(sprintf('Time: %0.2 f sec\nPositon: %0.2 f \nAngle : %0.2f(Radian) %0.2f(Degree) '...
    , T(1), x_tip_pos(1,1), y_tip_pos(1,1),rad2deg(y_tip_pos(1,1))));
    xlabel('Stickmodel')
    
    %Loop through the positions of links 1 and 2 and diplay them 
    for i = 1:length(T)
        set(hh1(1),'XData' ,T(i),'YData' , x_tip_pos(i,1));
        set(hh1(2),'XData' ,T(i),'YData' , y_tip_pos(i,1));
        y_loc_1 = 0.3*sin(X(i,3));
        x_loc_1 = 0.3*cos(X(i,3));
        y_loc_2= 0.3*sin(X(i,3))+0.3 * sin(X(i,3)+X(i,4));
        x_loc_2= 0.3*cos(X(i,3))+0.3 * cos(X(i,3)+X(i,4));
        set(hh2(1), 'XData', [0,x_loc_1,x_loc_2],'YData', [0, y_loc_1, y_loc_2]);
        set(ht, 'String', sprintf('Time: %0.2f sec \nAngle: %0.2f(Radian) %0.2f(Degree) ',...
        [T(i), X(i,3), rad2deg(X(i,3))]));
        f = getframe(gcf);
        if i==1
            [mov(:,:,1,i),map]= rgb2ind(f.cdata,256,'nodither');
        else
            mov(:,:,1,i) = rgb2ind(f.cdata,map,'nodither');
        end
    end
    imwrite(mov,map, 'Stickmodel_Animation.gif', 'DelayTime',0,'LoopCount',inf);

    function dx = plannarArmODE(t,x)
        theta = [x(3),x(4)]; %joint angle
        dtheta = [x(5),x(6)]; %joint velocity
        K_p = [10, 0; 0, 10]; %Kp gain
        K_v = [10, 0; 0 10]; % kv gain

        %Calculate the mass matrix
        M_11 = (m_2*a_1*a_1 + 2*m_2*a_1* l_2 * cos(theta(2)) + m_1* l_1 * l_1 + m_2*l_2 *l_2 + I_1 + I_2 );
        M_12 = (m_2*( l_2 * l_2 + a_1* l_2 * cos (theta(2))) + I_2);
        M_21 = (m_2*( l_2 * l_2 + a_1* l_2 * cos (theta(2))) + I_2);
        M_22 = (m_2* l_2 * l_2 + I_2);
        M = [ M_11,M_12;
              M_21,M_22];
        
        %Calculate the coriolis matrix
        C_11 =(-m_2*a_1* l_2 * sin (theta(2))*2*dtheta(2));
        C_12 =(-m_2*a_1*l_2*sin(theta(2))*dtheta(2));
        C_21 =(m_2*a_1*l_2*sin(theta(2))*dtheta(1));
        C_22 =0;
        C = [C_11,C_12;
            C_21,C_22];
               
        %Calculate the position of end effector and dont let it go though
        %the wall!!! Equation of the wall given by:
        x_pos = 0.3 - 0.12*t;
        y_pos = 0.45;
        
        %Wall force applied to end effector
        fe = [0,0];
        K_px =  0.8;
        K_x =   0.6;
        xd_xr = 0.6;
        
        %If the end effector is at the wall, apply the force
        if x_pos < -.28 %apply the force.
            fe = [((K_px*K_x)/(K_px+K_x))*(xd_xr),0];
            %Dont let the controller punch a hole in the wall!!!
            if x_pos < -0.3
                x_pos = -0.3; 
            end
        end
        %log the tip forces, cant though this into the dx returnn becuase
        %wierdness
        tip_forces = [tip_forces; fe];
     
        
        %Calculate the inverse kinematics
        qd_2 = acos((x_pos^2+y_pos^2-a_1^2-a_2^2)/(2*a_1^2));
        qd_1 = atan2(y_pos,x_pos)-atan2((a_2*sin(qd_2)),(a_1+a_2*cos(qd_2)));
        q_d = [qd_1;
               qd_2];
        qdDot = [0;
                 0];
%         qdDDot = [0;0]; 
        
        %Calculate position/velocity errors
        e=q_d-[theta(1);theta(2)]; % position error
        de = qdDot - [dtheta(1);dtheta(2)]; % velocity error
        tau = K_p*e + K_v*de;
        posError1 = [a_1*cos(q_1) + a_2*cos(q_1+q_2);
                    a_1*sin(q_1) + a_2*sin(q_1+q_2)];
        posError2 = [a_1*cos(q_d(1)) + a_2*cos(q_d(1)+q_d(2));
                    a_1*sin(q_d(1)) + a_2*sin(q_d(1)+q_d(2))];       
        error = posError2-posError1;
               
        %Calculate joint velocities
        invM = inv(C);
        invMC = invM*C;
        qdot = inv(M)*(tau-C*[dtheta(1);dtheta(2)] - fe);
        
        dx = [qd_1 - x(3);%%joint angle1
              qd_2 - x(4);%%joint angle2
                     x(5); %joint velocity1
                     x(6); %joint velocity2
                  qdot(1); %Computed joint velocity1
                  qdot(2); %Computed joint velocity1
                   tau(1); %torque 1
                   tau(2); %torque 2
               q_d(1)-q_1; %joint error 1 
               q_d(2)-q_2; %joint error 2
                 error(1); %position error 1
                 error(2)]; %position error 2
    end
end
