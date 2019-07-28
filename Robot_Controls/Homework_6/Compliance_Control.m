%% Subacz, RBE502, Homework 6, Problem 2
function Compliance_Control()
    %Assumptions: The force applied by to the end effector is a constant. 
    clc,close all,clear all
    Tf = [0,20] ; %Run time
  
    %Starting Coordinates
    x = 0.3
    y = 0.45
    
    %Parameters for the 2 link planar arm.
    g = 9.8 ;
    a_1 = 0.3 ;
    a_2 = 0.3 ;
    l_1 = 0.15 ;
    l_2 = 0.15 ;
    m_1 = 0.05 ;
    m_2 = 0.05 ;
    I_1 = 1;
    I_2 = 1;
    
    %Calculate the inverse kinematics
    a_1 = 0.3;
    a_2 = 0.3;
    q_2 = acos((x^2 + y^2 - a_1^2 - a_2^2)/(2*a_1^2)) ;
    q_1 = atan2 (y,x) - atan2((a_2* sin(q_2)),(a_1+a_2*cos(q_2))) ;
    q_d = [q_1;q_2];
    
    % % Initial ode45  input
    X0 =[x; y; q_1; q_2; 0; 0; 0; 0; 0; 0; 0;0]
    
    global prev_t; %force exerted
    prev_t = 0;
    global g_fe;
    g_fe = [0 0];
    global g_t;
    g_t = [0];

    [T, X] = ode45(@(t,x)plannarArmODE(t,x),Tf,X0) ;

    %Plot - Joint velocities vs time.
    figure(1);
    plot(T,X(:,5),T,X(:,6));
    title('Joint Velocities vs Time')
    xlabel('time')
    ylabel('d_theta');
    legend('q_1','q_2');

    %Plot - control input vs time.
    %Calculate t approximate derivative
    T1 = [0;diff(X(:,7))./diff(T)];
    T2 = [0;diff(X(:,8))./diff(T)];
    figure(2);
    plot(T,T1,T,T2) ;
    title('Control Input vs Time')
    xlabel('time')
    ylabel('torque')
    legend('tau_1' ,'tau_2')

    %Calculate the tip position over time
    figure(3);
    for i = 1 : length(X)    
        x = a_1*cos(X(i,3)) + a_2*cos(X(i,3)+X(i,4));
        y = a_1*sin(X(i,3)) + a_2*sin(X(i,3)+X(i,4));
        tip_Pos(i,:) = [x; y];
    end
    plot(T, tip_Pos(:,1),T,tip_Pos(:,2)) ;
    title('End Effector Position')
    xlabel('time')
    ylabel('pos (m)')
    
    %end effector error over time
    figure(4)
    plot(T,X(:,11),T,X(:,12));
    title('Position Error')
    xlabel('time')
    ylabel('pos error')
    legend('x_error','y_error')

    %end-effector force as function of time
    figure(5);
    plot(g_t,g_fe(:,1))
    hold on; plot(g_t,g_fe(:,2))
    title('end effector force')
    xlabel('time')
    ylabel('force')
    legend('x','y')

    %stick model.
    figure(7);
    clf('reset');
    subplot(2,1,1);
    plot(T,tip_Pos(:,1),T,tip_Pos(:,2));
    hh1(1)= line(T(1),tip_Pos(1,1),'Marker','.','MarkerSize',20,'Color','b');
    hh1(2)= line(T(1),tip_Pos(2,1),'Marker','.','MarkerSize',20,'Color',[0 .5 0]);
    xlabel('time(sec)'); ylabel('Position');
    
    subplot(2,1,2);
    hh2 = line ([0,1*cos(X(1,1))],[0,1*sin(X(1,1))]);
    axis equal;
    axis([-0.5 0.5 -0 0.5]);
%     ht = title(sprintf('Time: %0.2 f sec\nPositon: %0.2 f \nAngle : %0.2f(Radian) %0.2f(Degree) '...
%         , T(1), tip_Pos(1,1), tip_Pos(2,1),rad2deg(tip_Pos(2,1))));

    xlabel('Stickmodel')

    for i = 1:length(T)
        set(hh1(1),'XData' ,T(i),'YData' , tip_Pos(i,1));
        set(hh1(2),'XData' ,T(i),'YData' , tip_Pos(i,2));
        tempy1 = 0.3*sin(X(i,3));
        tempx1 = 0.3*cos(X(i,3));
        tempy2= 0.3*sin(X(i,3))+0.3 * sin(X(i,3)+X(i,4));
        tempx2= 0.3*cos(X(i,3))+0.3 * cos(X(i,3)+X(i,4));
        set(hh2(1), 'XData', [0,tempx1,tempx2],...
            'YData', [0, tempy1, tempy2]);
%         set(ht, 'String', sprintf('Time: %0.2f sec \nAngle: %0.2f(Radian) %0.2f(Degree) ',...
%             [T(i), X(i,3), rad2deg(X(i,3))]));
        f = getframe(gcf);

        if i==1
            [mov(:,:,1,i),map]= rgb2ind(f.cdata,256,'nodither');
        else
            mov(:,:,1,i) = rgb2ind(f.cdata,map,'nodither');
        end
    end
    imwrite(mov,map, 'strickAnimation.gif', 'DelayTime',0,'LoopCount',inf);

    function dx = plannarArmODE(t,x)
        %space-state variables
        theta = [x(3),x(4)];
        dtheta = [x(5),x(6)];
        
        %Gains
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
        
        %Calculate the gravity matrix
%         G_1 =((m_1*l_1+m_2*a_1)*g*cos(theta(1))+m_2*l_2*g*cos(theta(1)+theta(2)));
%         G_2 = (m_2* l_2 *g* cos(theta(1)+theta(2)));
%         G = [G_1;
%              G_2];
        
        %Calculate the position of end effector and dont let it go though
        %the wall!!!
        x_pos = 0.3 - 0.12*t;
        y_pos = 0.45;
        
        %Wall force applied to end effector
        fe = [0;0];
        Kpx = 1;
        Kx = 1;
        xd_xr = 1;
        
        %If the end effector is at the wall, apply the force
        if x_pos < -.28 %apply the force.
            fe = [((Kpx*Kx)/(Kpx+Kx))*(xd_xr);
                       0];
            %Dont let the controller punch a hole in the wall!!!
            if x_pos < -0.3
                x_pos = -0.3; 
            end
        end
        
        if prev_t < t
            g_fe = [g_fe; fe'];
            g_t = [g_t; t];
            prev_t = t;
        else
            prev_t = t;
        end        
        
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
        
        dx = [qd_1 - x(3);
              qd_2 - x(4);
                     x(5);
                     x(6);
                  qdot(1);
                  qdot(2);
                   tau(1);
                   tau(2);
               q_d(1)-q_1;
               q_d(2)-q_2;
                 error(1);
                 error(2)];
    end
end
