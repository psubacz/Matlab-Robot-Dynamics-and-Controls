%% Subacz, RBE502, Final,
function PD_Motion_Control_of_a_Walking_robot()
    %Assumptions:
    %Phase two motion can be broken into a 5 part piece wise function with
    %each funtion taking 1 second to complete. See CalculateIdealTrajectory
    %for function generation.
    %
    %h =0.075 for phase 2 motion and takes 1 second to complete
    %
    %H_L =0.15 for phase 2 motion and takes 2.5 second to reach the middle
    %point.
    %
    %All mathmatics are done is the negative quadrents to match the
    %template.
    %
    %The pelican robot is started in the elbow down configuration
    %
    %I believe the forward and inverse kinematics in the book it incorrect 
    %or they are using a different base frame because when i ran the 
    %the results or x and y were swapped. 
    %
    %Since the tip of the robot is not touching the ground, complaince
    %cotrol is ignored for this problem.
    %
    %
    %
    %% Problem 1A
    %The controller used in this application is the PD control law. The
    %assocaited equation with the PD controller is tau = k_p*q+K_v +q_dot+g(q) 
    %from lecture L6-P3. When you close the loop with the gravity term as 
    %part of tau. the gravity term cancels out in the dynamic model. 
    %See lecture 6 for derivation
    %
     
    clc,close all,clear all
    Tf = [0,15] ; %Run time
    
    Generate_Plots = true;
    %set to true to display the animation and create new gif
    run_anaimation = false;
    
    %Parameters for the 2 link planar arm. These can be found on page 115
    %of kelly's book
    l_1 = 0.26;
    l_2 = 0.26;
    l_c1 = 0.0983;
    l_c2 = 0.0229;
    m_1 = 6.5225;
    m_2 = 2.0458;
    I_1 = 0.1213;
    I_2 = 0.0116;
    g = 9.81;
    
    %Starting Coordinates
    x = 0.1;
    y = -0.3;
    
    %Inverse Kinematics
    qd_2 = acos((x^2+y^2-l_1^2-l_2^2)/(2*l_1^2));  
    qd_1 = atan2(y,x)-atan2((l_2*sin(qd_2)),(l_1+l_2*cos(qd_2)));

    %Forward Kinematics
    x_pos = l_1*cos(qd_1)+l_2*cos(qd_1+qd_2);
    y_pos = l_1*sin(qd_1)+l_2*sin(qd_1+qd_2);

    %Initial ode45  input format        
    X0 =[x; %%joint x pos
         y; %%joint y pos
         qd_1; %joint velocity1
         qd_2; %joint velocity2
         0; %Computed joint velocity1
         0; %Computed joint velocity1
         0; %torque 1
         0; %torque 2
         0; %joint error 1 
         0; %%joint error 2
         0; %%position error 1
         0;  %%position error 2
         0;
         0];

    [T,X] = ode45(@(t,x)plannarArmODE(t,x),Tf,X0);
    if Generate_Plots
        %%Problem 1B - Plot: Joint Angles vs Time.
        figure;
        plot(T,X(:,3),T,X(:,4));
        title('Joint Angles vs Time')
        xlabel('Time')
        ylabel('Angle (radians)');
        legend('q_1','q_2');
        hold on

        %%Problem 1C - Plot: Joint Velocities vs Time.
        figure;
        plot(T,X(:,5),T,X(:,6));
        title('Joint Velocities vs Time')
        xlabel('Time')
        ylabel('dTheta');
        legend('q_1','q_2');
            hold on

        %%Problem 1D - Plot: Control Input vs Time.
        %Calculate t approximate derivative
        T1 = [0;diff(X(:,7))./diff(T)];
        T2 = [0;diff(X(:,8))./diff(T)];
        figure;
        plot(T,T1,T,T2) ;
        title('Control Input vs Time')
        xlabel('Time')
        ylabel('Torque')
        legend('tau_1' ,'tau_2')

        %%Problem 1E - Plot: End Effector path overtime.
        %Calculate the tip position over time
        x_pos_act_phase_1 = [];
        y_pos_act_phase_1= []; 
        for t = 0:0.001:10
            [x,y] = CalculateIdealTrajectory(t);
            x_pos_act_phase_1 = [x_pos_act_phase_1,x];
            y_pos_act_phase_1 = [y_pos_act_phase_1,y];
        end
        x_pos_act_phase_2 = [];
        y_pos_act_phase_2= []; 
        for t = 10:0.001:15
            [x,y] = CalculateIdealTrajectory(t);
            x_pos_act_phase_2 = [x_pos_act_phase_2,x];
            y_pos_act_phase_2 = [y_pos_act_phase_2,y];
        end
        figure;
        x_tip_pos = l_1*cos(X(:,3)) + l_2*cos(X(:,3)+X(:,4));
        y_tip_pos = l_1*sin(X(:,3)) + l_2*sin(X(:,3)+X(:,4));
        plot(x_tip_pos,y_tip_pos,'g',...
            x_pos_act_phase_1,y_pos_act_phase_1,':r',...
            x_pos_act_phase_2,y_pos_act_phase_2,':b') ;
        title('End Effector Path ')
        xlabel('X Position')
        ylabel('Y Position')
        legend('Tip Position Path','Phase 1 Path','Phase 2 Path')
        axis([-0.15 0.15 -0.32 -0.13]);

        %%Problem 1F - Plot: Plot: End Effector position overtime.
        figure;
        plot(T,x_tip_pos,'r',T,y_tip_pos,'b') ;
        title('End Effector Position')
        xlabel('Time')
        ylabel('Position (m)')
        legend('x position','y position')

        %%Problem 1G - Plot: End Effector Velocity vs time.
        figure;
        subplot(2,1,1);
        plot(diff(x_tip_pos)./diff(T)) ;
        title('End Effector Horzontal Velocity')
        xlabel('Time')
        ylabel('Velocity(m/s)')
        subplot(2,1,2);
        plot(diff(y_tip_pos)./diff(T),'b') ;
        title('End Effector Vertical Velocity')
        xlabel('Time')
        ylabel('Velocity(m/s)')
    end
    %%Problem 1H - Animation: of end Effector
    %Stickmodel animation - code from hw solution 4
    if run_anaimation
        x_tip_pos = l_1*cos(X(:,3)) + l_2*cos(X(:,3)+X(:,4));
        y_tip_pos = l_1*sin(X(:,3)) + l_2*sin(X(:,3)+X(:,4));
        x_pos_act_phase_1 = [];
        y_pos_act_phase_1= []; 
        for t = 0:0.001:10
            [x,y] = CalculateIdealTrajectory(t);
            x_pos_act_phase_1 = [x_pos_act_phase_1,x];
            y_pos_act_phase_1 = [y_pos_act_phase_1,y];
        end
        x_pos_act_phase_2 = [];
        y_pos_act_phase_2= []; 
        for t = 10:0.001:15
            [x,y] = CalculateIdealTrajectory(t);
            x_pos_act_phase_2 = [x_pos_act_phase_2,x];
            y_pos_act_phase_2 = [y_pos_act_phase_2,y];
        end
        figure;
        subplot(2,1,1);
        plot(T,x_tip_pos,T,y_tip_pos);
        hh1(1)= line(T(1),x_tip_pos(1,1),'Marker','.','MarkerSize',20, ...
            'Color','b');
        hh1(2)= line(T(1),y_tip_pos(1,1),'Marker','.','MarkerSize',20, ...
            'Color',[0 .5 0]);
        xlabel('time(sec)'); ylabel('Position');    
        legend('x position','y position')

        subplot(2,1,2);
        plot(x_pos_act_phase_1,y_pos_act_phase_1,':r',...
            x_pos_act_phase_2,y_pos_act_phase_2,':b');
        hh3(3) = line(x_tip_pos(1,1),y_tip_pos(1,1),'Marker','.','MarkerSize',10, ...
            'Color',[0 .5 0]);
        hh2 = line ([0,1*cos(X(1,3))],[0,1*sin(X(1,3))]);
        ht = title(sprintf('Time: %0.2 f sec\nPositon: %0.2 f \nAngle : %0.2f(Radian) %0.2f(Degree) '...
        , T(1), x_tip_pos(1,1), y_tip_pos(1,1),rad2deg(y_tip_pos(1,1))));
        xlabel('Stickmodel')
        legend('Phase 1 Path','Phase 2 Path')
        axis([-0.2 0.2 -0.35 0.1]);

        %Loop through the positions of links 1 and 2 and diplay them 
        for i = 1:length(T)
            set(hh1(1),'XData' ,T(i),'YData' , x_tip_pos(i,1));
            set(hh1(2),'XData' ,T(i),'YData' , y_tip_pos(i,1));
            set(hh3(3),'XData' ,x_tip_pos(i,1),'YData' , y_tip_pos(i,1));
            y_loc_1 = l_1*sin(X(i,3));
            x_loc_1 = l_1*cos(X(i,3));
            y_loc_2= l_1*sin(X(i,3))+l_2*sin(X(i,3)+X(i,4));
            x_loc_2= l_1*cos(X(i,3))+l_2*cos(X(i,3)+X(i,4));
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
    end

    function dx = plannarArmODE(t,x)
        theta = [x(3),x(4)];  %joint angle
        dtheta = [x(5),x(6)]; %joint velocity

        %Calculate the mass matrix
        M_11 = (m_1*l_c1^2 + m_2*(l_1^2+l_c2^2+2*l_1*l_c2*cos(theta(2)))+ I_1 + I_2 );
        M_12 = (m_2*(l_c2^2 +l_1*l_c2*cos(theta(2))) + I_2);
        M_21 = (m_2*(l_c2^2 +l_1*l_c2*cos(theta(2))) + I_2);
        M_22 = (m_2*l_c2^2+ I_2);
        M = [ M_11,M_12;
              M_21,M_22];
        
        %Calculate the coriolis matrix
        C_11 =(-m_2*l_1*l_c2*sin(theta(2))*dtheta(2));
        C_12 =(-m_2*l_1*l_c2*sin(theta(2))*(dtheta(1)+dtheta(2)));
        C_21 =(m_2*l_1*l_c2*sin(theta(2))*dtheta(1));
        C_22 =0;
        C = [C_11,C_12;
            C_21,C_22];
        
        %Calculate the gravity matrix
        G_11 =(m_1*l_c1+m_2+l_1)*g*sin(theta(1))+m_2*l_c2*g*sin(theta(1)+theta(2));
        G_21 =(m_2*l_c2*g*sin(theta(1)+theta(2)));
        G = [G_11;
             G_21];

        if t<15
            [x_pos,y_pos] = CalculateIdealTrajectory(t);
        else
            t = t-15;
            [x_pos,y_pos] = CalculateIdealTrajectory(t);
        end

        %Calculate the inverse kinematics        
        qd_2 = acos((x_pos^2+y_pos^2-l_1^2-l_2^2)/(2*l_1^2));  
        qd_1 = atan2(y_pos,x_pos)-atan2((l_2*sin(qd_2)),(l_1+l_2*cos(qd_2)));

        q_d = [qd_1;
               qd_2];
        qdDot = [0;
                 0];
        
        %Calculate tau
        [tau,e] = PDControl(theta,dtheta,q_d,qdDot);
         
        posError1 = [l_1*cos(theta(1)) + l_2*cos(theta(1)+theta(2));
                     l_1*sin(theta(1)) + l_2*sin(theta(1)+theta(2))];
        posError2 = [l_1*cos(q_d(1))   + l_2*cos(q_d(1)+q_d(2));
                     l_1*sin(q_d(1))   + l_2*sin(q_d(1)+q_d(2))];       
        error = posError2-posError1;
               
        %Calculate joint velocities
        qdot = inv(M)*(tau-C*[dtheta(1);dtheta(2)]);
        
        dx = [qd_1 - x(3);%%joint error angle1
              qd_2 - x(4);%%joint error angle2
                     x(5); %joint velocity1
                     x(6); %joint velocity2
                  qdot(1); %Computed joint velocity1
                  qdot(2); %Computed joint velocity1
                   tau(1); %torque 1
                   tau(2); %torque 2
                     e(1); %joint error 1 
                     e(2); %joint error 2
                 error(1); %position error 1
                 error(2); %position error 2
                 dtheta(1);%
                 dtheta(2);];
    end

    function [tau,e] = PDControl(theta,dtheta,q_d,qdDot)
        K_p = [200, 0;
               0, 200]; %Kp gain
        K_v = [7, 0;
               0 5];  %Kv gain
        e=q_d-[theta(1);theta(2)]; % position error
        de = qdDot - [dtheta(1);dtheta(2)]; % velocity error
        tau = K_p*e + K_v*de;
    end

    function [x1,y1] = CalculateIdealTrajectory(t)   
        if t<10
            x1 = 0.1 - t*2*0.01;
            y1 = -0.3;
        elseif t>=10 && t<11 
            %p1
            x1 = -0.1;
            y1 = -0.3+(t-10)*0.075;
        elseif t>=11 && t<12 
            %curve y to match graph
            x1 = -0.1+(t-11)*0.066;
            dt = -(1-(t-11));
            y1 =((dt*.415).^3)-0.15;
        elseif t>=12 && t<13 
            %p3 
            x1 = -0.034+(t-12)*0.066;
            y1 = -0.15;
        elseif t>=13 && t<14
            %curve y to match graph
            x1 = 0.0320+(t-13)*0.066;
            dt = t-13;
            a = -1;
            y1 =(a*(dt*.415).^3)-0.15;
        elseif t>=14 && t<=15 
            x1 = 0.1;
            y1 = -0.2317-(t-14)*0.0688;
        end
    end
end
