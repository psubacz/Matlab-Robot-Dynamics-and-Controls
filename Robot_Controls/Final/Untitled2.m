
    %Parameters for the 2 link planar arm.
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

    rad2deg(qd_1)
    rad2deg(qd_2)
    
    %Forward Kinematics
    x_pos = l_1*cos(qd_1)+l_2*cos(qd_1+qd_2);
    y_pos = l_1*sin(qd_1)+l_2*sin(qd_1+qd_2);
    