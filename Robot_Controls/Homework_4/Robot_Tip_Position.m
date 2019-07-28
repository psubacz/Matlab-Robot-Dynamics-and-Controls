function [Position_Vector] = Robot_Tip_Position(Home_Frame)
%Returns the robot tip vector
Position_Vector = [Home_Frame(1,4), Home_Frame(2,4), Home_Frame(3,4)].';
end 