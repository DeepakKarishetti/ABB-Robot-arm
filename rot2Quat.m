% FUNCTION NAME: rot2Quat 
% Conversion of Rotation Matrix to Quaternions
% 
%  Q = rot2Quat(R) Converts the Rotation Matrix to quaternions q0 and q_vec.
% 
%
% Q = Quaternions [q0;q_vec]
%
% R = Rotation Matrix 
% 
% Deepak Rajasekhar Karishetti
% 10846936
% MEGN544 
% 09-20-2018

function Q = rot2Quat(R)
q_0 = 0.5*sqrt(1+(R(1,1)+R(2,2)+R(3,3)));
if q_0 == 0 
    q_1 = 0.5*sqrt((R(1,1)-R(2,2)-R(3,3))+1);
    q_2 = 0.5*sqrt((-R(1,1)+R(2,2)-R(3,3))+1);
    q_3 = 0.5*sqrt((-R(1,1)-R(2,2)+R(3,3))+1);
else
q_1 = (R(3,2)-R(2,3))/(4*q_0);
q_2 = (R(1,3)-R(3,1))/(4*q_0);
q_3 = (R(2,1)-R(1,2))/(4*q_0);
end
Q = [q_0;q_1;q_2;q_3];
end