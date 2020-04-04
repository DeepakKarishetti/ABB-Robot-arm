% FUNCTION NAME: rot2AngleAxis %Conversion of Rotation Matrix to Angle-Axis
% 
%  [k, theta] = rot2AngleAxis(R) Converts the Rotation Matrix to respective angle and axis using inverse solution.
% 
%
% k = Axis of rotation
% theta= angle of rotation
%
% R = Rotation Matrix 
% 
% Deepak Rajasekhar Karishetti
% 10846936
% MEGN544 
% 09-20-2018

function [k, theta] = rot2AngleAxis(R)
k=zeros(3,1);
A = [(R(3,2)-R(2,3));(R(1,3)-R(3,1));(R(2,1)-R(1,2))];
B = R(1,1)+R(2,2)+R(3,3);
theta = atan2(0.5*norm(A),(B-1)/2);
if(cos(theta)==-1)
    k(1,1) = sqrt((R(1,1)+1)/2);
    k(2,1) = -sqrt((R(2,2)+1)/2);
    k(3,1) = -sqrt((R(3,3)+1)/2);
else
k = (1/(2*sin(theta)))*A;
end