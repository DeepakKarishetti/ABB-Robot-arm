% FUNCTION NAME: rotZ %Rotation Matrix for Rotation along Z-axis
% 
%  [R] = rotZ(theta) using the convention below, the matrix
%  rotates points in the xy-plane/z-axis counterclockwise through an 
%  angle ? about the origin of the Cartesian coordinate system.
% 
% R = Rotation Matrix for rotation along Z.
% 
% theta = Angle to which it is Rotated 
% 
% Deepak Rajasekhar Karishetti
% 10846936
% MEGN544 
% 09-20-2018

function [R] = rotZ(theta)
R=[cos(theta) -sin(theta) 0;sin(theta) cos(theta) 0;0 0 1];
end