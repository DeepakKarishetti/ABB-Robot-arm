% FUNCTION NAME: rotY %Rotation Matrix for Rotation along Y-axis
% 
%  [R] = rotY(theta) using the convention below, the matrix
%  rotates points in the xz-plane/y-axis counterclockwise through an 
%  angle ? about the origin of the Cartesian coordinate system.
% 
% R = Rotation Matrix for rotation along Y.  
% 
% theta = Angle to which it is Rotated 
% 
% Deepak Rajasekhar Karishetti
% 10846936
% MEGN544 
% 09-20-2018

function [R] = rotY(theta)
R=[cos(theta) 0 sin(theta);0 1 0;-sin(theta) 0 cos(theta)];
end