% FUNCTION NAME: rotX %Rotation Matrix for Rotation along X-axis
% 
%  [R] = rotX(theta) using the convention below, the matrix
%  rotates points in the yz-plane/x-axis counterclockwise through an 
%  angle ? about the origin of the Cartesian coordinate system.
% 
% R = Rotation Matrix for rotation along X.  
% 
% theta = Angle to which it is Rotated 
% 
% Deepak Rajasekhar Karishetti
% 10846936
% MEGN544 
% 09-20-2018

function [R] = rotX(theta)
R=[1 0 0;0 cos(theta) -sin(theta);0 sin(theta) cos(theta)];
end