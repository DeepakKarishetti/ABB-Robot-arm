% FUNCTION NAME: rpy2Rot %Rotation Matrix for Rotation along
% Z(roll),Y(pitch),Z(yaw).
% 
%  [R_y] = rotY(theta) using the convention below, the matrix
%  rotates points in the z-plane counterclockwise through an 
%  angle yaw and in the y-plane counterclockwise through an 
%  angle pitch and in the x-plane counterclockwise through an 
%  angle roll and about the origin of the Cartesian coordinate system.
% 
% R = Rotation Matrix for rotation along Z and then along Y and then along X.  
% 
% yaw = Angle to which it is Rotated along Z
% pitch = Angle to which it is Rotated along Y
% roll = Angle to which it is Rotated along X
% 
% Deepak Rajasekhar Karishetti
% 10846936
% MEGN544 
% 09-20-2018

function [R] = rpy2Rot (roll, pitch, yaw) 
A=rotX(roll);
B=rotY(pitch);
C=rotZ(yaw);
R=C*B*A;
end