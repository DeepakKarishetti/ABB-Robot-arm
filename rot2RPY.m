% FUNCTION NAME: rpy2Rot %Rotation Matrix for Rotation along
% Z(roll),Y(pitch),Z(yaw).
% 
%  [R_y] = rotY(theta) using the convention below, the matrix
%  rotates points in the z-plane counterclockwise through an 
%  angle roll and in the y-plane counterclockwise through an 
%  angle pitch and in the x-plane counterclockwise through an 
%  angle yaw and about the origin of the Cartesian coordinate system.
% 
% 
% yaw = Angle to which it is Rotated along Z
% pitch = Angle to which it is Rotated along Y
% roll = Angle to which it is Rotated along X 
%
% R = Rotation Matrix for rotation along Z and then along Y and then along X.  
% 
% Deepak Rajasekhar Karishetti
% 10846936
% MEGN544 
% 09-20-2018

function [roll,pitch,yaw] = rot2RPY(R)

pitch(1,1) = atan2(-R(3,1),sqrt((R(3,2)^2)+(R(3,3)^2)));
pitch(2,1) = atan2(-R(3,1),-sqrt((R(3,2)^2)+(R(3,3)^2)));
if sin(pitch(1,1))== 1 || sin(pitch(2,1))== 1 
    yaw(1,1)=0;
    roll(1,1)=-atan2(-R(1,2),R(2,2));
    yaw(2,1)=0;
    roll(2,1)=-atan2(-R(1,2),R(2,2));
elseif sin(pitch(1,1))== -1 || sin(pitch(2,1))== -1 
    yaw(1,1)=0;
    roll(1,1)=atan2(-R(1,2),R(2,2));
    yaw(2,1)=0;
    roll(2,1)=atan2(-R(1,2),R(2,2));
else
roll(1,1)=atan2((R(3,2)/cos(pitch(1,1))),(R(3,3)/cos(pitch(1,1))));
roll(2,1)=atan2((R(3,2)/cos(pitch(2,1))),(R(3,3)/cos(pitch(2,1))));
yaw(1,1)=atan2((R(2,1)/cos(pitch(1,1))),(R(1,1)/cos(pitch(1,1))));
yaw(2,1)=atan2((R(2,1)/cos(pitch(2,1))),(R(1,1)/cos(pitch(2,1))));
end
roll=roll(1:2,1);
pitch=pitch(1:2,1);
yaw=yaw(1:2,1);
end