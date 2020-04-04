% rotationError - calculates and returns the position and rotation error as a angle-axis
% vector in the reference frame
%
%   error_vector = rotationError(Rot_desired,Rot_current) - calculates
%   and returns the position and rotation error as a angle-axis vector in the reference
%   frame
%   
%   error_vector = rotation error vector as angle-axis vector in the
%   reference frame
%   
%   Rot_current = current rotation in the reference frame
%   Rot_desired = desired rotation in the reference frame
%
%
% Deepak Rajasekhar Karishetti
% 10846936
% MEGN544
%   


function [error_vector] = transError(Td,Tc)
d_Des = Td(1:3,4);
d_Cur = Tc(1:3,4);
position_error = d_Des - d_Cur;
rot_error = rotationError(Td(1:3,1:3),Tc(1:3,1:3));
error_vector = [position_error;rot_error];
end