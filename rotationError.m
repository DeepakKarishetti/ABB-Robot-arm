% rotationError - calculates and returns the rotation error as a angle-axis
% vector in the reference frame
%
%   rot_error_vector = rotationError(Rot_desired,Rot_current) - calculates
%   and returns the rotation error as a angle-axis vector in the reference
%   frame
%   
%   rot_error_vector = rotation error vector as angle-axis vector in the
%   reference frame
%   
%   Rot_current = current rotation in the reference frame
%   Rot_desired = desired rotation in the reference frame
%
% Deepak Rajasekhar Karishetti
% 10846936
%   MEGN544
%   14-11-18
function rot_error_vector = rotationError(Rot_desired,Rot_current)
curRotDes = (Rot_current')*Rot_desired;
[kErr,angErr] = rot2AngleAxis(curRotDes);
rot_error_vector = Rot_current*(angErr.*kErr);
end