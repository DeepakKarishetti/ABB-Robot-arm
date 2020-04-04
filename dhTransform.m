% FUNCTION NAME: dhTransform % Transformation Matrix for DH-Parameters 
% 
%  H = dhTransform(a, d, alpha, theta) Find the Transformation Matrix corresponding to DH Parameters.
%
%
% H = Transformation Matrix
%
% a = distance translated along X-axis
% d = distance translated along Z-axis
% alpha = angle rotated along X-axis
% theta = angle rotated along Z-axis
% 
% Deepak Rajasekhar Karishetti
% 10846936
% MEGN544 
% 09-20-2018

function H = dhTransform(a, d, alpha, theta)
trans_a = [1 0 0 a;0 1 0 0;0 0 1 0;0 0 0 1];
trans_d = [1 0 0 0;0 1 0 0;0 0 1 d;0 0 0 1];
rot_alpha = [1 0 0 0;0 cos(alpha) -sin(alpha) 0;0 sin(alpha) cos(alpha) 0;0 0 0 1];
rot_theta = [cos(theta) -sin(theta) 0 0;sin(theta) cos(theta) 0 0;0 0 1 0;0 0 0 1];
H=trans_d*rot_theta*trans_a*rot_alpha;
end