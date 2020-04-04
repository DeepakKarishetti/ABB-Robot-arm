% FUNCTION NAME: angleAxis2Rot %Conversion of Angle-Axis to Rotation Matrix
% 
%  R = angleAxis2Rot(k, theta) Converts the given angle axis to a rotaion
%  matrix form.
% 
% 
% R = Rotation Matrix 
%
% k = Axis of rotation
% theta= angle of rotation
% 
% Deepak Rajasekhar Karishetti
% 10846936 
% MEGN544 
% 09-20-2018

function R = angleAxis2Rot(k, theta)
R = (cos(theta)*eye(3)) + ((1-cos(theta)).*(k*k')) + (sin(theta)*cpMap(k));
end