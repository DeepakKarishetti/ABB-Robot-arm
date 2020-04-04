% FUNCTION NAME: twist2Transform %Conversion of Twist vector to Transformation Matrix
% 
%  H = twist2Transform( t ) Converts the Twist vector [v;omega] to Transformation Matrix.
%
% t = Twist Vector 
%
% H = Transformation Matrix
%
% 
% Deepak Rajasekhar Karishetti
% 10846936 
% MEGN544 
% 09-20-2018

function H = twist2Transform( t )
omega = t(4:6,1);
v = t(1:3,1);
theta = norm(omega);
if theta==0
    R=eye(3);
    d=v;
else
omega = omega/theta;
R = (cos(theta)*eye(3)) + (sin(theta)*cpMap(omega)) + ((1-cos(theta))*omega*omega');
d = (eye(3) - R)*cpMap(omega)*v + theta*omega*omega'*v;
end
H = [R d;0 0 0 1];
end