% FUNCTION NAME: transform2Twist %Conversion of Transformation Matrix to Twist vector 
% 
%  t = transform2Twist( H ) Converts the Transformation Matrix to Twist vector [v;omega].
%
%
% H = Transformation Matrix
%
% t = Twist Vector 
% 
% Deepak Rajasekhar Karishetti
% 10846936
% MEGN544 
% 09-20-2018

function t = transform2Twist( H )
R(1:3,1:3) = H(1:3,1:3);
d(1:3,1) = H(1:3,4);
[k, theta] = rot2AngleAxis(R);
if theta==0
    v=d;
    omega=[0;0;0];
else
omega=k*theta;
v = ((((sin(theta))/(2*(1-cos(theta))))*eye(3)) + ((((2*(1-cos(theta)))-(theta*sin(theta)))/((2*theta)*(1-cos(theta))))*k*k') - (0.5*cpMap(k)))*d;
end
t=[v;omega];
end