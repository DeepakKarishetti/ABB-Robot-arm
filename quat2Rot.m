% FUNCTION NAME: quat2Rot %Conversion of Quaternions to Rotation Matrix
% 
%  R = quat2Rot(Q) Converts the quaternions q0 and q_vec to Rotation Matrix.
%
% R = Rotation Matrix 
%
% Q = Quaternions [q0;q_vec]
%
% 
% Deepak Rajasekhar Karishetti
% 10846936 
% MEGN544 
% 09-20-2018

function R = quat2Rot(Q)
q=zeros(3,1);
q_0 = Q(1);
q(1:3,1) = Q(2:4);
R = ((q_0^2)-(q'*q))*eye(3) + 2*q_0*cpMap(q) + 2*q*q';
end