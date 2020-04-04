% FUNCTION NAME: dhInvKine(linkList,desTransform,paramListGuess) finds the inverse kinematics 
% 
%  [paramList,error] = dhInvKine(linkList,desTransform,paramListGuess) using newton-Raphson method
% 
% linkList= link parameters
% desTransform = desired Transform
% paramListGuess = initial condition
%
% paramList = parameter list
% error = error obtained
% 
% Deepak Rajasekhar Karishetti
% 10846936
% MEGN544 
% 14-11-2018

function [paramList,error] = dhInvKine(linkList,desTransform,paramListGuess)
coder.extrinsic('exist')
paramList = paramListGuess;
for i = 1:10
    T = dhFwdKine(linkList,paramList);
    error = transError(desTransform,T);
    [Jv,~] = velocityJacobian(linkList,paramList);
    [U,S,V] = svd(Jv);
    Sinv = S';
    Sinv(Sinv/Sinv(1,1) > sqrt(eps)) = 1./Sinv(Sinv/Sinv(1,1) > sqrt(eps));
    Sinv(1./Sinv < sqrt(eps)) = 0;
    Jv_inv = (V)*Sinv*(U');
    
    diff = Jv_inv*error;
    paramList = paramList + diff;
    T = dhFwdKine(linkList,paramList);
    error = transError(desTransform,T);
    error = norm(error);
end
end