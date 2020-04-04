% FUNCTION NAME: H = dhFwdKine(linkList, paramList) creates Transformation
% matrix
% 
%  H = dhFwdKine(linkList, paramList) finds the transformation matrix usind
%  forward kinematics
% 
% H = Transformation Matrix
% 
% linkList = link patrameters
% paramList = variying parameters
% 
% Deepak Rajasekhar Karishetti
% 10846936
% MEGN544 
% 06-11-2018

function H = dhFwdKine(linkList,paramList)
paramList1=paramList';
H=eye(4);
for m=1:length(linkList)
    if linkList(1,m).isRotary==1
        linkList(1,m).theta=paramList1(m);
    else
        linkList(1,m).d=paramList1(m);
    end
H=H*dhTransform(linkList(1,m).a,linkList(1,m).d,linkList(1,m).alpha,linkList(1,m).theta);
end
end