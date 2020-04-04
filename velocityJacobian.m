% FUNCTION NAME: velocityJacobian( linkList, paramList, paramRateList) to
% find Jv,JvDot
% 
%  [Jv, JvDot]= velocityJacobian( linkList, paramList, paramRateList) finds
%  the Jacobian and Jacobian dot matrices
% 
% Jv = Jacobian Matrix
% JvDot = Jacobian_Dot Matrix
% 
% linkList = Link parameters
% paramList = varying parameters list
% paramRateList = differentiation of varying parameters
% 
% Deepak Rajasekhar Karishetti
% 10846936
% 
function [Jv, JvDot]= velocityJacobian(linkList, paramList, paramRateList) 
Jv=zeros(6,6);
JvDot=zeros(6,6);
omega=zeros(3,6);
dd_0i=zeros(3,6);

coder.extrinsic('exist')

H=dhFwdKine(linkList(1,:),paramList);
if exist('paramRateList','var')
    if linkList(1,1).isRotary==1
    Jv(:,1)=[cpMap([0;0;1])*(H(1:3,4));[0;0;1]];
    else
    Jv(:,1)=[[0;0;1];[0;0;0]];
    end
             for i=1:length(paramList)-1
                 Hi=dhFwdKine(linkList(1,1:i),paramList(1:i));
                 if linkList(1,i+1).isRotary==1
                 Jv(:,i+1) = [cpMap(Hi(1:3,1:3)*[0;0;1])*(H(1:3,4)-Hi(1:3,4));(Hi(1:3,1:3)*[0;0;1])];
                 else
                 Jv(:,i+1) = [(Hi(1:3,1:3)*[0;0;1]);[0;0;0]];
                 end
             end
             
             H1i=dhFwdKine(linkList(1,1),paramList(1));
             d_i1_1=H1i(1:3,4);
             if linkList(1).isRotary==1
             omega(:,1)=paramRateList(1)*[0;0;1];
             dd_0i(:,1)=(cpMap(omega(:,1))*d_i1_1);
             else
             omega(:,1)=[0;0;0];
             dd_0i(:,1)=(cpMap(omega(:,1))*d_i1_1)+(paramRateList(1)*[0;0;1]);
             end
             
%              dd_0N=Jv*paramRateList;

             for i=1:length(paramList)-1
             Hi=dhFwdKine(linkList(1,1:i),paramList(1:i));
             Hi1=dhFwdKine(linkList(1,1:i+1),paramList(1:i+1));
             d_i1_1 = Hi1(1:3,4)-Hi(1:3,4);
             z_i1=Hi(1:3,1:3)*[0;0;1];
             if linkList(1,i+1).isRotary==1
             omega(:,i+1)=omega(:,i)+paramRateList(i+1)*z_i1;
             dd_0i(:,i+1)=dd_0i(:,i)+(cpMap(omega(:,i+1))*d_i1_1);
             else
             omega(:,i+1)=omega(:,i);
             dd_0i(:,i+1)=dd_0i(:,i)+(cpMap(omega(:,i+1))*d_i1_1)+(paramRateList(i+1)*z_i1);
             end
             end
             
             dd_0N=dd_0i(:,end);
             
             if linkList(1,1).isRotary==1
             JvDot(:,1) = [cpMap(cpMap(omega(:,1))*[0;0;1])*(H(1:3,4))+(cpMap([0;0;1])*(dd_0N));cpMap(omega(:,1))*[0;0;1]];
             else
             JvDot(:,1) = [cpMap(omega(:,1))*[0;0;1];[0;0;0]];
             end
             
             for i=1:length(paramList)-1
                 Hi=dhFwdKine(linkList(1,1:i),paramList(1:i));
                 if linkList(1,i+1).isRotary==1
                 JvDot(:,i+1) = [(cpMap(cpMap(omega(:,i+1))*Hi(1:3,1:3)*[0;0;1])*(H(1:3,4)-Hi(1:3,4)))+(cpMap(Hi(1:3,1:3)*[0;0;1])*(dd_0N(1:3,1)-dd_0i(1:3,i)));cpMap(omega(:,i+1))*Hi(1:3,1:3)*[0;0;1]];
                 else
                 JvDot(:,i+1) = [cpMap(omega(:,i+1))*Hi(1:3,1:3)*[0;0;1];[0;0;0]];
                 end
             end
else
    if linkList(1,1).isRotary==1
    Jv(:,1)=[(cpMap([0;0;1])*(H(1:3,4)));[0;0;1]];
    else
    Jv(:,1)=[[0;0;1];[0;0;0]];
    end
             for i=1:length(paramList)-1
                 Hi=dhFwdKine(linkList(1,1:i),paramList(1:i));
                 if linkList(1,i+1).isRotary==1
                 Jv(:,i+1) = [cpMap(Hi(1:3,1:3)*[0;0;1])*(H(1:3,4)-Hi(1:3,4));Hi(1:3,1:3)*[0;0;1]];
                 else
                 Jv(:,i+1) = [Hi(1:3,1:3)*[0;0;1];[0;0;0]];
                 end
             end
             JvDot=zeros(6,6);
end
end