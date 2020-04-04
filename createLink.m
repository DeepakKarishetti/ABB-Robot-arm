% FUNCTION NAME: createLink(a,d,alpha,theta,centOfMass,mass,inertia)
% creates a link.
% 
%  L = createLink(a,d,alpha,theta,centOfMass,mass,inertia) creates a link
%  with the input parameters and also says whether it is a rotary joint or
%  prismatic joint
% 
% L = link
% 
% a = joint parameter
% d = joint parameter
% theta = joint parameter
% alpha = joint parameter
% centOfMass = center of mass of the link
% mass = mass of the link
% inertia = inertia of the link
% 
% Deepak Rajasekhar Karishetti
% 10846936
% MEGN544 
% 06-11-2018

function L = createLink(a,d,alpha,theta,centOfMass,mass,inertia)
L.a=0;
L.d=0;
L.alpha=0;
L.theta=0;
L.com=0;
L.mass=0;
L.inertia=0;

L.a=a;
L.d=d;
L.alpha=alpha;
L.theta=[];
L.com=centOfMass;
L.mass=mass;
L.inertia=inertia;
% if isempty(L.d)
%     L.isRotary=0;
%     L.d=[];
% else
    L.isRotary=1;
%     L.theta=[];
% end