function [y,y1] = fcn()
load points3D;
pos=zeros(76,6);
vel=zeros(76,6);
theta=zeros(21,7);
th_last=[0;-pi/2;0;0;0;0];
t=[0 2 2.5 2.75:0.25:7.5];
[th1,th2,th3,th4,th5,th6,reachable]=abbInvKine([eye(4);0 0 0 1],th_last);
theta(1,2:7)=[th1,th2,th3,th4,th5,th6];
theta(1,1)=t(1);
[th1,th2,th3,th4,th5,th6,reachable]=abbInvKine([eye(3) points3D(1,:)';0 0 0 1],theta(1,2:7)');
theta(2,2:7)=[th1,th2,th3,th4,th5,th6];
theta(2,1)=t(2);
[th1,th2,th3,th4,th5,th6,reachable]=abbInvKine([eye(3) points3D(1,:)';0 0 0 1],theta(1,2:7)');
theta(3,2:7)=[th1,th2,th3,th4,th5,th6];
theta(3,1)=t(3);
for i=3:22
[th1,th2,th3,th4,th5,th6,reachable]=abbInvKine([eye(3) points3D(i-1,:)';0 0 0 1],theta(i,2:7)');
theta(i+1,2:7)=[th1,th2,th3,th4,th5,th6];
theta(i+1,1)=t(i+1);
end
t1=0:0.1:7.5;
for j=1:length(t1)
[p,v,a]=constAccelInterp(t1(j),theta,100);
thetaa(j,:)=p;
thetaa_dot(j,:)=v;
end
y=thetaa
y1=thetaa_dot
end