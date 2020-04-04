% FUNCTION NAME: cpMap %Cross Product Matrix
% 
%  X = cpMap(w) returns the matrix of the cross product operator. 
%  cross(W)*V = W x V
% 
% 
% X = Cross product Matrix 
%
% W = Vector 
% 
% Deepak Rajasekhar Karishetti
% 10846936
% MEGN544 
% 09-20-2018

function X = cpMap(w)
X=[0 -w(3) w(2);w(3) 0 -w(1);-w(2) w(1) 0];
end