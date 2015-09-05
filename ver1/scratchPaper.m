clc, clear, close all

% A = [1 2 3;4 5 6;7 8 9]
% B = [2 2 2; 0 0 0; 0 0 0]
% C = [2 2 2]
% 
% A*B'
% 
% A*C'


% acc = [-15 -290 -9507]
% 
% temp = acc(3)*acc(3) + acc(2)*acc(2);
% fprintf('temp = %f \n', temp)
% x = sqrt(temp);
% fprintf('x = %f \n', x)
% y = acc(1);
% fprintf('y = %f \n', y)
% 
% theta = atan2(y,x)*180/pi()*100;
% fprintf('theta = %f  = %f degree\n', theta, theta/100)
% 
% 
 

 R = [0.7283   -0.6848    0.0263; 0.6851    0.7266   -0.0525; 0.0169    0.0563    0.9983]
[r(i) q(i) p(i)] = d2a(R')*16384;


 
 