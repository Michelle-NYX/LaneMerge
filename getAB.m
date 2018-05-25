function [A, B] = getAB(Xt, t, Xf, tf)

% T = [1 t t^2 t^3 t^4 t^5];
% X = [a0 a1 a2 a3 a4 a5];
% V = [0 a1 2*a2 3*a3 4*a4 5*a5];
% A = [0 0 2*a2 6*a3 12*a4 20*a5];

h = tf;
Coeff = inv([1 0 0 0 0 0;
           0 1 0 0 0 0;
           0 0 1 0 0 0;
           1 h   h^2   h^3   h^4    h^5;
           0 1 2*h   3*h^2 4*h^3  5*h^4;
           0 0 2     6*h   12*h^2 20*h^3]);
       
A = Coeff * [Xt(1:3); Xf(1:3)];%-->x
B = Coeff * [Xt(4:6); Xf(4:6)];%-->y