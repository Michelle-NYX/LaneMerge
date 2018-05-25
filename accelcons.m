function [c, ceq] = accelcons(t, th, X, Xf, dt)

a_max = 3;

tstep = t:dt:(t+th);
N = length(tstep);

[A, B] = getAB(X, t, Xf, t+th);
ax = 2*A(3) + 6*A(4)*tstep + 12*A(5)*tstep.^2 +20*A(6)*tstep.^3;
ay = 2*B(3) + 6*B(4)*tstep + 12*B(5)*tstep.^2 +20*B(6)*tstep.^3;

ceq = [];
c = max(ax.^2+ay.^2) - a_max^2;