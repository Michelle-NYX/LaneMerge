function J = get_cost(t, th, X, Xf, Xr, dt)
% Input: t: current time
%        th = prediction horizion
%        X0 = [x vx ax y vy ay]' = initial state
%        Xf = [x vx ax y vy ay]' = final state
%        Xr = Xr(t): zero order hold for reference vehicle
%        dt = updating time

tstep = t:dt:(t+th);
N = length(tstep);

% mission requirements
ygoal = 0;

% relative movement to the reference vehicle
dx = Xf(1) - (Xr(1)+Xr(2)*dt);
dy = Xf(4) - (Xr(4)+Xr(5)*dt);
dvx = Xf(2) - Xr(2);

%% cost functions
% Jx = (xF-Xf(1))^2;
% Jy = (yF-Xf(4))^2;

% lance chanfe incentive
Jl = 500*min([1.5+Xf(1)/150,1])*abs(ygoal-Xf(4));

% collision avoidance
if abs(dx) < 8 && abs(dy) < 2;
    Jc = 1000*(9.25-sqrt(dx^2 +dy^2));
else
    Jc = 0;
end

% longitudinal disambiguation incentive
Jd = -100*min([max([dx*dvx,0]),1]);

% control effort
[A, B] = getAB(X, t, Xf, t+th);
ax = 2*A(3) + 6*A(4)*tstep + 12*A(5)*tstep.^2 +20*A(6)*tstep.^3;
ay = 2*B(3) + 6*B(4)*tstep + 12*B(5)*tstep.^2 +20*B(6)*tstep.^3;

Ja = mean(ax.^2+500*ay.^2);

J = Jl + Jd + Jc + Ja;




