% AA222 Final Project
clear, close all

T0 = 0;
Tf = 15;
dt = 0.1;
th = 1.5; % predicting horizon
tstep = T0:dt:Tf;
N = length(tstep);


% load desired speed of the reference vehicle
Xr = ref_state(T0);

% Define state: X = [x vx ax y vy ay]';
X0 = [-150;   20; 0; -3.6; 0; 0];
Xf = [0; 20; 0; 0; 0; 0];

% [A,B] = getAB(X0, t, Xf, 15);

% Define cost function J at each step t
t = 0;

J = @(Xf)get_cost(t, th, X0, Xf, Xr, dt);
nonlcon = @(Xf)accelcons(t, th, X0, Xf, dt);
Aineq = [];
bineq = [];
Aeq = [];
beq = [];
lb = [X0(1); 10; 0; X0(4) ;  0; 0];
ub = [50;  38;  0;  0; 0; 0];

[Xf, Jf, exitflag, output] = fmincon(J, X0, Aineq, bineq, Aeq, beq, lb, ub, nonlcon)
% J1 = get_cost(t, th, X0, Xf, Xr, dt);

[A, B] = getAB(X0, t, Xf, t+th);
figure(1);
hold on
for h = t:dt:th
   
    x= get_coeff(h)*A;
    y= get_coeff(h)*B;

    plot(x(4),y(4),'*')
    plot(xr(ceil(h/dt+1)), 0, 'o')
end