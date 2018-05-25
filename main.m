% AA222 Final Project
% MPC framework
clear, close all

figure(1)
hold on
grid on
% xlim([-150, -100])
% ylim([-5 5])

%% simulation setting
T0 = 0;
X0 = [-150; 20; 0; -3.6; 0; 0]; % state: X= [x vx ax y vy ay]';
Tf = 5;
Xf = [0; 20; 0; 0; 0; 0];

dt = 0.1;
th = 1; % predicting horizon
tstep = T0:dt:Tf;
N = length(tstep);
n = length(0:dt:th);

%% Reserve spaces for ego vehicle and reference vehicle states
states_Xe = zeros(N, 6);
states_Xe(1,:) = X0';
states_Xr = zeros(N, 6);

%% MPC framework
X = X0;
exitflag = false;
for idx = 1:N-n-1
    t = tstep(idx);
    % observe and store reference vehicle state at t
    Xr = ref_state(t);
    states_Xr(idx,:) = Xr';
    plot(Xr(1), Xr(4),'s','Color',[1 1 1]*(0.9-t/Tf), 'MarkerSize',14)
    
    % optimization for final states up to time horizon
    J = @(Xf)get_cost(t, th, X, Xf, Xr, dt);
    nonlcon = @(Xf)accelcons(t, th, X0, Xf, dt);
    Aineq = [];
    bineq = [];
    Aeq = [];
    beq = [];
    lb = [X(1)+1.5*10; 17; 0; X(4) ;  0; 0];
    ub = [50;  38;  0;  0; 0; 0];

    [Xf, Jf] = fmincon(J, X, Aineq, bineq, Aeq, beq, lb, ub, nonlcon);
    [At, Bt] = getAB(X, t, Xf, t+th);
    if abs(Xf(4)-0) < 0.01
        exitflag = true;
    end
    
    Xs = get_states(t,th, dt, At, Bt);
    plot(Xs(:,1), Xs(:,4)) % plot trail
    
    %update to new state
%     X = get_state(t, dt, A, B);
    X = Xs(1,:)';
    states_Xe(idx,:) = X';
    
    % plot the existing states
    if exitflag
        states_Xe(idx+1: idx+n,:) = Xs;
%         plot(Xs(:,1), Xs(:,4),'ks','MarkerSize',12)
        Xf = get_state(t, t+th, At, Bt);
%         plot(Xf(1), Xf(4),'rs', 'MarkerSize',12)
        Xr = ref_state(t+th);
%         plot(Xr(1), Xr(4),'bs', 'MarkerSize',12)
        break
    end
end
    
%% Plot the trajectory
states_Xe = states_Xe(1:idx+n,:);
states_Xr = states_Xr(1:idx,:);

for i = 1:(idx+n)
     plot(states_Xe(i,1), states_Xe(i,4),'s','Color',[1 1 1]*(1-i/(idx+n)), 'MarkerSize',14)
end