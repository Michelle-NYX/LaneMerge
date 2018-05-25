function Xs = get_states(t, th, dt, A, B)
% get trajectory over time horizon th
% t = 0;
tstep = t:dt:(t+th);
n = length(tstep);
Xs = zeros(n,6);

for i = 1:n
    h = tstep(i);
    x_coeff= get_coeff(h)*A;
    y_coeff= get_coeff(h)*B;
    Xs(i,:) = [x_coeff(4:6); y_coeff(4:6)]';
end

