function X = get_state(t, dt, A, B)
% t = 0+dt;
% t0 < t < th
x_coeff= get_coeff(t)*A;
y_coeff= get_coeff(t)*B;

X = [x_coeff(4:6); y_coeff(4:6)];
