function Xr = ref_state(t)

dt = 0.1;
persistent vr  xr
if isempty(vr)
    load('V_ref.mat');
    vr = 15+Uxdes(1:end)';
    xr = dt*cumtrapz(vr)-150;  
end
idx = ceil(t/dt)+1;
Xr = [xr(idx), vr(idx), 0, 0, 0, 0]';