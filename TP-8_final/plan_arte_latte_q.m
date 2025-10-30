function [Q_arte, dt] = plan_arte_latte_q(R, centro_base_xyz, rpy_fijo)
% PLAN_ARTE_LATTE_Q Genera la trayectoria articular del corazón sin animar.

dt = 0.05; v_max = 0.10; t_blend = 0.10;

% Corazón paramétrico
Np = 50; escala = 0.003;
t = linspace(pi, -pi, Np)';
x_raw = escala*(13*cos(t)-5*cos(2*t)-2*cos(3*t)-cos(4*t));
y_raw = escala*(16*sin(t).^3);
x_rel = (x_raw - x_raw(1));
y_rel = (y_raw - y_raw(1));

Cart_puntos = [centro_base_xyz(1)+x_rel, centro_base_xyz(2)+y_rel, ...
               repmat(centro_base_xyz(3), Np, 1), repmat(rpy_fijo, Np, 1)];
Cart_traj = mstraj(Cart_puntos, v_max, [], Cart_puntos(1,:), dt, t_blend);

% IK a mundo
try
    q_seed = R.getpos()';
catch
    q_seed = zeros(6,1);
end
Q_arte = zeros(size(Cart_traj,1), 6);
for i=1:size(Cart_traj,1)
    vec = Cart_traj(i,:);
    T_base = transl(vec(1:3)) * rpy2tr(vec(4:6), 'zyx');
    T_w    = R.base.double * T_base;
    q_next = ik_barista(R, T_w, q_seed, true);
    Q_arte(i,:) = q_next';
    q_seed = q_next;
end

end


