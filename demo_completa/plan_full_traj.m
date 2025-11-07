function [Q, dt, seg_bounds] = plan_full_traj(R, plist, Tlist, qseq, n, N, include_arte)
% PLAN_FULL_TRAJ Precalcula trayectoria articular completa para un robot.
% [Q, dt] = plan_full_traj(R, plist, Tlist, qseq, n, N, include_arte)
% - Devuelve Q [K x 6] con poses articulares y dt (sugerido 0.05s)

if nargin < 7, include_arte = false; end
dt = 0.05;

Q = [];
seg_bounds = struct('ends', [], 'labels', {{} });
q_curr = qseq(:,1)';

for k = 2:numel(plist)
    tipo = plist{k}.tipo;
    switch tipo
        case 'articular'
            [qt, ~, ~] = jtraj(q_curr, qseq(:,k)', n);
            Q = [Q; qt]; %#ok<AGROW>
            q_curr = qt(end,:);
            seg_bounds.ends(end+1) = size(Q,1); %#ok<AGROW>
            seg_bounds.labels{end+1} = sprintf('art %d->%d', k-1, k); %#ok<AGROW>
        case 'cartesiana'
            T_start = R.fkine(q_curr);
            T_end   = SE3(Tlist{k});
            Ts = ctraj(T_start, T_end, N);
            qt_seg = zeros(N, R.n);
            q_seed = q_curr';
            for i=1:N
                q_next = ik_barista(R, Ts(i), q_seed, true);
                qt_seg(i,:) = q_next';
                q_seed = q_next;
            end
            Q = [Q; qt_seg]; %#ok<AGROW>
            q_curr = qt_seg(end,:);
            seg_bounds.ends(end+1) = size(Q,1); %#ok<AGROW>
            seg_bounds.labels{end+1} = sprintf('cart %d->%d', k-1, k); %#ok<AGROW>
        otherwise
            % ignorar
    end
end

% Arte latte (sólo R1) si corresponde
if include_arte
    centro = [0.499 0 0.55];
    rpy    = [0.01 pi/2+0.5 0];
    Np = 50; escala = 0.003;
    t = linspace(pi, -pi, Np)';
    x_raw = escala*(13*cos(t)-5*cos(2*t)-2*cos(3*t)-cos(4*t));
    y_raw = escala*(16*sin(t).^3);
    x_rel = (x_raw - x_raw(1));
    y_rel = (y_raw - y_raw(1));
    Cart_puntos = [centro(1)+x_rel, centro(2)+y_rel, ...
                   repmat(centro(3), Np, 1), repmat(rpy, Np, 1)];
    v_max = 0.10; t_blend = 0.10;
    Cart_traj = mstraj(Cart_puntos, v_max, [], Cart_puntos(1,:), dt, t_blend);

    q_seed = q_curr';
    for i = 1:size(Cart_traj,1)
        vec = Cart_traj(i,:);
        T_base = transl(vec(1:3)) * rpy2tr(vec(4:6), 'zyx');
        T_w    = R.base.double * T_base;
        q_next = ik_barista(R, T_w, q_seed, true);
        Q = [Q; q_next']; %#ok<AGROW>
        q_seed = q_next;
    end
    seg_bounds.ends(end+1) = size(Q,1);
    seg_bounds.labels{end+1} = 'arte latte';
end

end


