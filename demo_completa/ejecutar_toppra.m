function ejecutar_toppra(R, archivo_mat)
if nargin < 2
    archivo_mat = 'trayectoria_optimizada.mat';
end

% --- Carga ---
datos   = load(archivo_mat);
ts      = datos.ts_sample;
q_traj  = datos.q_toppra;
qd_traj = datos.qd_toppra;
qdd_traj= datos.qdd_toppra;
fprintf('Cargados %d puntos. Duración total: %.3f s.\n', numel(ts), ts(end));

% --- Usa la figura actual (NO se crea otra) ---
fig = gcf;

% --- (Opcional) quita cualquier delay interno del RTB ---
if isprop(R,'delay'), R.delay = 0; end

% --- Preparar/recuperar handle del TCP (sin crear/borrar por iteración) ---
h_frame = getappdata(fig,'h_tcp_frame');
if isempty(h_frame) || ~isgraphics(h_frame)
    T0 = R.fkine(q_traj(1,:));
    if isa(T0,'SE3'), T0 = T0.T; end
    hold on;
    % Nota: sin 'arrow' ni 'thick' para aligerar
    h_frame = trplot(T0, 'frame','', 'color','m', 'length',0.2);
    hold off;
    setappdata(fig,'h_tcp_frame', h_frame);
end

% --- Parámetros de rendimiento ---
dt         = mean(diff(ts));
traj_hz    = 1/dt;
decim_draw = 2;    % dibujar cada 2 muestras (ajusta a 3-4 si tu PC va justa)
tcp_stride = 4;    % actualizar visual del TCP cada 4 muestras
fprintf('dt=%.4f s (%.1f Hz) | decim_draw=%d | tcp_stride=%d\n', dt, traj_hz, decim_draw, tcp_stride);

% --- Animación cronometrada en la MISMA figura/robot ---
t0 = tic;
i  = 1; 
N  = size(q_traj,1);

while i <= N
    % Sincroniza con el tiempo físico de la trayectoria
    t_esp  = ts(i);
    t_real = toc(t0);
    if t_real < t_esp
        pause(t_esp - t_real);
    end

    % Actualiza robot (NO reconfigura escena)
    qi = q_traj(i,:);
    R.animate(qi);

    % Actualiza el TCP solo cada tcp_stride muestras (rápido)
    if mod(i, tcp_stride) == 1
        T = R.fkine(qi);
        if isa(T,'SE3'), T = T.T; end
        % *** CLAVE: mover el mismo objeto, sin trplot de nuevo ***
        set(h_frame, 'Matrix', T);
    end

    % Throttle del render y sin callbacks
    drawnow limitrate nocallbacks

    % Decimación de frames de dibujo (el tiempo físico sigue por ts)
    i = i + decim_draw;
end

fprintf('Animación completada. Duración real: %.3f s (ts(end)=%.3f s)\n', toc(t0), ts(end));

% --- (Opcional) tus plots siguen igual ---
robot_name = R.name;
labels = arrayfun(@(k) sprintf('q%d',k), 1:size(q_traj,2), 'UniformOutput', false);

fig1 = figure(); set(fig1,'Name',sprintf('%s - Posiciones (Toppra)',robot_name));
plot(ts, q_traj); grid on; legend(labels{:}, 'Location','best');
title(sprintf('%s - q1..q6 (Toppra)', robot_name)); xlabel('Tiempo (s)'); ylabel('rad');

fig2 = figure(); set(fig2,'Name',sprintf('%s - Velocidades (Toppra)',robot_name));
plot(ts, qd_traj); grid on; legend(labels{:}, 'Location','best');
title(sprintf('%s - dq1..dq6 (Toppra)', robot_name)); xlabel('Tiempo (s)'); ylabel('rad/s');

fig3 = figure(); set(fig3,'Name',sprintf('%s - Aceleraciones (Toppra)',robot_name));
plot(ts, qdd_traj); grid on; legend(labels{:}, 'Location','best');
title(sprintf('%s - ddq1..ddq6 (Toppra)', robot_name)); xlabel('Tiempo (s)'); ylabel('rad/s^2');
end
