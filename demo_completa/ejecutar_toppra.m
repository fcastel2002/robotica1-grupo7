function ejecutar_toppra(R, archivo_mat, plot_results)
% Ejecuta una trayectoria optimizada desde un archivo .mat
%
% ENTRADAS:
%   R             - Handle del robot (ej: R1)
%   archivo_mat   - Ruta completa al archivo .mat (ej: 'toppra_trajectories/R1_toppratraj_0-1.mat')
%   plot_results  - (Opcional) true/false. Si es true, genera los gráficos.

if nargin < 3
    plot_results = false; % Por defecto, no mostrar gráficos
end

% --- Carga ---
try
    datos = load(archivo_mat);
catch e
    fprintf('Error cargando el archivo: %s\n', archivo_mat);
    disp(e.message);
    return;
end

ts      = datos.ts_sample;
q_traj  = datos.q_toppra;
qd_traj = datos.qd_toppra;
qdd_traj= datos.qdd_toppra;
fprintf('Cargados %d puntos. Duración total: %.3f s.\n', numel(ts), ts(end));

% --- Usa la figura actual (la que crea el script principal) ---
fig = gcf;
if isempty(fig)
    fig = figure; % Fallback por si se llama solo
end

% --- (Opcional) quita cualquier delay interno del RTB ---
if isprop(R,'delay'), R.delay = 0; end

% --- Preparar/recuperar handle del TCP ---
h_frame = getappdata(fig,'h_tcp_frame');
if isempty(h_frame) || ~isgraphics(h_frame)
    T0 = R.fkine(q_traj(1,:));
    if isa(T0,'SE3'), T0 = T0.T; end
    hold on;
    h_frame = trplot(T0, 'frame','', 'color','m', 'length',0.2);
    hold off;
    setappdata(fig,'h_tcp_frame', h_frame);
end

% --- Parámetros de rendimiento ---
dt         = mean(diff(ts));
traj_hz    = 1/dt;
decim_draw = 2;    % dibujar cada 2 muestras (ajusta si es necesario)
tcp_stride = 4;    % actualizar visual del TCP cada 4 muestras
% fprintf('dt=%.4f s (%.1f Hz) | decim_draw=%d | tcp_stride=%d\n', dt, traj_hz, decim_draw, tcp_stride);

% --- Animación cronometrada ---
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

    % Actualiza robot
    qi = q_traj(i,:);
    R.animate(qi);

    % Actualiza el TCP
    if mod(i, tcp_stride) == 1
        T = R.fkine(qi);
        if isa(T,'SE3'), T = T.T; end
        set(h_frame, 'Matrix', T);
    end

    drawnow limitrate nocallbacks
    i = i + decim_draw;
end
% Asegura que el robot termine en la última postura
R.animate(q_traj(end,:));
T = R.fkine(q_traj(end,:));
if isa(T,'SE3'), T = T.T; end
set(h_frame, 'Matrix', T);
drawnow;

% fprintf('Animación completada. Duración real: %.3f s (ts(end)=%.3f s)\n', toc(t0), ts(end));

% --- Gráficos (solo si se piden) ---
if plot_results
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

end