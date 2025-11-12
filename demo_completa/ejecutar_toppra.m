function [q_seg, qd_seg, qdd_seg] = ejecutar_toppra(R, archivo_mat, plot_results)% Ejecuta una trayectoria optimizada desde un archivo .mat
%
% ENTRADAS:
%   R             - Handle del robot (ej: R1)
%   archivo_mat   - Ruta completa al archivo .mat (ej: 'toppra_trajectories/R1_toppratraj_0-1.mat')
%   plot_results  - (Opcional) true/false. Si es true, genera los gráficos.
%% DEBUG
    global animar;
    global demo_tareas;
%%

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


if(animar)
    % --- Parámetros de rendimiento ---
    % dt         = mean(diff(ts)); % (No se usa, pero se podría)
    decim_draw = 2;    % dibujar cada 2 muestras (ajusta si es necesario)
    tcp_stride = 4;    % actualizar visual del TCP cada 4 muestras

    % --- Animación cronometrada ---
    t0 = tic;
    i  = 1; 
    N  = size(q_traj,1);

    while i <= N
        % Sincroniza con el tiempo físico de la trayectoria
        t_esp  = ts(i);
        t_real = toc(t0);
        if t_real < t_esp
            % Esta pausa AHORA solo ocurre si animar == true
            pause(t_esp - t_real);
        end

        % Actualiza robot
        qi = q_traj(i,:);
        R.animate(qi); % Ya no se necesita 'if(animar)' aquí dentro
        
        if mod(i, tcp_stride) == 1
            T = R.fkine(qi);
            if isa(T,'SE3'), T = T.T; end
            set(h_frame, 'Matrix', T);
            drawnow limitrate nocallbacks;
        end

        i = i + decim_draw;
    end
    
    % Asegura que el robot termine en la última postura
    R.animate(q_traj(end,:));
    T = R.fkine(q_traj(end,:));
    if isa(T,'SE3'), T = T.T; end
    set(h_frame, 'Matrix', T);
    drawnow;
    
    % fprintf('Animación completada. Duración real: %.3f s (ts(end)=%.3f s)\n', toc(t0), ts(end));

else
    % --- animar == false ---
    % No animamos, solo saltamos el robot al final de ESTE segmento
    % para que el siguiente segmento (si lo hay) empiece bien.
    fprintf('Animación omitida (animar=false). Saltando al final del segmento.\n');
    q_final = q_traj(end,:);
    R.animate(q_final);
    T = R.fkine(q_final);
    if isa(T,'SE3'), T = T.T; end
    
    set(h_frame, 'Matrix', T);
    drawnow;
end


% --- Gráficos ---
if plot_results
    graficar_perfiles(R.name, q_traj, qd_traj, qdd_traj, []);
end
q_seg = q_traj;
qd_seg = qd_traj;
qdd_seg = qdd_traj;
end