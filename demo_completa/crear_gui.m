function crear_gui(R, plist, Tlist, qseq, n, N, groups)
% --- (El inicio de la función es igual) ---
if nargin < 5, n = 30; end
if nargin < 6, N = 20; end
if nargin < 7, groups = []; end
fig = gcf;
% ====================================================================
% Layout derecho simple robot 1
% ====================================================================
x = 0.85; w = 0.12; h = 0.06; dy = 0.08; y0 = 0.80;
% Botón "Todo" R1 (Raw)
uicontrol('Parent',fig,'Style','pushbutton','Units','normalized', ...
    'Position',[x y0 w h], 'String','Ejecutar TODO R1', ...
    'FontSize',11, ...
    'Callback', @(~,~) ejecutar_todo_r1(R(1), plist{1}, Tlist{1}, qseq{1}, n, N));

% Botón "Ambos (sync)"
y_sync = y0 - dy/2; % un poco debajo
uicontrol('Parent',fig,'Style','pushbutton','Units','normalized', ...
    'Position',[0.45 y_sync 0.18 h], 'String','Ejecutar Ambos (sync)', ...
    'FontSize',11, 'FontWeight', 'bold', 'ForegroundColor', [0 0.5 0], ...
    'Callback', @(~,~) ejecutar_ambos_sync(R));

% --- 'y' se inicializa para el botón Optimizado ---
y = y0 - dy; % 'y' empieza en 0.72
try
    % Asumimos que el arte latte se hace desde la última pose de R1
    pose_final_R1 = plist{1}{end}.pose;
    centro_xyz = pose_final_R1(1:3);
    rpy_orient = pose_final_R1(4:6);
catch
    % Fallback por si acaso
    warning('No se pudo leer la pose final de plist{1}. Usando valores por defecto para Arte Latte.');
    centro_xyz = [0.5 0 0.55];
    rpy_orient = [0 pi/2 0];
end
uicontrol('Parent',fig,'Style','pushbutton','Units','normalized', ...
    'Position',[x y w h], 'String','Arte latte (corazón)', ...
    'FontSize',11, 'FontWeight', 'bold', 'ForegroundColor', [0.8 0 0.4], ...
    'Callback', @(~,~) callback_arte_latte(R(1), centro_xyz, rpy_orient));

y = y - dy; % Mover 'y' hacia abajo DE NUEVO

% --- Botón: Ejecutar Optimizado R1 ---
uicontrol('Parent',fig,'Style','pushbutton','Units','normalized', ...
    'Position',[x y w h], 'String','Ejecutar Optimizado R1', ...
    'FontSize',11, 'FontWeight', 'bold', 'ForegroundColor', [0 0 0.8], ... % Color azul
    'Callback', @(~,~) ejecutar_secuencia_toppra(R(1)));

% Botones agrupados o por segmento (raw)
y = y - dy; % Mover 'y' (ahora y = 0.64)

% ===================================================================
% --- INICIO DE MODIFICACIÓN LÓGICA (R1) ---
% ===================================================================
% Revisar si 'groups{1}' existe y tiene contenido
use_groups_r1 = ~isempty(groups) && numel(groups) >= 1 && ~isempty(groups{1});

if use_groups_r1
    % groups{1}: celdas con índices de plist{1}
    for g = 1:numel(groups{1})
        idxs = groups{1}{g};
        if isempty(idxs), continue; end
        i0 = idxs(1); i1 = idxs(end);
        etiqueta = sprintf('R1 %d -> %d', i0, i1);
        uicontrol('Parent',fig,'Style','pushbutton','Units','normalized', ...
            'Position',[x y w h], 'String',etiqueta, 'FontSize',10, ...
            'Callback', @(~,~) ejecutar_grupo(R(1), plist{1}, Tlist{1}, qseq{1}, idxs));
        y = y - dy;
    end
else
    % Si no se usan grupos para R1, usar plist{1}
    for k = 2:numel(plist{1})
    txt = sprintf('Punto %d → %d', k-1, k);
    uicontrol('Parent',fig,'Style','pushbutton','Units','normalized', ...
        'Position',[x y w h], 'String',txt, 'FontSize',10, ...
        'Callback', @(~,~) ejecutar_tramo(k,R(1),plist{1},Tlist{1},qseq{1}));
    y = y - dy;
    end
end
% ===================================================================
% --- FIN DE MODIFICACIÓN LÓGICA (R1) ---
% ===================================================================


% ====================================================================
% Layout izquierdo simple robot 2
% ====================================================================
x = 0.03; w = 0.12; h = 0.06; dy = 0.08; y0 = 0.80;
% Botón "Todo" R2 (Raw)
uicontrol('Parent',fig,'Style','pushbutton','Units','normalized', ...
    'Position',[x y0 w h], 'String','Ejecutar TODO R2', ...
    'FontSize',11, ...
    'Callback', @(~,~) ejecutar_trayectorias(R(2), plist{2}, Tlist{2}, qseq{2}, n, N));

% --- Botón: Ejecutar Optimizado R2 ---
y = y0 - dy; % Mover 'y' hacia abajo
uicontrol('Parent',fig,'Style','pushbutton','Units','normalized', ...
    'Position',[x y w h], 'String','Ejecutar Optimizado R2', ...
    'FontSize',11, 'FontWeight', 'bold', 'ForegroundColor', [0 0 0.8], ...
    'Callback', @(~,~) ejecutar_secuencia_toppra(R(2)));

% Botones agrupados o por segmento para R2 (raw)
y = y - dy; % Mover 'y' hacia abajo DE NUEVO

% ===================================================================
% --- INICIO DE MODIFICACIÓN LÓGICA (R2) ---
% ===================================================================
% Revisar si 'groups{2}' existe y tiene contenido
use_groups_r2 = ~isempty(groups) && numel(groups) >= 2 && ~isempty(groups{2});

if use_groups_r2
    for g = 1:numel(groups{2})
        idxs = groups{2}{g};
        if isempty(idxs), continue; end
        i0 = idxs(1); i1 = idxs(end);
        etiqueta = sprintf('R2 %d -> %d', i0, i1);
        uicontrol('Position',[x y w h],'String',etiqueta, ...
            'Callback', @(~,~) ejecutar_grupo(R(2), plist{2}, Tlist{2}, qseq{2}, idxs),'Units','normalized');
        y = y - dy;
    end
else
    % Si no se usan grupos para R2, usar plist{2}
    for k = 2:numel(plist{2})
        txt = sprintf('Punto R2 %d → %d', k-1, k);
        uicontrol('Position',[x y w h],'String',txt, ...
            'Callback', @(~,~) ejecutar_tramo(k,R(2),plist{2},Tlist{2},qseq{2}),'Units','normalized');
        y = y - dy;
    end
end
% ===================================================================
% --- FIN DE MODIFICACIÓN LÓGICA (R2) ---
% ===================================================================

%% ====================================================================
%% =================== FUNCIONES HELPER INTERNAS ======================
%% ====================================================================
    function callback_arte_latte(robot, centro_xyz, rpy_orient)
        fprintf('--- Ejecutando Arte Latte (Corazón) ---\n');
        cerrar_figuras_anteriores(); % Cerrar plots antiguos

        % 1. Ejecutar la función (con animación) y obtener datos
        % Asegurarse que 'arte_latte_corazon.m' esté en el path
        try
            % Llamamos con animar = true
            [Q, t] = arte_latte_corazon(robot, centro_xyz, rpy_orient);
        catch ME
            if strcmp(ME.identifier, 'MATLAB:UndefinedFunction')
                msgbox('Error: No se encuentra "arte_latte_corazon.m". Asegúrate de que esté en el path de MATLAB.', 'Error de Función', 'error');
            else
                rethrow(ME);
            end
            return;
        end

        if isempty(Q) || numel(t) < 2
            warning('Arte latte no devolvió datos suficientes para graficar.');
            return;
        end

        % 2. Calcular dt y Velocidades
        dt = t(2) - t(1); % Asumir dt constante
        % Calcular velocidad por diferencia finita
        Qd = [zeros(1, robot.n); diff(Q)] / dt;

        % 3. Graficar (usando el helper 'plot_q_evol' que ya existe)
        plot_q_evol(Q, dt, sprintf('%s - Posiciones (Arte Latte)', robot.name));
        plot_q_evol(Qd, dt, sprintf('%s - Velocidades (Arte Latte)', robot.name));

        fprintf('--- Arte Latte completado y graficado ---\n');
        figure(fig); % Traer la GUI al frente
    end
    function cerrar_figuras_anteriores()
        % (Función sin cambios)
        figs = findall(0, 'Type', 'figure');
        if isempty(figs), return; end
        names_to_close = {'Posiciones', 'Velocidades'};
        for i = 1:length(figs)
            fig_name = get(figs(i), 'Name'); 
            if ~isempty(fig_name)
                if any(contains(fig_name, names_to_close))
                    if figs(i) ~= fig
                        close(figs(i));
                    end
                end
            end
        end
    end
% --------------------------------------------------------------------
    function ejecutar_tramo(k,R,plist,Tlist,qseq)
        % (Función sin cambios)
        cerrar_figuras_anteriores()
        plist_sub = plist(k-1:k);
        Tlist_sub = Tlist(k-1:k);
        qseq_sub  = qseq(:,k-1:k);
        ejecutar_trayectorias(R, plist_sub, Tlist_sub, qseq_sub, n, N);
    end
% --------------------------------------------------------------------
    function ejecutar_todo_r1(R, plist, Tlist, qseq, n, N)
        % --- MODIFICADA (Llama a tu plan_arte_latte_q) ---
        
        % 1. Ejecuta y guarda todas las trayectorias RAW (como antes)
        ejecutar_trayectorias(R, plist, Tlist, qseq, n, N);
        
        % 2. Ahora, GENERA Y GUARDA el archivo RAW para el arte latte
        try
            fprintf('--- Generando archivo RAW para Arte Latte ---\n');
            
            % Obtener la última pose de la secuencia
            q_final_secuencia = qseq(:,end);
            T_final_mundo = R.fkine(q_final_secuencia');
            
            % Convertir a pose relativa a la base del robot
            T_base_inv = SE3(R.base.double).inv();
            T_final_base = T_base_inv.double * T_final_mundo.double;
            centro_base = T_final_base(1:3,4)';
            rpy_base = tr2rpy(T_final_base, 'zyx');

            % --- ¡¡AQUÍ ESTÁ EL CAMBIO!! ---
            % Llama a tu función externa plan_arte_latte_q.m
            % (Asegúrate de que esté en el path de MATLAB)
            [Q_arte, ~] = plan_arte_latte_q(R, centro_base, rpy_base);
            
            % Preparar para guardar
            waypoints_q = Q_arte;
            path_pos_s = linspace(0, 1, size(waypoints_q, 1))';
            
            % Determinar directorio de salida
            [script_dir, ~, ~] = fileparts(mfilename('fullpath'));
            output_dir = fullfile(script_dir, 'raw_trajectories');
            
            % ========================================================
            % --- INICIO DE MODIFICACIÓN (Req. 2) ---
            % Cambiado de 9-10 a 10-11
            start_idx = 10; 
            end_idx = 11;
            % --- FIN DE MODIFICACIÓN ---
            % ========================================================
            
            nombre_archivo_base = sprintf('%s_rawtraj_%d-%d.mat', R.name, start_idx, end_idx);
            nombre_archivo_completo = fullfile(output_dir, nombre_archivo_base);
            
            save(nombre_archivo_completo, 'waypoints_q', 'path_pos_s');
            fprintf('--- Arte Latte guardado en: %s ---\n', nombre_archivo_base);

        catch ME
            % Si falla aquí, es probable que 'plan_arte_latte_q.m' no esté en el path
            if strcmp(ME.identifier, 'MATLAB:UndefinedFunction')
                msgbox('Error: No se encuentra "plan_arte_latte_q.m". Asegúrate de que esté en el path de MATLAB.', 'Error de Función', 'error');
            else
                warning('Fallo al generar/guardar el archivo raw de arte latte: %s', ME.message);
            end
        end
    end
% --------------------------------------------------------------------
function ejecutar_ambos_sync(R_all)
        % ===============================================================
        % --- ¡¡FUNCIÓN MODIFICADA (ORQUESTADOR)!! ---
        % ===============================================================
        
        fprintf('\n--- Iniciando ejecución Sincrónica OPTIMIZADA (Toppra) ---\n');
        R1 = R_all(1);
        R2 = R_all(2);
        
        % --- 1. Definición de la Secuencia (Sin cambios) ---
        plan_R1 = {
           % { 'segmento', [R1.name, '_toppratraj_0-1.mat'] },
            { 'segmento', [R1.name, '_toppratraj_1-2.mat'] },
            { 'segmento', [R1.name, '_toppratraj_2-3.mat'] },
            { 'segmento', [R1.name, '_toppratraj_3-4.mat'] },
            { 'delay', 10.0 }, % espera a que se espume la leche
            { 'segmento', [R1.name, '_toppratraj_4-5.mat'] },
            { 'segmento', [R1.name, '_toppratraj_5-6.mat'] },
            { 'segmento', [R1.name, '_toppratraj_6-7.mat'] },
            { 'segmento', [R1.name, '_toppratraj_7-8.mat'] },
            { 'segmento', [R1.name, '_toppratraj_8-9.mat'] },
            { 'delay', 5.0 }, % <--- R1 espera a que R2 entregue la taza
            { 'segmento', [R1.name, '_toppratraj_9-10.mat'] }, % <-- Movimiento a la taza
            { 'segmento', [R1.name, '_rawtraj_10-11.mat'] } % <-- Arte Latte
        };
        plan_R2 = {
            %{ 'segmento', [R2.name, '_toppratraj_0-1.mat'] },
            { 'segmento', [R2.name, '_toppratraj_1-2.mat'] },
            { 'segmento', [R2.name, '_toppratraj_2-3.mat'] },
            { 'segmento', [R2.name, '_toppratraj_3-4.mat'] },
            { 'segmento', [R2.name, '_toppratraj_4-5.mat'] },
            { 'segmento', [R2.name, '_toppratraj_5-6.mat'] },
            { 'delay', 1.0 }, % <--- Delay de ejemplo R2
            { 'segmento', [R2.name, '_toppratraj_6-7.mat'] },
            { 'segmento', [R2.name, '_toppratraj_7-8.mat'] },
            { 'segmento', [R2.name, '_toppratraj_8-9.mat'] },
            { 'segmento', [R2.name, '_toppratraj_9-10.mat'] },
            { 'segmento', [R2.name, '_toppratraj_10-11.mat'] },
            { 'delay', 10.0 }, % espera a que se haga el cafe luego de apretar el boton
            { 'segmento', [R2.name, '_toppratraj_11-12.mat'] },
            { 'segmento', [R2.name, '_toppratraj_12-13.mat'] },
            { 'segmento', [R2.name, '_toppratraj_13-14.mat'] }
        };
        
        % --- 2. Cargar TODAS las trayectorias (Sin cambios) ---
        try
            [trajectories_R1, q_inicial_R1] = load_trajectory_data(R1, plan_R1);
            [trajectories_R2, q_inicial_R2] = load_trajectory_data(R2, plan_R2);
        catch ME
            msgbox(sprintf('Error al cargar archivos Toppra:\n%s', ME.message), 'Error de Carga', 'error');
            return;
        end
        
        % --- 3. Inicializar Estados (Sin cambios) ---
        state_R1 = init_robot_state(q_inicial_R1);
        state_R2 = init_robot_state(q_inicial_R2);
        
        % Mover robots a la posición inicial
        R1.animate(q_inicial_R1);
        R2.animate(q_inicial_R2);
        pause(1.0); % Pausa para ver
        
        % --- 4. Bucle de Simulación Asíncrono ---
        dt = 0.05; % Paso de simulación (100 Hz)
        t_sim = 0;
        t_start = tic;
        
        % ========================================================
        % --- INICIO DE MODIFICACIÓN (Rendimiento) ---
        
        % Un 'anim_decim' de 5 da 20 FPS (100 Hz / 5). Es un buen balance.
        anim_decim = 3; 
        anim_count = 0;
        
        % Umbral mínimo para pausas (evita 'pause(0.0001)')
        min_pause_duration = 0.002; % 2ms
        
        % --- FIN DE MODIFICACIÓN ---
        % ========================================================

        while ~state_R1.finished || ~state_R2.finished
            
            t_real = toc(t_start);
            
            % ========================================================
            % --- INICIO DE MODIFICACIÓN (Lógica de Pausa) ---
            
            t_wait = t_sim - t_real; % Tiempo que falta para el prox. paso

            if t_wait > 0
                
                if t_wait > min_pause_duration
                    pause(t_wait);
                end

                continue; 
            end
            
            % Si llegamos aquí, es porque t_real >= t_sim
            % (Vamos a tiempo o ATRASADOS, no hay tiempo para pausar)
            
            % --- FIN DE MODIFICACIÓN ---
            % ========================================================
            
            % Avanzar el tiempo de simulación
            t_sim = t_sim + dt;
            anim_count = anim_count + 1;
            
            % Actualizar el estado de cada robot de forma independiente
            [state_R1, q_R1] = update_robot_state(state_R1, plan_R1, trajectories_R1, dt);
            [state_R2, q_R2] = update_robot_state(state_R2, plan_R2, trajectories_R2, dt);
            
            % ========================================================
            % --- INICIO DE MODIFICACIÓN (Lógica de Dibujo) ---
            if mod(anim_count, anim_decim) == 0
                R1.animate(q_R1);
                R2.animate(q_R2);

                drawnow; 
            end
            % --- FIN DE MODIFICACIÓN ---
            % ========================================================
        end
        
        % Asegurar que los robots terminen en la última postura (gráficamente)
        R1.animate(state_R1.last_q);
        R2.animate(state_R2.last_q);
        drawnow;
        
        fprintf('--- ¡Simulación Sincrónica Completada! ---\n');
    end
% --------------------------------------------------------------------
    function ejecutar_grupo(R, plist, Tlist, qseq, idxs)
        % (Función sin cambios)
        cerrar_figuras_anteriores()
        plist_sub = plist(idxs);
        Tlist_sub = Tlist(idxs);
        qseq_sub  = qseq(:, idxs);
        ejecutar_trayectorias(R, plist_sub, Tlist_sub, qseq_sub, n, N);
        drawnow;      
        pause(0.05);  
        figure(fig);
    end
% --------------------------------------------------------------------
    function plot_q_evol(Q, dt, figTitle)
        % (Función sin cambios)
        if isempty(Q), return; end
        t = (0:size(Q,1)-1)' * dt;
        figure; set(gcf,'Name',figTitle);
        for i=1:6
            subplot(3,2,i);
            plot(t, Q(:,i), 'LineWidth',1.2);
            grid on; xlabel('t [s]'); ylabel(sprintf('q%d [rad]',i));
        end
        sgtitle(figTitle);
    end
% --------------------------------------------------------------------
    function ejecutar_secuencia_toppra(robot_a_ejecutar)
        % (Función sin cambios)
        fprintf('\n--- Iniciando ejecución de secuencia optimizada (TOPPRA) para %s ---\n', robot_a_ejecutar.name);
        prefijo_robot = robot_a_ejecutar.name;
        try
            [script_dir, ~, ~] = fileparts(mfilename('fullpath'));
            directorio_base = script_dir;
        catch
            directorio_base = pwd;
        end
        directorio_toppra = fullfile(directorio_base, 'toppra_trajectories');
        patron_busqueda = fullfile(directorio_toppra, [prefijo_robot, '_toppratraj_*.mat']);
        lista_archivos = dir(patron_busqueda);
        if isempty(lista_archivos)
            msgbox(sprintf('No se encontraron archivos optimizados en:\n%s\n\nCon el prefijo: "%s_toppratraj_..."', directorio_toppra, prefijo_robot), ...
                   'Error - Faltan archivos Toppra', 'error');
            return;
        end
        indices_inicio = zeros(numel(lista_archivos), 1);
        for i = 1:numel(lista_archivos)
            try
                nums = sscanf(lista_archivos(i).name, [prefijo_robot, '_toppratraj_%d-%d.mat']);
                indices_inicio(i) = nums(1);
            catch
                indices_inicio(i) = inf;
            end
        end
        [~, orden_correcto] = sort(indices_inicio);
        lista_archivos_ordenada = lista_archivos(orden_correcto);
        try
            archivo_primero = fullfile(lista_archivos_ordenada(1).folder, lista_archivos_ordenada(1).name);
            datos_primero = load(archivo_primero);
            q_inicial = datos_primero.q_toppra(1, :);
            fprintf('Posición inicial cargada. Saltando robot a q_inicial.\n');
            robot_a_ejecutar.animate(q_inicial);
            pause(0.5);
        catch e
            msgbox(sprintf('Error al cargar la posición inicial desde %s:\n%s', archivo_primero, e.message), 'Error de carga', 'error');
            return;
        end
        Q_total = []; Qd_total = []; Qdd_total = []; T_total = [];
        t_final_anterior = 0;
        for i = 1:numel(lista_archivos_ordenada)
            archivo = lista_archivos_ordenada(i);
            ruta_completa = fullfile(archivo.folder, archivo.name);
            try
                datos = load(ruta_completa);
                ts = datos.ts_sample;
                q_traj = datos.q_toppra;
                qd_traj = datos.qd_toppra;
                qdd_traj = datos.qdd_toppra;
            catch e
                warning('No se pudo cargar %s para gráficos. Saltando. Error: %s', archivo.name, e.message);
                continue;
            end
            if i > 1 && ~isempty(ts)
                idx_start = 2;
                ts_segmento = ts(idx_start:end) + t_final_anterior;
                Q_total     = [Q_total;   q_traj(idx_start:end, :)];
                Qd_total    = [Qd_total;  qd_traj(idx_start:end, :)];
                Qdd_total   = [Qdd_total; qdd_traj(idx_start:end, :)];
                T_total     = [T_total;   ts_segmento(:)];
            elseif i == 1 && ~isempty(ts)
                ts_segmento = ts;
                Q_total     = q_traj;
                Qd_total    = qd_traj;
                Qdd_total   = qdd_traj;
                T_total     = ts_segmento(:);
            end
            if ~isempty(ts_segmento)
                t_final_anterior = ts_segmento(end);
            end
            fprintf('--- [%d/%d] Ejecutando: %s ---\n', i, numel(lista_archivos_ordenada), archivo.name);
            if ~exist('ejecutar_toppra', 'file')
                msgbox('Error: La función "ejecutar_toppra.m" no se encuentra en el path de MATLAB.', 'Error de función', 'error');
                return;
            end
            ejecutar_toppra(robot_a_ejecutar, ruta_completa, false);
            pause(0.1);
        end
        fprintf('\n--- ¡Secuencia optimizada completada para %s! ---\n', prefijo_robot);
        disp('Mostrando gráficos de la secuencia completa...');
        if ~isempty(T_total)
            robot_name = robot_a_ejecutar.name;
            labels = arrayfun(@(k) sprintf('q%d',k), 1:size(Q_total,2), 'UniformOutput', false);
            fig1 = figure(); set(fig1,'Name',sprintf('%s - Posiciones (Toppra Completa)',robot_name));
            plot(T_total, Q_total); grid on; legend(labels{:}, 'Location','best');
            title(sprintf('%s - q1..q6 (Toppra Completa)', robot_name)); xlabel('Tiempo (s)'); ylabel('rad');
            fig2 = figure(); set(fig2,'Name',sprintf('%s - Velocidades (Toppra Completa)',robot_name));
            plot(T_total, Qd_total); grid on; legend(labels{:}, 'Location','best');
            title(sprintf('%s - dq1..dq6 (Toppra Completa)', robot_name)); xlabel('Tiempo (s)'); ylabel('rad/s');
            fig3 = figure(); set(fig3,'Name',sprintf('%s - Aceleraciones (Toppra Completa)',robot_name));
            plot(T_total, Qdd_total); grid on; legend(labels{:}, 'Location','best');
            title(sprintf('%s - ddq1..ddq6 (Toppra Completa)', robot_name)); xlabel('Tiempo (s)'); ylabel('rad/s^2');
        else
            disp('No se acumularon datos para graficar.');
        end
    end

% --- NUEVOS HELPERS PARA EL ORQUESTADOR ---
    function [map, q0] = load_trajectory_data(robot, plan)
        % (Función sin cambios)
        % Carga todos los .mat necesarios para un plan en un mapa
        map = containers.Map('KeyType','char','ValueType','any');
        q0 = [];
        
        try
            [script_dir, ~, ~] = fileparts(mfilename('fullpath'));
        catch
            script_dir = pwd;
        end
        directorio_toppra = fullfile(script_dir, 'toppra_trajectories');

        for i = 1:numel(plan)
            step = plan{i};
            if strcmp(step{1}, 'segmento')
                filename = step{2};
                if ~isKey(map, filename) % No cargar duplicados
                    filepath = fullfile(directorio_toppra, filename);
                    if ~exist(filepath, 'file')
                        error('Archivo Toppra FALTANTE: %s', filepath);
                    end
                    datos = load(filepath);
                    map(filename) = datos;
                    
                    % Capturar la q inicial del PRIMER segmento
                    if isempty(q0)
                        q0 = datos.q_toppra(1,:);
                    end
                end
            end
        end
        if isempty(q0)
           q0 = zeros(1, robot.n); % Fallback
           warning('No se cargó ningún segmento para %s. Usando q0 = [0...0]', robot.name);
        end
    end

    function [state] = init_robot_state(q_inicial)
        % (Función sin cambios)
        % Crea la estructura de estado inicial para un robot
        state = struct(...
            'plan_index', 1, ...         % Siguiente paso a ejecutar
            'segment_data', [], ...      % Datos del .mat actual
            'segment_time', 0, ...       % Tiempo transcurrido en el segmento actual
            'segment_duration', 0, ...   % Duración total del segmento actual
            'wait_remaining', 0, ...     % Segundos restantes de un delay
            'last_q', q_inicial, ...     % Última q conocida (para delays)
            'finished', false ...        % Si el plan terminó
        );
    end

    function [state, q] = update_robot_state(state, plan, trajectories, dt)
        % (Función sin cambios)
        % El "cerebro": actualiza el estado de UN robot para un paso de tiempo dt
        
        % Si el plan terminó, no hacer nada y devolver la última q
        if state.finished
            q = state.last_q;
            return;
        end
        
        % 1. Procesar Delay (si está activo)
        if state.wait_remaining > 0
            state.wait_remaining = state.wait_remaining - dt;
            q = state.last_q; % Mantener la última posición
            return;
        end
        
        % 2. Cargar Siguiente Paso (si no hay segmento activo)
        if isempty(state.segment_data)
            if state.plan_index > numel(plan)
                % El plan se completó en este ciclo
                state.finished = true;
                q = state.last_q;
                return;
            end
            
            % Obtener el siguiente paso del plan
            current_step = plan{state.plan_index};
            state.plan_index = state.plan_index + 1;
            
            if strcmp(current_step{1}, 'delay')
                state.wait_remaining = current_step{2}; % Activar delay
                q = state.last_q; % Mantener posición
                return;
            elseif strcmp(current_step{1}, 'segmento')
                segment_name = current_step{2};
                state.segment_data = trajectories(segment_name);
                state.segment_time = 0; % Iniciar tiempo del segmento
                state.segment_duration = state.segment_data.ts_sample(end);
                % (Continuar al paso 3 para ejecutar el tiempo 0)
            end
        end
        
        % 3. Procesar Segmento (si está activo)
        state.segment_time = state.segment_time + dt;
        
        if state.segment_time >= state.segment_duration
            % --- Segmento terminado ---
            q = state.segment_data.q_toppra(end, :); % Devolver la última q exacta
            state.last_q = q;
            state.segment_data = []; % Limpiar para cargar el siguiente paso
        else
            % --- Segmento en progreso ---
            % Interpolar para encontrar la q en el tiempo actual
            q = interp1(state.segment_data.ts_sample, state.segment_data.q_toppra, state.segment_time, 'linear');
            state.last_q = q; % Guardar para el próximo ciclo
        end
    end
% --- FIN DE LAS FUNCIONES HELPER ---
end