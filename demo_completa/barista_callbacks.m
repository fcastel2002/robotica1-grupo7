function handles = barista_callbacks(fig, R, plist, Tlist, qseq, n, N, groups)
% BARISTA_CALLBACKS
%   Centraliza toda la lógica de ejecución (orquestación) para la GUI del barista.
%   Recibe todos los datos de la simulación y devuelve un struct de 
%   "function handles" listos para ser asignados a los 'Callback' de uicontrol.

    % --- Asignar handles de funciones a la estructura de salida ---
    handles.ejecutar_todo_r1 = @ejecutar_todo_r1_callback;
    handles.ejecutar_todo_r2 = @ejecutar_todo_r2_callback;
    handles.ejecutar_ambos_sync = @ejecutar_ambos_sync_callback;
    handles.arte_latte = @arte_latte_callback;
    handles.optimizado_r1 = @optimizado_r1_callback;
    handles.optimizado_r2 = @optimizado_r2_callback;
    handles.grupo_r1 = @grupo_r1_callback;
    handles.grupo_r2 = @grupo_r2_callback;

    % ====================================================================
    % --- INICIO: DEFINICIONES DE FUNCIONES ANIDADAS (LÓGICA) ---
    % ====================================================================
    % Estas funciones anidadas "capturan" las variables de entrada 
    % (R, plist, Tlist, etc.) del scope de 'barista_callbacks'.
    % ====================================================================
    
    %% --- WRAPPERS DE CALLBACK PRINCIPALES ---

    function ejecutar_todo_r1_callback(~, ~)
        % (Esta es la función modificada que también guarda el 'arte latte' raw)
        ejecutar_todo_r1(R(1), plist{1}, Tlist{1}, qseq{1}, n, N);
    end

    function ejecutar_todo_r2_callback(~, ~)
        ejecutar_trayectorias(R(2), plist{2}, Tlist{2}, qseq{2}, n, N);
    end

    function ejecutar_ambos_sync_callback(~, ~)
        ejecutar_ambos_sync(R);
    end

    function arte_latte_callback(~, ~)
        try
            % Asumir que el arte latte se hace desde la última pose de R1
            pose_final_R1 = plist{1}{end}.pose;
            centro_xyz = pose_final_R1(1:3);
            rpy_orient = pose_final_R1(4:6);
        catch
            warning('No se pudo leer la pose final de plist{1}. Usando valores por defecto para Arte Latte.');
            centro_xyz = [0.5 0 0.55];
            rpy_orient = [0 pi/2 0];
        end
        callback_arte_latte(R(1), centro_xyz, rpy_orient);
    end

    function optimizado_r1_callback(~, ~)
        ejecutar_secuencia_toppra(R(1));
    end

    function optimizado_r2_callback(~, ~)
        ejecutar_secuencia_toppra(R(2));
    end

    function grupo_r1_callback(g_idx)
        % 'g_idx' es el índice (g) del grupo a ejecutar
        idxs = groups{1}{g_idx};
        ejecutar_grupo(R(1), plist{1}, Tlist{1}, qseq{1}, idxs);
    end

    function grupo_r2_callback(g_idx)
        % 'g_idx' es el índice (g) del grupo a ejecutar
        idxs = groups{2}{g_idx};
        ejecutar_grupo(R(2), plist{2}, Tlist{2}, qseq{2}, idxs);
    end

    %% --- FUNCIONES DE LÓGICA Y ORQUESTACIÓN (MOVIMIDAS DESDE crear_gui) ---

    function callback_arte_latte(robot, centro_xyz, rpy_orient)
        fprintf('--- Ejecutando Arte Latte (Corazón) ---\n');
        cerrar_figuras_anteriores(); 

        try
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

        dt = t(2) - t(1); 
        Qd = [zeros(1, robot.n); diff(Q)] / dt;

        plot_q_evol(Q, dt, sprintf('%s - Posiciones (Arte Latte)', robot.name));
        plot_q_evol(Qd, dt, sprintf('%s - Velocidades (Arte Latte)', robot.name));

        fprintf('--- Arte Latte completado y graficado ---\n');
        figure(fig); 
    end

    function cerrar_figuras_anteriores()
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



    function ejecutar_todo_r1(R_e, plist_e, Tlist_e, qseq_e, n_e, N_e)
        ejecutar_trayectorias(R_e, plist_e, Tlist_e, qseq_e, n_e, N_e);
        
        try
            fprintf('--- Generando archivo RAW para Arte Latte ---\n');
            q_final_secuencia = qseq_e(:,end);
            T_final_mundo = R_e.fkine(q_final_secuencia');
            
            T_base_inv = SE3(R_e.base.double).inv();
            T_final_base = T_base_inv.double * T_final_mundo.double;
            centro_base = T_final_base(1:3,4)';
            rpy_base = tr2rpy(T_final_base, 'zyx');

            % Asumimos que plan_arte_latte_q.m existe en el path
            [Q_arte, ~] = plan_arte_latte_q(R_e, centro_base, rpy_base);
            
            waypoints_q = Q_arte;
            path_pos_s = linspace(0, 1, size(waypoints_q, 1))';
            
            [script_dir, ~, ~] = fileparts(mfilename('fullpath'));
            output_dir = fullfile(script_dir, 'raw_trajectories');
            
            start_idx = 10; 
            end_idx = 11;
            
            nombre_archivo_base = sprintf('%s_rawtraj_%d-%d.mat', R_e.name, start_idx, end_idx);
            nombre_archivo_completo = fullfile(output_dir, nombre_archivo_base);
            
            save(nombre_archivo_completo, 'waypoints_q', 'path_pos_s');
            fprintf('--- Arte Latte guardado en: %s ---\n', nombre_archivo_base);

        catch ME
            if strcmp(ME.identifier, 'MATLAB:UndefinedFunction')
                msgbox('Error: No se encuentra "plan_arte_latte_q.m". Asegúrate de que esté en el path de MATLAB.', 'Error de Función', 'error');
            else
                warning('Fallo al generar/guardar el archivo raw de arte latte: %s', ME.message);
            end
        end
    end

    function ejecutar_ambos_sync(R_all)
        fprintf('\n--- Iniciando ejecución Sincrónica OPTIMIZADA (Toppra) ---\n');
        R1_sync = R_all(1);
        R2_sync = R_all(2);
        
        plan_R1 = {
            { 'segmento', [R1_sync.name, '_toppratraj_1-2.mat'] },
            { 'segmento', [R1_sync.name, '_toppratraj_2-3.mat'] },
            { 'segmento', [R1_sync.name, '_toppratraj_3-4.mat'] },
            { 'delay', 10.0 }, 
            { 'segmento', [R1_sync.name, '_toppratraj_4-5.mat'] },
            { 'delay', 3.0 }, 
            { 'segmento', [R1_sync.name, '_toppratraj_5-6.mat'] },
            { 'delay', 3.0 }, 
            { 'segmento', [R1_sync.name, '_toppratraj_6-7.mat'] },
            { 'segmento', [R1_sync.name, '_toppratraj_7-8.mat'] },
            { 'delay', 3.0 }, 
            { 'segmento', [R1_sync.name, '_toppratraj_8-9.mat'] },
            { 'delay', 6.0 }, 
            { 'segmento', [R1_sync.name, '_toppratraj_9-10.mat'] }, 
            { 'segmento', [R1_sync.name, '_toppratraj_10-11.mat'] } 
        };
        plan_R2 = {
            { 'segmento', [R2_sync.name, '_toppratraj_1-2.mat'] },
            { 'segmento', [R2_sync.name, '_toppratraj_2-3.mat'] },
            { 'segmento', [R2_sync.name, '_toppratraj_3-4.mat'] },
            { 'segmento', [R2_sync.name, '_toppratraj_4-5.mat'] },
            { 'segmento', [R2_sync.name, '_toppratraj_5-6.mat'] },
            { 'delay', 1.0 }, 
            { 'segmento', [R2_sync.name, '_toppratraj_6-7.mat'] },
            { 'segmento', [R2_sync.name, '_toppratraj_7-8.mat'] },
            { 'segmento', [R2_sync.name, '_toppratraj_8-9.mat'] },
            { 'segmento', [R2_sync.name, '_toppratraj_9-10.mat'] },
            { 'segmento', [R2_sync.name, '_toppratraj_10-11.mat'] },
            { 'delay', 10.0 }, 
            { 'segmento', [R2_sync.name, '_toppratraj_11-12.mat'] },
            { 'segmento', [R2_sync.name, '_toppratraj_12-13.mat'] },
            { 'segmento', [R2_sync.name, '_toppratraj_13-14.mat'] }
        };
        
        try
            [trajectories_R1, q_inicial_R1] = load_trajectory_data(R1_sync, plan_R1);
            [trajectories_R2, q_inicial_R2] = load_trajectory_data(R2_sync, plan_R2);
        catch ME
            msgbox(sprintf('Error al cargar archivos Toppra:\n%s', ME.message), 'Error de Carga', 'error');
            return;
        end
        
        state_R1 = init_robot_state(q_inicial_R1);
        state_R2 = init_robot_state(q_inicial_R2);
        
        R1_sync.animate(q_inicial_R1);
        R2_sync.animate(q_inicial_R2);
        pause(1.0); 
        
        dt_sim = 0.05; 
        t_sim = 0;
        t_start = tic;
        
        anim_decim = 3; 
        anim_count = 0;
        min_pause_duration = 0.002; 

        while ~state_R1.finished || ~state_R2.finished
            t_real = toc(t_start);
            t_wait = t_sim - t_real; 

            if t_wait > 0
                if t_wait > min_pause_duration
                    pause(t_wait);
                end
                continue; 
            end
            
            t_sim = t_sim + dt_sim;
            anim_count = anim_count + 1;
            
            [state_R1, q_R1] = update_robot_state(state_R1, plan_R1, trajectories_R1, dt_sim);
            [state_R2, q_R2] = update_robot_state(state_R2, plan_R2, trajectories_R2, dt_sim);
            
            if mod(anim_count, anim_decim) == 0
                R1_sync.animate(q_R1);
                R2_sync.animate(q_R2);
                drawnow; 
            end
        end
        
        R1_sync.animate(state_R1.last_q);
        R2_sync.animate(state_R2.last_q);
        drawnow;
        
        fprintf('--- ¡Simulación Sincrónica Completada! ---\n');
    end

    function ejecutar_grupo(R_g, plist_g, Tlist_g, qseq_g, idxs_g)
        cerrar_figuras_anteriores()
        plist_sub = plist_g(idxs_g);
        Tlist_sub = Tlist_g(idxs_g);
        qseq_sub  = qseq_g(:, idxs_g);
        
        % Llamamos a la función externa 'ejecutar_trayectorias.m'
        ejecutar_trayectorias(R_g, plist_sub, Tlist_sub, qseq_sub, n, N);
        drawnow;      
        pause(0.05);  
        figure(fig);
    end

    function plot_q_evol(Q, dt_plot, figTitle)
        if isempty(Q), return; end
        t = (0:size(Q,1)-1)' * dt_plot;
        figure; set(gcf,'Name',figTitle);
        for i=1:6
            subplot(3,2,i);
            plot(t, Q(:,i), 'LineWidth',1.2);
            grid on; xlabel('t [s]'); ylabel(sprintf('q%d [rad]',i));
        end
        sgtitle(figTitle);
    end

    function ejecutar_secuencia_toppra(robot_a_ejecutar)
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
        seg_bounds = [];
        for i = 1:numel(lista_archivos_ordenada)
            archivo = lista_archivos_ordenada(i);
            ruta_completa = fullfile(archivo.folder, archivo.name);

            fprintf('--- [%d/%d] Ejecutando: %s ---\n', i, numel(lista_archivos_ordenada), archivo.name);
            if ~exist('ejecutar_toppra', 'file')
                msgbox('Error: "ejecutar_toppra.m" no se encuentra.', 'Error', 'error');
                return;
            end

            % 1. LLAMAMOS A LA FUNCIÓN Y CAPTURAMOS LOS DATOS
            %    'false' es para plot_results (no plotear cada segmento)
            [q_seg, qd_seg, qdd_seg] = ejecutar_toppra(robot_a_ejecutar, ruta_completa, false);

            % 2. ACUMULAMOS LOS DATOS DEVUELTOS
            if ~isempty(q_seg)
                Q_total     = [Q_total;   q_seg];
                Qd_total    = [Qd_total;  qd_seg];
                Qdd_total   = [Qdd_total; qdd_seg];
            else
                warning('Segmento %s devolvió datos vacíos.', archivo.name);
            end

            % 3. GUARDAMOS EL LÍMITE (COMO ANTES)
            seg_bounds(end+1) = size(Q_total, 1);

            pause(0.1);
        end
        
        fprintf('\n--- ¡Secuencia optimizada completada para %s! ---\n', prefijo_robot);
        disp('Mostrando gráficos de la secuencia completa...');
        
        if ~isempty(Q_total)
            robot_name = robot_a_ejecutar.name;
            if ~isempty(seg_bounds)
                seg_bounds(end) = [];
            end
            graficar_perfiles(robot_name, Q_total, Qd_total, Qdd_total, seg_bounds);
        else
            % Este mensaje ya no debería aparecer
            disp('No se acumularon datos para graficar.');
        end
    end

    %% --- HELPERS DEL ORQUESTADOR SÍNCRONO ---

    function [map, q0] = load_trajectory_data(robot, plan)
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
                if ~isKey(map, filename) 
                    filepath = fullfile(directorio_toppra, filename);
                    if ~exist(filepath, 'file')
                        % Intenta buscar en el directorio 'raw_trajectories'
                        % si es un archivo raw (como el arte latte)
                        raw_dir = fullfile(script_dir, 'raw_trajectories');
                        filepath_raw = fullfile(raw_dir, filename);
                        if exist(filepath_raw, 'file')
                            filepath = filepath_raw;
                            % Los archivos RAW no tienen datos de toppra
                            % Necesitamos cargarlos y 'simular' la estructura
                            raw_data = load(filepath);
                            datos.q_toppra = raw_data.waypoints_q;
                            % Asumir una velocidad fija para el arte latte
                            T_ARTE = 5.0; % Duración 5 segundos
                            datos.ts_sample = linspace(0, T_ARTE, size(raw_data.waypoints_q, 1));
                            datos.qd_toppra = [zeros(1, size(datos.q_toppra,2)); diff(datos.q_toppra)] ./ (T_ARTE / size(datos.q_toppra,1));
                            datos.qdd_toppra = [zeros(1, size(datos.q_toppra,2)); diff(datos.qd_toppra)] ./ (T_ARTE / size(datos.q_toppra,1));
                            
                        else
                             error('Archivo de trayectoria FALTANTE: %s', filename);
                        end
                    else
                         datos = load(filepath);
                    end
                    
                    map(filename) = datos;
                    
                    if isempty(q0)
                        q0 = datos.q_toppra(1,:);
                    end
                end
            end
        end
        if isempty(q0)
           q0 = zeros(1, robot.n); 
           warning('No se cargó ningún segmento para %s. Usando q0 = [0...0]', robot.name);
        end
    end

    function [state] = init_robot_state(q_inicial)
        state = struct(...
            'plan_index', 1, ...         
            'segment_data', [], ...      
            'segment_time', 0, ...       
            'segment_duration', 0, ...   
            'wait_remaining', 0, ...     
            'last_q', q_inicial, ...     
            'finished', false ...        
        );
    end

    function [state, q] = update_robot_state(state, plan, trajectories, dt_update)
        if state.finished
            q = state.last_q;
            return;
        end
        
        if state.wait_remaining > 0
            state.wait_remaining = state.wait_remaining - dt_update;
            q = state.last_q; 
            return;
        end
        
        if isempty(state.segment_data)
            if state.plan_index > numel(plan)
                state.finished = true;
                q = state.last_q;
                return;
            end
            
            current_step = plan{state.plan_index};
            state.plan_index = state.plan_index + 1;
            
            if strcmp(current_step{1}, 'delay')
                state.wait_remaining = current_step{2}; 
                q = state.last_q; 
                return;
            elseif strcmp(current_step{1}, 'segmento')
                segment_name = current_step{2};
                state.segment_data = trajectories(segment_name);
                state.segment_time = 0; 
                state.segment_duration = state.segment_data.ts_sample(end);
            end
        end
        
        state.segment_time = state.segment_time + dt_update;
        
        if state.segment_time >= state.segment_duration
            q = state.segment_data.q_toppra(end, :); 
            state.last_q = q;
            state.segment_data = []; 
        else
            q = interp1(state.segment_data.ts_sample, state.segment_data.q_toppra, state.segment_time, 'linear');
            state.last_q = q; 
        end
    end

    % ====================================================================
    % --- FIN: DEFINICIONES DE FUNCIONES ANIDADAS ---
    % ====================================================================

end