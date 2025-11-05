function crear_gui(R, plist, Tlist, qseq, n, N, groups)
% Crea botones en la figura activa para ejecutar:
%   - Todo el recorrido (raw)
%   - Todo el recorrido (optimizado con Toppra) <-- NUEVO
%   - Un segmento por vez (raw)
% Usa sublistas para no alterar lógica interna de 'ejecutar_trayectorias'.

if nargin < 5, n = 30; end
if nargin < 6, N = 20; end
if nargin < 7, groups = []; end

fig = gcf;
 
% Layout derecho simple robot 1
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
    'FontSize',11, ...
    'Callback', @(~,~) ejecutar_ambos_sync(R, plist, Tlist, qseq, n, N));

% --- CORRECCIÓN: 'y' se inicializa para el botón Optimizado ---
y = y0 - dy; % 'y' empieza en 0.72
y = y - dy; % Mover 'y' de nuevo (ahora y = 0.56) <-- Empezar segmentos aqui

% --- NUEVO BOTÓN: Ejecutar Optimizado R1 ---
uicontrol('Parent',fig,'Style','pushbutton','Units','normalized', ...
    'Position',[x y w h], 'String','Ejecutar Optimizado R1', ...
    'FontSize',11, 'FontWeight', 'bold', 'ForegroundColor', [0 0 0.8], ... % Color azul
    'Callback', @(~,~) ejecutar_secuencia_toppra(R(1)));
% --- FIN NUEVO ---

% Botones agrupados o por segmento (raw)
y = y - dy; % Mover 'y' hacia abajo DE NUEVO (ahora y = 0.64)
if ~isempty(groups)
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
    for k = 2:numel(plist{1})
    txt = sprintf('Punto %d → %d', k-1, k);
    uicontrol('Parent',fig,'Style','pushbutton','Units','normalized', ...
        'Position',[x y w h], 'String',txt, 'FontSize',10, ...
        'Callback', @(~,~) ejecutar_tramo(k,R(1),plist{1},Tlist{1},qseq{1}));
    y = y - dy;
    end
end

% Layout izquierdo simple robot 2
x = 0.03; w = 0.12; h = 0.06; dy = 0.08; y0 = 0.80;

% Botón "Todo" R2 (Raw)
uicontrol('Parent',fig,'Style','pushbutton','Units','normalized', ...
    'Position',[x y0 w h], 'String','Ejecutar TODO R2', ...
    'FontSize',11, ...
    'Callback', @(~,~) ejecutar_trayectorias(R(2), plist{2}, Tlist{2}, qseq{2}, n, N));

% --- NUEVO BOTÓN: Ejecutar Optimizado R2 ---
y = y0 - dy; % Mover 'y' hacia abajo
uicontrol('Parent',fig,'Style','pushbutton','Units','normalized', ...
    'Position',[x y w h], 'String','Ejecutar Optimizado R2', ...
    'FontSize',11, 'FontWeight', 'bold', 'ForegroundColor', [0 0 0.8], ... % Color azul
    'Callback', @(~,~) ejecutar_secuencia_toppra(R(2)));
% --- FIN NUEVO ---

% Botones agrupados o por segmento para R2 (raw)
y = y - dy; % Mover 'y' hacia abajo DE NUEVO
if ~isempty(groups)
    for g = 1:numel(groups{2})
        idxs = groups{2}{g};
        if isempty(idxs), continue; end
        i0 = idxs(1); i1 = idxs(end);
        etiqueta = sprintf('R2 %d -> %d', i0, i1);
        uicontrol('Parent',fig,'Style','pushbutton','Units','normalized', ...
            'Position',[x y w h], 'String',etiqueta, 'FontSize',10, ...
            'Callback', @(~,~) ejecutar_grupo(R(2), plist{2}, Tlist{2}, qseq{2}, idxs));
        y = y - dy;
    end
else
% (Resto del loop 'else' de segmentos individuales...)
for k = 2:numel(plist{2})
    txt = sprintf('Punto R2 %d → %d', k-1, k);
    uicontrol('Parent',fig,'Style','pushbutton','Units','normalized', ...
        'Position',[x y w h], 'String',txt, 'FontSize',10, ...
        'Callback', @(~,~) ejecutar_tramo(k,R(2),plist{2},Tlist{2},qseq{2}));
    y = y - dy;
    end
end


%% helpers (funciones internas)
function cerrar_figuras_anteriores()
        % (Esta función no se modifica)
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
    function ejecutar_tramo(k,R,plist,Tlist,qseq)
        % (Esta función no se modifica)
        cerrar_figuras_anteriores()
        plist_sub = plist(k-1:k);
        Tlist_sub = Tlist(k-1:k);
        qseq_sub  = qseq(:,k-1:k);
        ejecutar_trayectorias(R, plist_sub, Tlist_sub, qseq_sub, n, N);
    end
    function ejecutar_todo_r1(R, plist, Tlist, qseq, n, N)
        % (Esta función no se modifica)
        ejecutar_trayectorias(R, plist, Tlist, qseq, n, N);
        try
            arte_latte_corazon(R, [0.499 0 0.55], [0.01 pi/2+0.5 0]);
        catch ME
            warning('Fallo al ejecutar arte latte: %s', ME.message);
        end
    end
    function ejecutar_ambos_sync(R_all, plist_all, Tlist_all, qseq_all, n, N)
        % (Esta función no se modifica, es larga, así que la omito por brevedad)
        R1 = R_all(1); R2 = R_all(2);
        plist_R1 = plist_all{1}; Tlist_R1 = Tlist_all{1}; qseq_R1 = qseq_all{1};
        plist_R2 = plist_all{2}; Tlist_R2 = Tlist_all{2}; qseq_R2 = qseq_all{2};

        % 1) Plan completo R2
        [Q2, dt, seg2] = plan_full_traj(R2, plist_R2, Tlist_R2, qseq_R2, n, N, false);

        % 2) R1 por segmentos exactos: PRE = (1->2)+(2->3)+(3->4)+(4->5)
        [Q12, ~, seg12] = plan_full_traj(R1, plist_R1(1:2), Tlist_R1(1:2), qseq_R1(:,1:2), n, N, false); % 0->1
        [Q23, ~, seg23] = plan_full_traj(R1, plist_R1(2:3), Tlist_R1(2:3), qseq_R1(:,2:3), n, N, false); % 1->2
        [Q34, ~, seg34] = plan_full_traj(R1, plist_R1(3:4), Tlist_R1(3:4), qseq_R1(:,3:4), n, N, false); % 2->3
        [Q45, ~, seg45] = plan_full_traj(R1, plist_R1(5:6), Tlist_R1(5:6), qseq_R1(:,5:6), n, N, false); % 4->5
        Q1_pre = [Q12; Q23; Q34; Q45];
        seg1_pre.ends = [seg12.ends, size(Q12,1)+seg23.ends, size(Q12,1)+size(Q23,1)+seg34.ends, size(Q12,1)+size(Q23,1)+size(Q34,1)+seg45.ends];
        seg1_pre.labels = [seg12.labels, seg23.labels, seg34.labels, seg45.labels];
        % POST = (5->6) + arte latte (armado explícito)
        [Q56, ~, seg56] = plan_full_traj(R1, plist_R1(6:7), Tlist_R1(6:7), qseq_R1(:,6:7), n, N, false); % 5->6
        Tw_end6 = R1.fkine(Q56(end,:));
        T_base_inv = SE3(R1.base.double).inv();
        T_end6 = T_base_inv.double * Tw_end6.double;
        centro_end6 = T_end6(1:3,4)';
        rpy_end6 = tr2rpy(T_end6, 'zyx');
        [Q_arte_temp, ~] = plan_arte_latte_q(R1, centro_end6, rpy_end6);
        Q_arte = [Q56(end,:); Q_arte_temp(2:end,:)];
        Q1_post = [Q56; Q_arte(2:end,:)];
        seg1_post.ends = [seg56.ends, size(Q56,1) + size(Q_arte,1) - 1];
        seg1_post.labels = [seg56.labels, {'arte latte'}];

        % 3) Sincronía: R1 espera a que R2 termine
        wait_len = max(size(Q2,1) - size(Q1_pre,1), 0);
        pad = repmat(Q1_pre(end,:), wait_len, 1);
        q_pre_end = Q1_pre(end,:);
        q_post_start = Q1_post(1,:);
        blend_len = max(10, round(0.3/dt));
        if max(abs(q_post_start - q_pre_end)) > 1e-6
            [qblend, ~, ~] = jtraj(q_pre_end, q_post_start, blend_len);
        else
            qblend = zeros(0, size(Q1_pre,2));
        end
        Q1_sync = [Q1_pre; pad; qblend; Q1_post];

        % 4) Animación simultánea
        arte_start_post = size(Q56,1) + 1;
        arte_start_global = size(Q1_pre,1) + wait_len + size(qblend,1) + arte_start_post;
        htrail = [];
        K = max(size(Q1_sync,1), size(Q2,1));
        for k = 1:K
            if k <= size(Q2,1)
                R2.animate(Q2(k,:));
            end
            if k <= size(Q1_sync,1)
                R1.animate(Q1_sync(k,:));
                if k >= arte_start_global
                    if isempty(htrail) || ~isvalid(htrail)
                        htrail = animatedline('Color',[0.8 0 0], 'LineWidth', 2);
                    end
                    T_tcp = R1.fkine(Q1_sync(k,:));
                    p = T_tcp.t;
                    addpoints(htrail, p(1), p(2), p(3));
                end
            end
            drawnow;
            pause(dt);
        end

        % 5) Graficar
        bnds_r1 = [];
        bnds_r1 = [bnds_r1, seg1_pre.ends];
        if wait_len>0
            bnds_r1(end+1) = size(Q1_pre,1) + wait_len;
        end
        off = size(Q1_pre,1) + wait_len;
        bnds_r1 = [bnds_r1, off + seg1_post.ends];
        colors = lines(6);
        labels = arrayfun(@(i) sprintf('q%d',i), 1:6, 'UniformOutput', false);

        f1 = figure('Name','R1 - Posiciones');
        hold on; grid on;
        for i=1:6, plot(Q1_sync(:,i), 'Color', colors(i,:), 'LineWidth',1.2); end
        legend(labels{:}, 'Location','best'); title('R1 - q'); xlabel('muestra'); ylabel('rad');
        for i=1:numel(bnds_r1), xline(bnds_r1(i),'r--'); end

        Q1d = [diff(Q1_sync)/dt; zeros(1,size(Q1_sync,2))];
        f2 = figure('Name','R1 - Velocidades');
        hold on; grid on;
        for i=1:6, plot(Q1d(:,i), 'Color', colors(i,:), 'LineWidth',1.2); end
        legend(labels{:}, 'Location','best'); title('R1 - dq'); xlabel('muestra'); ylabel('rad/s');
        for i=1:numel(bnds_r1), xline(bnds_r1(i),'r--'); end

        f3 = figure('Name','R2 - Posiciones');
        hold on; grid on;
        for i=1:6, plot(Q2(:,i), 'Color', colors(i,:), 'LineWidth',1.2); end
        legend(labels{:}, 'Location','best'); title('R2 - q'); xlabel('muestra'); ylabel('rad');
        for i=1:numel(seg2.ends), xline(seg2.ends(i),'r--'); end

        Q2d = [diff(Q2)/dt; zeros(1,size(Q2,2))];
        f4 = figure('Name','R2 - Velocidades');
        hold on; grid on;
        for i=1:6, plot(Q2d(:,i), 'Color', colors(i,:), 'LineWidth',1.2); end
        legend(labels{:}, 'Location','best'); title('R2 - dq'); xlabel('muestra'); ylabel('rad/s');
        for i=1:numel(seg2.ends), xline(seg2.ends(i),'r--'); end
    end
    function ejecutar_grupo(R, plist, Tlist, qseq, idxs)
        % (Esta función no se modifica)
        cerrar_figuras_anteriores()
        plist_sub = plist(idxs);
        Tlist_sub = Tlist(idxs);
        qseq_sub  = qseq(:, idxs);
        ejecutar_trayectorias(R, plist_sub, Tlist_sub, qseq_sub, n, N);
        drawnow;      
        pause(0.05);  
        figure(fig);
    end
    function plot_q_evol(Q, dt, figTitle)
        % (Esta función no se modifica)
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

% --- NUEVA FUNCIÓN HELPER (CALLBACK) ---
    function ejecutar_secuencia_toppra(robot_a_ejecutar)
        % (Esta función no se modifica)
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
        
        for i = 1:numel(lista_archivos_ordenada)
            archivo = lista_archivos_ordenada(i);
            ruta_completa = fullfile(archivo.folder, archivo.name);
            
            fprintf('--- [%d/%d] Ejecutando: %s ---\n', i, numel(lista_archivos_ordenada), archivo.name);
            
            if ~exist('ejecutar_toppra', 'file')
                msgbox('Error: La función "ejecutar_toppra.m" no se encuentra en el path de MATLAB.', 'Error de función', 'error');
                return;
            end
            
            ejecutar_toppra(robot_a_ejecutar, ruta_completa, false);
            
            pause(0.1);
        end
        fprintf('\n--- ¡Secuencia optimizada completada para %s! ---\n', prefijo_robot);
        
        disp('Mostrando gráficos del último segmento...');
        ejecutar_toppra(robot_a_ejecutar, ruta_completa, true);
    end
% --- FIN NUEVA FUNCIÓN ---

end