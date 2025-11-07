function ejecutar_trayectorias(R, plist, Tlist, qseq, n, N,TF)
% Ejecuta los segmentos según la lógica original.
% Entradas:
%   R, plist, Tlist, qseq   -> mismos objetos del script
%   n                       -> muestras por segmento articular
%   N                       -> puntos de interpolación cartesiana
%% DEBUG
    global animar;
%%
%% Aclaracion respecto a las trayectorias
% Las trayectorias generadas en este archivo son las que finalmente el
% robot ejecutará. Más bien, se usaron como base para crear las
% trayectorias elementales, luego estás son posprocesadas usando la
% libreria "toppra" en python, este manejo se realiza a través de archivos
% .mat 
if nargin < 5, n = 30; end
if nargin < 6, N = 20; end
if nargin < 7, TF = 0.5; end

q_curr = qseq(:,1);
fig = gcf;
h_ant = getappdata(fig,'h_tcp_frame');
if ~isempty(h_ant) && isgraphics(h_ant)
    delete(h_ant);
end
h_frame = [];
axlen = 0.3;
T0_tcp = R.fkine(q_curr');
h_frame = trplot( eye(4), 'frame','TCP', 'color','b', 'length',axlen, 'width',0.2, 'arrow' );
set(h_frame,'Matrix', T0_tcp.double);
setappdata(fig,'h_tcp_frame', h_frame);

% Acumular trayectorias completas para gráficos finales
Q_completo = [];
Qd_completo = [];
seg_bounds = [];
dt_seg = 0.05;  % dt aproximado por muestra

try
    [script_dir, ~, ~] = fileparts(mfilename('fullpath'));
    output_dir = fullfile(script_dir, 'raw_trajectories');
    if ~exist(output_dir, 'dir')
        mkdir(output_dir);
        fprintf('Directorio creado: %s\n', output_dir);
    end
catch e
    fprintf('ERROR AL CREAR DIRECTORIO: %s\n', e.message);
    output_dir = pwd; % Usar directorio actual como fallback
end

for k = 2:numel(plist) % el segundo punto es el primer "destino"
    tipo_movimiento = plist{k}.tipo;
    q_segmento = [];
    fprintf('Moviendo de waypoint %d → %d (tipo: %s)\n', k-1, k, tipo_movimiento);

    switch tipo_movimiento
        
        % --- NUEVO CASO 'directa' ---
        case 'directa'
            % Este movimiento es articular, usa jtraj para un perfil suave.
            % qseq(:,k) ya contiene el q final (calculado en barista.m)
            fprintf('--- Movimiento articular directo ---\n');
            [qt, qd, qdd] = jtraj(q_curr', qseq(:,k)', n); %#ok<ASGLU>

            for i = 1:n


                if ~isempty(h_frame), delete(h_frame); end

                T_tcp = R.fkine(qt(i,:));
                % Color magenta para distinguirlo
                h_frame = trplot(T_tcp, 'frame', '', 'color', 'm', 'length', 0.2,'width',0.2,'arrow');
                if(animar)
                    R.animate(qt(i,:));
                    setappdata(fig,'h_tcp_frame', h_frame);
                    drawnow;

                end
            end
            
            % Acumular trayectoria
            q_segmento = qt;
            Q_completo = [Q_completo; qt];
            Qd_completo = [Qd_completo; qd];
            seg_bounds(end+1) = size(Q_completo,1); %#ok<AGROW>
        % --- FIN NUEVO CASO ---
        
        case 'articular'
            [qt, qd, qdd] = jtraj(q_curr', qseq(:,k)', n); %#ok<ASGLU>

            for i = 1:n
                
                if ~isempty(h_frame), delete(h_frame); end

                T_tcp = R.fkine(qt(i,:));
                h_frame = trplot(T_tcp, 'frame', '', 'color', 'b', 'length', 0.2,'width',0.2,'arrow');
                if(animar)
                    R.animate(qt(i,:));
                    setappdata(fig,'h_tcp_frame', h_frame);
                    drawnow;
                end
                
            end
            
            % Acumular trayectoria
            q_segmento = qt;
            Q_completo = [Q_completo; qt];
            Qd_completo = [Qd_completo; qd];
            seg_bounds(end+1) = size(Q_completo,1); %#ok<AGROW>
            
            t_articular = 1:n; %tiempo articular
        
        case 'cartesiana'
            T_start = R.fkine(q_curr);
            T_end = SE3(Tlist{k});
            Ts = ctraj(T_start, T_end, N);
            q_tray_cart = zeros(N, R.n); % Matriz para guardar la trayectoria
            q_intermedio = q_curr;

            for i = 1:N
                q_next = ik_barista(R, Ts(i), q_intermedio, true);
              
                q_intermedio = q_next;  
                q_tray_cart(i,:) = q_next';
                if ~isempty(h_frame), delete(h_frame); end
                %Se saco TCP de ''
                h_frame = trplot(Ts(i), 'frame', '', 'color', 'g', 'length', 0.2,'width',0.2);
                T_tcp = R.fkine(q_next');
                set(h_frame, 'Matrix', T_tcp.double);
                 if(animar)
                    R.animate(q_next');
                    setappdata(fig,'h_tcp_frame', h_frame);
                
                    drawnow;
                end 
            end
            q_segmento = q_tray_cart;
            qd_tray_cart = diff(q_tray_cart)*N;

            % Agregar última fila de velocidades (cero)
            qd_tray_cart = [qd_tray_cart; zeros(1, size(q_tray_cart,2))];
            
            % Acumular trayectoria
            Q_completo = [Q_completo; q_tray_cart];
            Qd_completo = [Qd_completo; qd_tray_cart];
            seg_bounds(end+1) = size(Q_completo,1); %#ok<AGROW>
            
        case 'cartesiana_fina'
            t = 60;
            T_start = R.fkine(q_curr);
            T_end = SE3(Tlist{k});
            Ts = ctraj(T_start, T_end, t);
            q_tray_cart = zeros(N, R.n); % Matriz para guardar la trayectoria
            q_intermedio = q_curr;

            for i = 1:t
                q_next = ik_barista(R, Ts(i), q_intermedio, true);
             
                q_intermedio = q_next;
                q_tray_cart(i,:) = q_next';
                if ~isempty(h_frame), delete(h_frame); end
                %Se saco TCP de ''
                h_frame = trplot(Ts(i), 'frame', '', 'color', 'g', 'length', 0.2,'width',0.2);
                T_tcp = R.fkine(q_next');
                set(h_frame, 'Matrix', T_tcp.double);
                   if(animar)
                    R.animate(q_next');
                   
                setappdata(fig,'h_tcp_frame', h_frame);
                drawnow;
                   end
            end
            qd_tray_cart = diff(q_tray_cart)*N;
            % Agregar última fila de velocidades (cero)
            qd_tray_cart = [qd_tray_cart; zeros(1, size(q_tray_cart,2))];
            q_segmento = q_tray_cart;

            % Acumular trayectoria
            Q_completo = [Q_completo; q_tray_cart];
            Qd_completo = [Qd_completo; qd_tray_cart];
            seg_bounds(end+1) = size(Q_completo,1); %#ok<AGROW>
        case 'toppra'
            ejecutar_toppra(R,'trayectoria_optimizada.mat')
        otherwise
            warning('Tipo de movimiento "%s" no reconocido. Saltando segmento.', tipo_movimiento);
    end
    if ~isempty(q_segmento)
        waypoints_q = q_segmento;
        num_muestras = size(waypoints_q, 1);
        path_pos_s = linspace(0, 1, num_muestras)';

        % nombre de archivo
        robot_prefix = R.name;
        start_idx = k-1;
        end_idx = k;
        nombre_archivo_base = sprintf('%s_rawtraj_%d-%d.mat', robot_prefix, start_idx, end_idx);
        nombre_archivo_completo = fullfile(output_dir, nombre_archivo_base);
        try
            save(nombre_archivo_completo, 'waypoints_q', "path_pos_s");
        catch e
            fprintf('Error al guardar %s: %s\n', nombre_archivo_base);
        end

    elseif ~strcmp(tipo_movimiento, 'toppra') && k>1
        fprintf('Trayectoria ya optimizada, no se genero ninguna nueva.');
    end
    pause(0.5);
    q_curr = qseq(:,k);
end

% Graficar trayectorias completas en ventanas nuevas
if ~isempty(Q_completo)
    robot_name = R.name;
    
    if ~isempty(seg_bounds) && seg_bounds(end) >= size(Q_completo, 1)
        seg_bounds(end) = [];
    end
    graficar_perfiles(robot_name, Q_completo, Qd_completo, [], seg_bounds);
end

disp('Trayectoria completada ');
end