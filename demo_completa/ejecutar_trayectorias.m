function ejecutar_trayectorias(R, plist, Tlist, qseq, n, N,TF)
% Ejecuta los segmentos según la lógica original.
% Entradas:
%   R, plist, Tlist, qseq   -> mismos objetos del script
%   n                       -> muestras por segmento articular
%   N                       -> puntos de interpolación cartesiana

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

for k = 2:numel(plist) % el segundo punto es el primer "destino"
    tipo_movimiento = plist{k}.tipo;

    fprintf('Moviendo de waypoint %d → %d (pausa de %f segundos)\n', k-1, k, TF);

    switch tipo_movimiento
        
        case 'articular'
            [qt, qd, qdd] = jtraj(q_curr', qseq(:,k)', n); %#ok<ASGLU>

            for i = 1:n
                R.animate(qt(i,:));

                if ~isempty(h_frame), delete(h_frame); end

                T_tcp = R.fkine(qt(i,:));
                h_frame = trplot(T_tcp, 'frame', '', 'color', 'b', 'length', 0.2,'width',0.2,'arrow');
                setappdata(fig,'h_tcp_frame', h_frame);
                drawnow;
            end
            
            % Acumular trayectoria
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
                R.animate(q_next');
                q_intermedio = q_next;  
                q_tray_cart(i,:) = q_next';
                if ~isempty(h_frame), delete(h_frame); end
                %Se saco TCP de ''
                h_frame = trplot(Ts(i), 'frame', '', 'color', 'g', 'length', 0.2,'width',0.2);
                T_tcp = R.fkine(q_next');
                set(h_frame, 'Matrix', T_tcp.double);
                setappdata(fig,'h_tcp_frame', h_frame);
                
                drawnow;
            end
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
                R.animate(q_next');
                q_intermedio = q_next;
                q_tray_cart(i,:) = q_next';
                if ~isempty(h_frame), delete(h_frame); end
                %Se saco TCP de ''
                h_frame = trplot(Ts(i), 'frame', '', 'color', 'g', 'length', 0.2,'width',0.2);
                T_tcp = R.fkine(q_next');
                set(h_frame, 'Matrix', T_tcp.double);
                setappdata(fig,'h_tcp_frame', h_frame);
                drawnow;
            end
            qd_tray_cart = diff(q_tray_cart)*N;
            % Agregar última fila de velocidades (cero)
            qd_tray_cart = [qd_tray_cart; zeros(1, size(q_tray_cart,2))];

            % Acumular trayectoria
            Q_completo = [Q_completo; q_tray_cart];
            Qd_completo = [Qd_completo; qd_tray_cart];
            seg_bounds(end+1) = size(Q_completo,1); %#ok<AGROW>
        case 'toppra'
            ejecutar_toppra(R,'trayectoria_optimizada.mat')
        otherwise
            warning('Tipo de movimiento "%s" no reconocido. Saltando segmento.', tipo_movimiento);
    end

    pause(0.5);
    q_curr = qseq(:,k);
end

% Graficar trayectorias completas en ventanas nuevas
if ~isempty(Q_completo)
    robot_name = R.name;
    
    colors = lines(6);
    labels = arrayfun(@(i) sprintf('q%d',i), 1:6, 'UniformOutput', false);
    
    % Ventana 1: Posiciones (nueva ventana sin número específico)
    fig1 = figure();
    set(fig1, 'Name', sprintf('%s - Posiciones', robot_name));
    hold on; grid on;
    for i=1:6
        plot(Q_completo(:,i), 'Color', colors(i,:), 'LineWidth',1.2);
    end
    legend(labels{:}, 'Location','best');
    title(sprintf('%s - q1..q6', robot_name));
    xlabel('muestra'); ylabel('rad');
    % Marcar límites de segmentos
    for i=1:numel(seg_bounds)
        xline(seg_bounds(i), 'r--', 'HandleVisibility','off');
    end
    
    % Ventana 2: Velocidades (nueva ventana sin número específico)
    fig2 = figure();
    set(fig2, 'Name', sprintf('%s - Velocidades', robot_name));
    hold on; grid on;
    for i=1:6
        plot(Qd_completo(:,i), 'Color', colors(i,:), 'LineWidth',1.2);
    end
    legend(labels{:}, 'Location','best');
    title(sprintf('%s - dq1..dq6', robot_name));
    xlabel('muestra'); ylabel('rad/s');
    % Marcar límites de segmentos
    for i=1:numel(seg_bounds)
        xline(seg_bounds(i), 'r--', 'HandleVisibility','off');
    end
end
disp('Exportando trayectoria para Toppra...');

% 1. Los waypoints son tu trayectoria completa
waypoints_q = Q_completo;

% 2. Creamos el vector de posición de trayectoria (path position 's')
%    Lo normalizamos para que vaya de 0 a 1.
num_muestras = size(waypoints_q, 1);
path_pos_s = linspace(0, 1, num_muestras)'; % Vector columna

try
    % 'mfilename('fullpath')' obtiene la ruta completa del script actual (ej: C:\..._trayectorias.m)
    % 'fileparts' extrae solo la parte de la carpeta (ej: C:\...MiProyecto\)
    [script_dir, ~, ~] = fileparts(mfilename('fullpath'));

    % Define el nombre del archivo
    nombre_archivo_base = 'trayectoria_para_toppra.mat';
    
    % 'fullfile' une la ruta y el nombre de forma segura (funciona en Windows, Mac y Linux)
    nombre_archivo_completo = fullfile(script_dir, nombre_archivo_base);

    % Guardar las variables en esa ruta completa
    save(nombre_archivo_completo, 'waypoints_q', 'path_pos_s');

    fprintf('Trayectoria guardada exitosamente en: \n%s\n', nombre_archivo_completo);

catch e
    fprintf('ERROR AL INTENTAR GUARDAR EL ARCHIVO: %s\n', e.message);
    disp('Asegúrese de tener permisos de escritura en la carpeta del script.');

end
disp('Trayectoria completada ');
end
