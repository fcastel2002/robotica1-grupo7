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
                %Se saco TCP de ''
                h_frame = trplot(T_tcp, 'frame', '', 'color', 'b', 'length', 0.2,'width',0.2,'arrow');
                setappdata(fig,'h_tcp_frame', h_frame);
                drawnow;
            end
            
            % Acumular trayectoria
            Q_completo = [Q_completo; qt];
            Qd_completo = [Qd_completo; qd];
            seg_bounds(end+1) = size(Q_completo,1); %#ok<AGROW>
            
            t_articular = 1:n; %tiempo articular
            % No graficar aquí, se hará al final

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
            
            % No graficar aquí, se hará al final
            
        case 'articular_relativo'
            % Movimiento articular RELATIVO a la pose anterior (q_curr)
            % Usado para acciones de herramienta en singularidades (ej. volcar)
            % No usa IK, por lo tanto no hay picos de velocidad.
            
            q_start = q_curr;
            
            % La 'pose' es el vector de offset articular
            q_relativo = plist{k}.pose;
            
            % Verificamos que sea un vector 1x6
            if numel(q_relativo) ~= R.n
                error('El tipo "articular_relativo" requiere una "pose" de [1x%d] (vector de offset q)', R.n);
            end
            
            q_end = q_start + q_relativo(:); % Asegurar que sea columna
            
            % Opcional: Verificar límites de la pose final
            if ~all(q_end >= R.qlim(:, 1) & q_end <= R.qlim(:, 2))
                warning('MOVIMIENTO RELATIVO RESULTA FUERA DE LÍMITES');
                disp('Q-Start:'); disp(q_start);
                disp('Q-Rel:'); disp(q_relativo);
                disp('Q-End:'); disp(q_end);
            end

            % Calcular trayectoria JTRAJ (usamos N=15 pasos fijos para esta acción)
            N_segmento = 15;
            [q_tray_segmento, qd_tray_segmento] = jtraj(q_start', q_end', N_segmento);
            
            % Animar y acumular
            for i = 1:N_segmento
                R.animate(q_tray_segmento(i, :));
                
                if ~isempty(h_frame), delete(h_frame); end
                T_tcp = R.fkine(q_tray_segmento(i, :));
                h_frame = trplot(T_tcp, 'frame', '', 'color', 'm', 'length', 0.2,'width',0.2,'arrow');
                setappdata(fig,'h_tcp_frame', h_frame);
                drawnow;
                
                % (Pausa dt)
                if TF > 0, pause(TF/N_segmento); else, pause(0.01); end
            end
            
            % Acumular trayectoria completa
            Q_completo = [Q_completo; q_tray_segmento];
            Qd_completo = [Qd_completo; qd_tray_segmento];
            seg_bounds(end+1) = size(Q_completo,1); %#ok<AGROW>
            
            % Actualizar estado para el próximo bucle
            Tlist{k} = R.fkine(q_end').double; % Actualizamos Tlist con el resultado
            q_curr = q_end; % Usar q_end en lugar de qseq(:,k) - será sobrescrito al final pero no importa
            
        otherwise
            warning('Tipo de movimiento "%s" no reconocido. Saltando segmento.', tipo_movimiento);
    end

    pause(0.5);
    % Para 'articular_relativo', q_curr ya fue actualizado dentro del case
    % Para otros tipos, usar qseq(:,k)
    if ~strcmp(tipo_movimiento, 'articular_relativo')
        q_curr = qseq(:,k);
    end
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

disp('Trayectoria completada ');
end
