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

for k = 2:numel(plist) % el segundo punto es el primer "destino"
    tipo_movimiento = plist{k}.tipo;

    fprintf('Moviendo de waypoint %d → %d (pausa de %f segundos)\n', k-1, k,tf);

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
            
            t_articular = 1:n; %tiempo articular
            graficar_perfiles(t_articular,qt, qd, 'Articular');

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
            q_tray_plot = q_tray_cart(1:end-1, :);
            t_cartesiano = 1:(N-1);
            graficar_perfiles(t_cartesiano, q_tray_plot, qd_tray_cart, 'Cartesiana');
        otherwise
            warning('Tipo de movimiento "%s" no reconocido. Saltando segmento.', tipo_movimiento);
    end

    pause(0.5);
    q_curr = qseq(:,k);
end
disp('Trayectoria completada ');
end
