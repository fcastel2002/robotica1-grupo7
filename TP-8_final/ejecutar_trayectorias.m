function ejecutar_trayectorias(R, plist, Tlist, qseq, n, N)
% Ejecuta los segmentos según la lógica original.
% Entradas:
%   R, plist, Tlist, qseq   -> mismos objetos del script
%   n                       -> muestras por segmento articular
%   N                       -> puntos de interpolación cartesiana

if nargin < 5, n = 30; end
if nargin < 6, N = 20; end

q_curr = qseq(:,1);
h_frame = [];

for k = 2:numel(plist)
    tipo_movimiento = plist{k}.tipo;
    fprintf('Moviendo de waypoint %d → %d (pausa de 1 segundo)\n', k-1, k);

    switch tipo_movimiento
        case 'articular'
            [qt, qd, qdd] = jtraj(q_curr', qseq(:,k)', n); %#ok<ASGLU>

            for i = 1:n
                R.animate(qt(i,:));

                if ~isempty(h_frame), delete(h_frame); end

                T_tcp = R.fkine(qt(i,:));
                h_frame = trplot(T_tcp, 'frame', 'TCP', 'color', 'b', 'length', 0.2,'width',0.2);
                drawnow;
            end

        case 'cartesiano'
            T_start = R.fkine(q_curr);
            T_end = SE3(Tlist{k});
            Ts = ctraj(T_start, T_end, N);
            q_intermedio = q_curr;

            for i = 1:N
                q_next = ik_barista(R, Ts(i), q_intermedio, true);
                R.animate(q_next');
                q_intermedio = q_next;   % corrige nombre usado en el bucle
                if ~isempty(h_frame), delete(h_frame); end
                h_frame = trplot(Ts(i), 'frame', 'TCP', 'color', 'g', 'length', 0.1);
                drawnow;
            end

        otherwise
            warning('Tipo de movimiento "%s" no reconocido. Saltando segmento.', tipo_movimiento);
    end

    pause(0.5);
    q_curr = qseq(:,k);
end

disp('Trayectoria completada ');
end
