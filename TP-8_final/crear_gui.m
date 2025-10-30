function crear_gui(R, plist, Tlist, qseq, n, N)
% Crea botones en la figura activa para ejecutar:
%   - Todo el recorrido
%   - Un segmento por vez (k-1 -> k)
% Usa sublistas para no alterar lógica interna de 'ejecutar_trayectorias'.

if nargin < 5, n = 30; end
if nargin < 6, N = 20; end

fig = gcf;
 
% Layout derecho simple robot 1
x = 0.85; w = 0.12; h = 0.06; dy = 0.08; y0 = 0.80;

% Botón "Todo"
uicontrol('Parent',fig,'Style','pushbutton','Units','normalized', ...
    'Position',[x y0 w h], 'String','Ejecutar TODO', ...
    'FontSize',11, ...
    'Callback', @(~,~) ejecutar_trayectorias(R(1), plist{1}, Tlist{1}, qseq{1}, n, N));

% Botón especial: Arte latte (corazón)
y = y0 - dy;
uicontrol('Parent',fig,'Style','pushbutton','Units','normalized', ...
    'Position',[x y w h], 'String','Arte latte (corazón)', ...
    'FontSize',11, ...
    'Callback', @(~,~) animar_corazon(R(1), plist{1}));

% Botones por segmento
y = y - dy;
for k = 2:numel(plist{1})
    txt = sprintf('Punto %d → %d', k-1, k);
    
    uicontrol('Parent',fig,'Style','pushbutton','Units','normalized', ...
        'Position',[x y w h], 'String',txt, 'FontSize',10, ...
        'Callback', @(~,~) ejecutar_tramo(k,R(1),plist{1},Tlist{1},qseq{1}));
    y = y - dy;
end

% Layout izquierdo simple robot 2
x = 0.03; w = 0.12; h = 0.06; dy = 0.08; y0 = 0.80;

% Botón "Todo"
uicontrol('Parent',fig,'Style','pushbutton','Units','normalized', ...
    'Position',[x y0 w h], 'String','Ejecutar TODO R2', ...
    'FontSize',11, ...
    'Callback', @(~,~) ejecutar_trayectorias(R(2), plist{2}, Tlist{2}, qseq{2}, n, N));

% Botones por segmento
y = y0 - dy;
for k = 2:numel(plist{2})
    txt = sprintf('Punto R2 %d → %d', k-1, k);
    uicontrol('Parent',fig,'Style','pushbutton','Units','normalized', ...
        'Position',[x y w h], 'String',txt, 'FontSize',10, ...
        'Callback', @(~,~) ejecutar_tramo(k,R(2),plist{2},Tlist{2},qseq{2}));
    y = y - dy;
end


%% helpers
    function ejecutar_tramo(k,R,plist,Tlist,qseq)
        % Subconjuntos consistentes para mantener la lógica intacta
        plist_sub = plist(k-1:k);
        Tlist_sub = Tlist(k-1:k);
        qseq_sub  = qseq(:,k-1:k);

        ejecutar_trayectorias(R, plist_sub, Tlist_sub, qseq_sub, n, N);
    end
    
    function animar_corazon(R, plist)
        % Usa la pose 6 (volcando leche) como referencia
        if numel(plist) < 6
            warning('No existe la pose 6 en plist para generar el corazón.');
            return;
        end
        pose_cafe_vec = plist{6}.pose; % [x y z r p y] (coordenadas base de R)
        % Centro/orientación en marco BASE del robot de leche (R1)
        centro_taza = [0.499, 0.0, 0.55];
        rpy_fijo = [0.01, pi/2+0.5, 0];

        % Parámetros
        N_puntos = 50;
        escala = 0.003;
        t = linspace(pi, -pi, N_puntos)';

        % Corazón 2D (ajustado: inicia en el punto base y avanza hacia X positiva)
        x_raw = escala * (13*cos(t) - 5*cos(2*t) - 2*cos(3*t) - cos(4*t));
        y_raw = escala * (16 * sin(t).^3);
        % Normalizar para que el punto inicial sea el centro de la taza (offset 0)
        x_rel = (x_raw - x_raw(1));
        y_rel = (y_raw - y_raw(1));

        % Hitos cartesianos [x y z r p y]
        puntos_X = centro_taza(1) + x_rel;
        puntos_Y = centro_taza(2) + y_rel;
        puntos_Z = ones(N_puntos, 1) * centro_taza(3);
        matriz_RPY = repmat(rpy_fijo, N_puntos, 1);
        Cart_puntos = [puntos_X, puntos_Y, puntos_Z, matriz_RPY];

        % mstraj en espacio cartesiano
        dt = 0.05; v_max = 0.10; t_blend = 0.10;
        q_inicial = Cart_puntos(1,:);
        Cart_traj = mstraj(Cart_puntos, v_max, [], q_inicial, dt, t_blend);

        % Mantener cámara actual (sin forzar zoom)
        hold on; grid on;

        % IK + animación (comenzar desde postura actual si disponible)
        try
            q_anterior = R.getpos()';
        catch
            q_anterior = zeros(6,1);
        end
        for i = 1:size(Cart_traj,1)
            vec_cart = Cart_traj(i,:);
            % Construir pose en MARCO BASE y convertir a MARCO MUNDO
            T_base = transl(vec_cart(1:3)) * rpy2tr(vec_cart(4:6),'zyx');
            T_k = R.base.double * T_base;
            q_next = ik_barista(R, T_k, q_anterior, true);
            R.animate(q_next');
            q_anterior = q_next;
            drawnow;
        end
        title('Arte latte: trayectoria de corazón');
    end


end
