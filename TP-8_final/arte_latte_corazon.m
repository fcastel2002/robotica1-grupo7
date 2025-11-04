function arte_latte_corazon(R, centro_base_xyz, rpy_fijo)
% ARTE_LATTE_CORAZON Genera y anima una trayectoria de corazón en el marco BASE de R.
% Uso:
%   arte_latte_corazon(R1)  % Modo dinámico: usa pose actual del robot
%   arte_latte_corazon(R1, [0.499 0 0.55], [0.01 pi/2+0.5 0])  % Modo manual
% - R: SerialLink del robot (R1 leche)
% - centro_base_xyz: [x y z] en marco BASE del robot (opcional, se calcula dinámicamente si no se proporciona)
% - rpy_fijo: [r p y] en rad, orientación constante del efector (opcional, se calcula dinámicamente si no se proporciona)

    % Si no se proporcionan parámetros, calcular dinámicamente desde la pose actual
    q_actual = [];  % Inicializar para evitar errores de alcance
    if nargin < 2 || isempty(centro_base_xyz)
        % Obtener pose articular actual del robot
        try
            q_actual = R.getpos()';
        catch
            q_actual = zeros(6, 1);
        end
        
        % Calcular T_lienzo en marco MUNDO
        T_lienzo_world = R.fkine(q_actual);
        
        % Convertir T_lienzo a marco BASE: T_base = inv(R.base) * T_mundo
        T_base_inv = SE3(R.base.double).inv();
        T_lienzo_base = T_base_inv.double * T_lienzo_world.double;
        
        % Extraer posición y orientación desde T_lienzo
        centro_base_xyz = T_lienzo_base(1:3, 4)';  % [x y z] en BASE
        if nargin < 3 || isempty(rpy_fijo)
            rpy_fijo = tr2rpy(T_lienzo_base, 'zyx');   % [r p y] en BASE
        end
    elseif nargin < 3 || isempty(rpy_fijo)
        % Si se proporciona centro pero no rpy, calcular rpy desde pose actual
        try
            q_actual = R.getpos()';
        catch
            q_actual = zeros(6, 1);
        end
        T_lienzo_world = R.fkine(q_actual);
        T_base_inv = SE3(R.base.double).inv();
        T_lienzo_base = T_base_inv.double * T_lienzo_world.double;
        rpy_fijo = tr2rpy(T_lienzo_base, 'zyx');
    end

    % Parámetros de forma/velocidad
    N_puntos = 50;       % hitos de forma
    escala   = 0.003;    % ~6 cm de tamaño
    dt       = 0.05;     % 50 ms
    v_max    = 0.50;     % m/s y rad/s (equivalente)
    t_blend  = 0.10;     % s

    % Corazón paramétrico 2D en marco BASE (XY), comenzando en el centro
    t = linspace(pi, -pi, N_puntos)';
    x_raw = escala * (13*cos(t) - 5*cos(2*t) - 2*cos(3*t) - cos(4*t));
    y_raw = escala * (16 * sin(t).^3);
    % Normalizar para que el primer punto sea el centro y avanzar hacia +X
    x_rel = (x_raw - x_raw(1));
    y_rel = (y_raw - y_raw(1));

    % Hitos cartesianos [x y z r p y] en marco BASE
    % El corazón se dibuja relativo a T_lienzo (el punto actual del robot)
    puntos_X = centro_base_xyz(1) + x_rel;
    puntos_Y = centro_base_xyz(2) + y_rel;
    puntos_Z = ones(N_puntos, 1) * centro_base_xyz(3);
    matriz_RPY = repmat(rpy_fijo, N_puntos, 1);
    Cart_puntos = [puntos_X, puntos_Y, puntos_Z, matriz_RPY];
    
    % Asegurar continuidad perfecta: el primer punto debe ser exactamente el punto actual
    % (x_rel(1) y y_rel(1) son 0 por construcción, así que Cart_puntos(1,:) ya es correcto)

    % Trayectoria suavizada en espacio cartesiano (en marco BASE)
    % Asegurar que el primer punto sea exactamente la pose actual para continuidad
    q_inicial = Cart_puntos(1,:);
    Cart_traj = mstraj(Cart_puntos, v_max, [], q_inicial, dt, t_blend);
    
    % Forzar el primer punto de Cart_traj a ser exactamente el punto inicial
    % (esto garantiza continuidad perfecta desde el movimiento anterior)
    Cart_traj(1,:) = q_inicial;

    % Mantener cámara actual; solo animar
    hold on; grid on;

    % IK + animación (convertir a marco MUNDO usando R.base)
    % Trazo de la trayectoria del TCP (sólo para arte latte)
    htrail = animatedline('Color',[0.8 0 0], 'LineWidth', 2); % rojo vino
    
    % Obtener la pose articular actual como semilla para IK (continuidad)
    try
        q_anterior = R.getpos()';
    catch
        % Si no se puede obtener, usar la pose calculada dinámicamente o cero
        if ~isempty(q_actual)
            q_anterior = q_actual;
        else
            q_anterior = zeros(6,1);
        end
    end
    for i = 1:size(Cart_traj,1)
        vec_cart = Cart_traj(i,:);
        T_base = transl(vec_cart(1:3)) * rpy2tr(vec_cart(4:6),'zyx');
        T_k = R.base.double * T_base;
        q_next = ik_barista(R, T_k, q_anterior, true);
        R.animate(q_next');
        % Agregar punto del TCP en mundo al trazo
        T_tcp = R.fkine(q_next');
        p = T_tcp.t;
        addpoints(htrail, p(1), p(2), p(3));
        q_anterior = q_next;
        drawnow;
    end

    title('Arte latte: trayectoria de corazón');
end


