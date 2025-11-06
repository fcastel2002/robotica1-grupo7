function [Q_out, t_out] = arte_latte_corazon(R, centro_base_xyz, rpy_fijo)
% ARTE_LATTE_CORAZON Genera y anima una trayectoria de corazón en el marco BASE de R.
%
% Salidas:
%   [Q_out, t_out] = Perfiles de posición articular (Nx6) y tiempo (Nx1)
%
% Argumentos:
%   R: SerialLink del robot (R1 leche)
%   centro_base_xyz: [x y z] en marco BASE del robot
%   rpy_fijo: [r p y] en rad, orientación constante del efector
%   animar: (opcional) true/false. Por defecto true.

    if nargin < 3
        error('Faltan argumentos: arte_latte_corazon(R, [x y z], [r p y])');
    end
   global animar;


    % Parámetros de forma/velocidad
    N_puntos = 50;       % hitos de forma
    escala   = 0.003;    % ~6 cm de tamaño
    dt       = 0.1;     % 50 ms
    v_max    = 0.50;     % m/s y rad/s (equivalente)
    t_blend  = 0.10;     % s

    % Corazón paramétrico 2D en marco BASE (XY), comenzando en el centro
    t_param = linspace(pi, -pi, N_puntos)';
    x_raw = escala * (13*cos(t_param) - 5*cos(2*t_param) - 2*cos(3*t_param) - cos(4*t_param));
    y_raw = escala * (16 * sin(t_param).^3);
    % Normalizar para que el primer punto sea el centro y avanzar hacia +X
    x_rel = (x_raw - x_raw(1));
    y_rel = (y_raw - y_raw(1));

    % Hitos cartesianos [x y z r p y] en marco BASE
    puntos_X = centro_base_xyz(1) + x_rel;
    puntos_Y = centro_base_xyz(2) + y_rel;
    puntos_Z = ones(N_puntos, 1) * centro_base_xyz(3);
    matriz_RPY = repmat(rpy_fijo, N_puntos, 1);
    Cart_puntos = [puntos_X, puntos_Y, puntos_Z, matriz_RPY];

    % Trayectoria suavizada en espacio cartesiano (en marco BASE)
    q_inicial = Cart_puntos(1,:);
    Cart_traj = mstraj(Cart_puntos, v_max, [], q_inicial, dt, t_blend);

    % --- Preparar Salidas ---
    num_pasos = size(Cart_traj,1);
    Q_out = zeros(num_pasos, R.n);
    t_out = (0:num_pasos-1)' * dt;

    if animar
        % Mantener cámara actual; solo animar
        hold on; grid on;
        % Trazo de la trayectoria del TCP (sólo para arte latte)
        htrail = animatedline('Color',[0.8 0 0], 'LineWidth', 2); % rojo vino
        title('Arte latte: trayectoria de corazón');
    end

    try
        q_anterior = R.getpos()';
    catch
        q_anterior = zeros(6,1);
    end
    
    % --- Bucle de IK y Animación ---
    for i = 1:num_pasos
        vec_cart = Cart_traj(i,:);
        T_base = transl(vec_cart(1:3)) * rpy2tr(vec_cart(4:6),'zyx');
        T_k = R.base.double * T_base;
        q_next = ik_barista(R, T_k, q_anterior, true);
        
        % --- Almacenar Salida ---
        Q_out(i,:) = q_next';

        if animar
            R.animate(q_next');
            % Agregar punto del TCP en mundo al trazo
            T_tcp = R.fkine(q_next');
            p = T_tcp.t;
            addpoints(htrail, p(1), p(2), p(3));
            drawnow;
        end
        q_anterior = q_next;
    end
end