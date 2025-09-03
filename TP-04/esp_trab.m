%=========================================================================%
%               Solución TP4 - Ejercicio TF: Espacio de Trabajo           %
%                    (Método de Trazado de Contornos)                     %
%=========================================================================%

function esp_trab()
    close all;

    % 1. Cargar la definición del robot
    if exist('robot.m', 'file') == 2
        run('robot.m');
        fprintf('Archivo robot.m cargado.\n');
    else
        error('No se encontró el archivo robot.m.');
        return;
    end
    
    % --- Parámetros de la simulación ---
    pasos = 200; % Puntos a calcular por cada articulación. Aumentar para más definición.

    %% ===================================================================
    %               GRÁFICO 1: VISTA LATERAL (PLANO XZ)               
    %=====================================================================%
    fprintf('Calculando contorno de la vista lateral (Plano XZ)...\n');
    
    % Creamos vectores de ángulos para el hombro (q2) y el codo (q3)
    q2_range = linspace(R1.qlim(2,1), R1.qlim(2,2), pasos);
    q3_range = linspace(R1.qlim(3,1), R1.qlim(3,2), pasos);
    
    contorno_xz = [];
    
    % Dejamos la muñeca en una posición "recta" para maximizar el alcance
    q_muneca = [0, 0, 0]; 
    
    % Bucle anidado para probar todas las combinaciones de q2 y q3
    for q2 = q2_range
        for q3 = q3_range
            % Articulación 1 (cintura) en 0 para estar en el plano XZ
            q = [0, q2, q3, q_muneca];
            
            T = R1.fkine(q);
            contorno_xz(end+1, :) = [T.t(1), T.t(3)]; % Guardamos [x, z]
        end
    end
    
    figure('Name', 'Workspace Contorno - Vista Lateral (Plano XZ)', 'NumberTitle', 'off');
    k = boundary(contorno_xz(:,1), contorno_xz(:,2), 0.95);
    plot(contorno_xz(k,1), contorno_xz(k,2), '-', 'Color', [0 0.4470 0.7410], 'LineWidth', 1.5);
    title('Contorno del Espacio de Trabajo (Plano XZ)');
    xlabel('Alcance en X (m)'); ylabel('Alcance en Z (m)');
    grid on; axis equal;
    hold on;

    cota_color = [1 0 0]; % Rojo
    cota_style = '--';
      % 1. Cota para el alcance máximo en X
    [maxX, idxMaxX] = max(contorno_xz(:,1));
    z_en_maxX = contorno_xz(idxMaxX, 2);
    line([0, maxX], [0, 0], 'Color', cota_color, 'LineStyle', cota_style); % Línea sobre el eje X
    line([maxX, maxX], [0, z_en_maxX], 'Color', cota_color, 'LineStyle', cota_style); % Línea vertical
    texto_x = sprintf('Alcance X_{max} = %.2f m', maxX);
    text(maxX/2, -0.03, texto_x, 'Color', cota_color, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'top');

    % 2. Cota para la altura máxima en Z
    [maxZ, idxMaxZ] = max(contorno_xz(:,2));
    x_en_maxZ = contorno_xz(idxMaxZ, 1);
    line([-0.7, -0.7], [0, maxZ], 'Color', cota_color, 'LineStyle', cota_style); % Línea sobre el eje Z
    line([-0.7, x_en_maxZ], [maxZ, maxZ], 'Color', cota_color, 'LineStyle', cota_style); % Línea horizontal
    texto_z = sprintf('Altura Z_{max} = %.2f m', maxZ);
    text(-0.73, maxZ/2, texto_z, 'Color', cota_color, 'HorizontalAlignment', 'right', 'Rotation', 90);
    
    % --- FIN: CÓDIGO PARA ACOTACIONES ---
    
    hold off;
    %% ===================================================================
    %               GRÁFICO 2: VISTA SUPERIOR (PLANO XY)               
    %=====================================================================%
    fprintf('Calculando contorno de la vista superior (Plano XY)...\n');
    
    q1_range = linspace(R1.qlim(1,1), R1.qlim(1,2), pasos*2);
    contorno_exterior_xy = [];
    contorno_interior_xy = [];
    
    % Simplificación para alcance máximo y mínimo
    q_max_alcance = [0, 0, 0, q_muneca];
    q_min_alcance = [0, R1.qlim(2,2), R1.qlim(3,1), q_muneca];
    
    for q1 = q1_range
        q_max_alcance(1) = q1;
        T_ext = R1.fkine(q_max_alcance);
        contorno_interior_xy(end+1, :) = [T_ext.t(1), T_ext.t(2)];
        
        q_min_alcance(1) = q1;
        T_int = R1.fkine(q_min_alcance);
        contorno_exterior_xy(end+1, :) = [T_int.t(1), T_int.t(2)];
    end

    figure('Name', 'Workspace Contorno - Vista Superior (Plano XY)', 'NumberTitle', 'off');
    hold on;
    plot(contorno_exterior_xy(:,1), contorno_exterior_xy(:,2), 'r-');
    plot(contorno_interior_xy(:,1), contorno_interior_xy(:,2), 'b-');
    title('Contorno del Espacio de Trabajo (Plano XY)');
    xlabel('Alcance en X (m)'); ylabel('Alcance en Y (m)');
    legend('Contorno Exterior', 'Contorno Interior');
    grid on; axis equal;

    % --- INICIO: CÓDIGO PARA ACOTACIONES (PLANO XY) ---
    
    % 1. Cota para el radio exterior
    punto_ext = contorno_exterior_xy(50,:); % Tomamos el primer punto del contorno
    radio_ext = norm(punto_ext); % Calculamos la distancia al origen (radio)
    line([0, punto_ext(1)], [0, punto_ext(2)], 'Color', 'r', 'LineStyle', cota_style);
    texto_ext = sprintf('R_{ext} = %.2f m', radio_ext);
    text(0, punto_ext(1)/2, texto_ext, 'Color', 'r', 'BackgroundColor', 'w');

    % 2. Cota para el radio interior
    punto_int = contorno_interior_xy(1,:); % Tomamos el primer punto del contorno
    radio_int = norm(punto_int); % Calculamos la distancia al origen (radio)
    line([0, punto_int(1)], [0, punto_int(2)], 'Color', 'b', 'LineStyle', cota_style);
    texto_int = sprintf('R_{int} = %.2f m', radio_int);
    text(punto_int(1)/2, 0.1, texto_int, 'Color', 'b', 'BackgroundColor', 'w');
    
    % --- FIN: CÓDIGO PARA ACOTACIONES ---
    
    hold off;
    
    fprintf('Cálculos finalizados.\n');
end