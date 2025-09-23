%=========================================================================%
%               Solución TP4 - Ejercicio TF: Espacio de Trabajo           %
%                    (Método de Trazado de Contornos)                     %
%=========================================================================%

function esp_trab_contorno()
    clc; clear; close all;

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
    plot(contorno_xz(:,1), contorno_xz(:,2), '.', 'Color', [0 0.4470 0.7410], 'MarkerSize', 5);
    title('Contorno del Espacio de Trabajo (Plano XZ)');
    xlabel('Alcance en X (m)'); ylabel('Alcance en Z (m)');
    grid on; axis equal;
    
    %% ===================================================================
    %               GRÁFICO 2: VISTA SUPERIOR (PLANO XY)               
    %=====================================================================%
    fprintf('Calculando contorno de la vista superior (Plano XY)...\n');
    
    % Hacemos girar la cintura (q1) en todo su rango
    q1_range = linspace(R1.qlim(1,1), R1.qlim(1,2), pasos*2);
    
    contorno_exterior_xy = [];
    contorno_interior_xy = [];
    
    % Buscamos las posturas de q2 y q3 para alcance máximo y mínimo
    % (Esto es una simplificación, pero funciona bien visualmente)
    q_max_alcance = [0, 0, 0, q_muneca]; % Brazo estirado
    q_min_alcance = [0, R1.qlim(2,2), R1.qlim(3,1), q_muneca]; % Brazo plegado
    
    for q1 = q1_range
        % Calcular punto exterior
        q_max_alcance(1) = q1;
        T_ext = R1.fkine(q_max_alcance);
        contorno_exterior_xy(end+1, :) = [T_ext.t(1), T_ext.t(2)];
        
        % Calcular punto interior
        q_min_alcance(1) = q1;
        T_int = R1.fkine(q_min_alcance);
        contorno_interior_xy(end+1, :) = [T_int.t(1), T_int.t(2)];
    end

    figure('Name', 'Workspace Contorno - Vista Superior (Plano XY)', 'NumberTitle', 'off');
    hold on;
    plot(contorno_exterior_xy(:,1), contorno_exterior_xy(:,2), 'r-');
    plot(contorno_interior_xy(:,1), contorno_interior_xy(:,2), 'b-');
    title('Contorno del Espacio de Trabajo (Plano XY)');
    xlabel('Alcance en X (m)'); ylabel('Alcance en Y (m)');
    legend('Contorno Exterior', 'Contorno Interior');
    grid on; axis equal;
    
    fprintf('Cálculos finalizados.\n');
end