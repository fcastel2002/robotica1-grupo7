% -------------------------------------------------------------------------
% PRUEBA DE SINGULARIDAD DE CODO (q3 = -pi/2)
%
% Propósito:
% 1. Mover el robot a una pose singular (codo extendido, q3 = -pi/2).
% 2. Definir un movimiento cartesiano lineal "hacia atrás" colineal.
% 3. Observar cuál solución (arriba/abajo) elige ik_barista.
% 4. Graficar y animar el resultado, mostrando error cartesiano.
% -------------------------------------------------------------------------

clear all;
close all;
clc;

% --- 1. Cargar la definición del Robot ---
disp('Cargando robot_v2.m...');
try
    run('robot_v2.m');
    disp('Robot R1 cargado.');
catch
    error('No se pudo ejecutar robot_v2.m. Asegúrate de que esté en el path.');
end

% --- 2. Definir la Pose Singular ---
q_sing = [0.2, 0.3, -1.343, 0.1, 0.2, 0]'; 
fprintf('Configuración singular (q_sing) (q3 = %.4f):\n', q_sing(3));
disp(q_sing');

% T_sing se calcula como matriz 4x4 (double)
T_sing = R1.fkine(q_sing); 
disp('Pose cartesiana en la singularidad (T_sing):');

% --- 3. Definir el Movimiento Lineal "Colineal" (Hacia Atrás) ---
% Movimiento de 5cm "hacia adentro" (o "atrás") en el eje Z del tool.
delta_mov_tool = transl(0, 0, -0.05); % 4x4 double

% Multiplicación de matrices (double * double)
T_target =  delta_mov_tool*T_sing.double; % T_target es 4x4 double

disp('T_target (T_sing movida 5cm hacia atrás en Z del TOOL) definida.');


% --- 4. Test A: Encontrar TODAS las soluciones ---
% ik_barista recibe T_target como 4x4 double
disp('--- Test A: Buscando TODAS las soluciones para T_target ---');
Q_todas = ik_barista(R1, T_target, q_sing, false, false);

if isempty(Q_todas)
    error('ik_barista no encontró NINGUNA solución. Revisa la pose T_target.');
end
fprintf('Se encontraron %d soluciones para T_target:\n', size(Q_todas, 2));
sol_arriba = Q_todas(:, Q_todas(3,:) > -pi/2);
sol_abajo  = Q_todas(:, Q_todas(3,:) < -pi/2);
disp('Solución(es) "Codo Arriba" (q3 > -pi/2):');
if isempty(sol_arriba), disp('  (Ninguna)'); else, disp(sol_arriba); end
disp('Solución(es) "Codo Abajo" (q3 < -pi/2):');
if isempty(sol_abajo), disp('  (Ninguna)'); else, disp(sol_abajo); end

% --- 5. Test B: Encontrar la "Mejor" Solución ---
% ik_barista recibe T_target como 4x4 double
disp('--- Test B: Buscando la "MEJOR" solución para T_target ---');
Q_mejor = ik_barista(R1, T_target, q_sing, true, false);
disp('Solución "Mejor" (Q_mejor) elegida:');
disp(Q_mejor);
if ~isempty(sol_arriba) && norm(Q_mejor - sol_arriba(:,1)) < 1e-3
    disp('>> Resultado: La IK eligió la solución "Codo Arriba".');
elseif ~isempty(sol_abajo) && norm(Q_mejor - sol_abajo(:,1)) < 1e-3
    disp('>> Resultado: La IK eligió la solución "Codo Abajo".');
else
    disp('>> Resultado: La IK eligió una solución inesperada.');
end

% --- 6. Graficar la Trayectoria de este movimiento ---
disp('Calculando trayectoria corta (Singular -> Target)...');
N_pasos = 30;

% [!] Conversión NECESARIA para ctraj: double -> SE3
T_sing_se3 = SE3(T_sing);
T_target_se3 = SE3(T_target);

% ctraj AHORA funciona, y T_traj es un vector de objetos SE3
T_traj = ctraj(T_sing_se3, T_target_se3, N_pasos);

Q_traj = zeros(6, N_pasos);
Q_traj(:, 1) = q_sing;
q_actual = q_sing;
ik_exito = true;
for i = 2:N_pasos
    
    % [!] LA CORRECCIÓN CLAVE ESTÁ AQUÍ
    % T_traj(i) es un objeto SE3.
    % Lo convertimos a 4x4 double antes de pasarlo a ik_barista
    T_step_double = T_traj(i).double;
    
    q_next = ik_barista(R1, T_step_double, q_actual, true, false);
    
    if isempty(q_next)
        Q_traj(:, i:end) = NaN; % Fallo
        ik_exito = false;
        fprintf('¡FALLO! ik_barista no encontró solución en el paso %d\n', i);
        break;
    end
    Q_traj(:, i) = q_next;
    q_actual = q_next; % q_actual siempre es la última solución válida
end
disp('Cálculo de trayectoria completado.');

% --- 7. Graficar Resultados Articulares ---
figure('Name', 'Prueba Moviendo desde Singularidad');
subplot(2,1,1);
plot(Q_traj');
title('Evolución de todas las articulaciones (Q)');
xlabel('Paso de la trayectoria'); ylabel('Ángulo (rad)');
legend('q1', 'q2', 'q3', 'q4', 'q5', 'q6'); grid on;
subplot(2,1,2);
plot(Q_traj(3, :), 'r', 'LineWidth', 2);
hold on;
plot(xlim, [q_sing(3) q_sing(3)], 'k--', 'LineWidth', 1);
title('Evolución de q3 (Codo)');
xlabel('Paso de la trayectoria'); ylabel('Ángulo q3 (rad)');
legend('q3', 'q3 = -pi/2 (Singularidad)'); grid on;
disp('Observa la gráfica de q3...');


% --- 8. Animar, Graficar Poses y Mostrar Error ---
disp('Presiona Enter en la consola para animar y ver el error cartesiano...');
pause;

try
    figure('Name', 'Animación y Error Cartesiano');
    ws = [-1.5 1.5 -1.5 1.5 -0.5 2]; % Workspace genérico
    R1.plot(Q_traj', 'workspace', ws, 'fps', 30, 'nowrist');
    
    hold on; % Para dibujar sobre la animación

    % T_final_real es 4x4 double (salida de fkine)
    T_final_real = R1.fkine(q_actual); 

    disp('Graficando pose objetivo (ROJO) vs. pose alcanzada (AZUL)...');
    
    % trplot espera 4x4 double (T_target ya lo es)
    trplot(T_target, 'frame', 'Target', 'color', 'r', 'length', 0.2, 'thick', 2);
    
    % trplot espera 4x4 double (T_final_real ya lo es)
    trplot(T_final_real, 'frame', 'Real', 'color', 'b', 'length', 0.2, 'thick', 2, 'linestyle', '--');

    % Cálculo de error con matrices 4x4 double
    T_error_mat = T_target.double * inv(T_final_real);
    
    err_pos = norm(T_error_mat(1:3, 4));
    [err_rot_angle, ~] = tr2angvec(T_error_mat(1:3, 1:3));
    
    fprintf('\n--- Error Cartesiano Final ---\n');
    if ~ik_exito
        fprintf('¡La IK falló antes de llegar al final!\n');
    end
    fprintf('Error de Posición (norma): %.4f mm\n', err_pos * 1000);
    fprintf('Error de Rotación (ángulo): %.4f grados\n', rad2deg(err_rot_angle));

    hold off;
    legend('show'); 

catch e
    fprintf('Error al animar: %s\n', e.message);
end