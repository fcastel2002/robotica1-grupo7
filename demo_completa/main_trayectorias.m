%% EJECUTOR DE SECUENCIAS OPTIMIZADAS (TOPPRA)
clear all;
close all;
clc;
run('robot.m') % Carga los modelos R1 y R2

%% 1. Configuración de Ejecución
% ----------------------------------------------------
% --- ¡MODIFICA ESTAS 2 LÍNEAS! ---
robot_a_ejecutar = R1; % Elige el robot (R1 o R2)
try
    % Obtiene la ruta de la carpeta DONDE ESTÁ ESTE SCRIPT
    [script_dir, ~, ~] = fileparts(mfilename('fullpath'));
    directorio_base = script_dir; % Asume que está en la carpeta principal
catch
    directorio_base = pwd; % Fallback al directorio actual si falla mfilename
end

% Arma la ruta completa a la carpeta de trayectorias
directorio_toppra = fullfile(directorio_base, 'toppra_trajectories');
% ----------------------------------------------------

prefijo_robot = robot_a_ejecutar.name; % 'R1' o 'R2'

%% 2. Obtener, Ordenar y Cargar Posición Inicial
patron_busqueda = fullfile(directorio_toppra, [prefijo_robot, '_toppratraj_*.mat']);
lista_archivos = dir(patron_busqueda);

if isempty(lista_archivos)
    error('No se encontraron archivos en "%s" con el prefijo "%s"', directorio, prefijo_robot);
end

% --- Ordenar los archivos numéricamente ---
indices_inicio = zeros(numel(lista_archivos), 1);
for i = 1:numel(lista_archivos)
    try
        nums = sscanf(lista_archivos(i).name, [prefijo_robot, '_toppratraj_%d-%d.mat']);
        indices_inicio(i) = nums(1); % Guardamos solo el primer número (0)
    catch
        warning('No se pudo parsear el nombre: %s', lista_archivos(i).name);
        indices_inicio(i) = inf;
    end
end

[~, orden_correcto] = sort(indices_inicio);
lista_archivos_ordenada = lista_archivos(orden_correcto);

fprintf('Archivos encontrados y ordenados para %s.\n', prefijo_robot);

% --- Cargar la primera posición ---
try
    archivo_primero = fullfile(lista_archivos_ordenada(1).folder, lista_archivos_ordenada(1).name);
    datos_primero = load(archivo_primero);
    q_inicial_robot_activo = datos_primero.q_toppra(1, :); % Primera fila de la trayectoria
    fprintf('Posición inicial cargada desde: %s\n', lista_archivos_ordenada(1).name);
catch e
    warning('No se pudo cargar la posición inicial. Usando zeros. Error: %s', e.message);
    q_inicial_robot_activo = zeros(1, 6);
end

%% 3. Configuración de la Escena
figure(10); clf;

% Definir posiciones de ploteo iniciales
q_plot_R1 = zeros(1, 6)'; % Por defecto en cero
q_plot_R2 = zeros(1, 6)'; % Por defecto en cero

% Asignar la q_inicial al robot correcto
if strcmp(robot_a_ejecutar.name, 'R1')
    q_plot_R1 = q_inicial_robot_activo;
else
    q_plot_R2 = q_inicial_robot_activo;
end

% --- Dibujar robots en su estado inicial ---
if(exist('modelSTL','var') && modelSTL)
    % (Código de ploteo 3D si lo tienes, sino usa el plot normal)
    R1.plot3d(q_plot_R1', 'workspace', workspace, 'notiles', 'path', modelPath);
    hold on;
    R2.plot3d(q_plot_R2','workspace', workspace, 'notiles', 'path',modelPath2);
else
    % Plot 2D normal
    R1.plot(q_plot_R1','workspace', workspace, 'scale',1, 'jointdiam',1.4,'nowrist','notiles');
    hold on;
    R2.plot(q_plot_R2,'workspace', workspace, 'scale',1, 'jointdiam',1.4,'nowrist','notiles');
end
view(135, 25);
camtarget([0 0 0]);
grid on;
title('Ejecutando Secuencia Optimizada (Toppra)');
pause(1); % Pausa para que la figura cargue

%% 4. Ejecutar la Secuencia
for i = 1:numel(lista_archivos_ordenada)
    archivo = lista_archivos_ordenada(i);
    ruta_completa = fullfile(archivo.folder, archivo.name);
    
    fprintf('--- [%d/%d] Ejecutando: %s ---\n', i, numel(lista_archivos_ordenada), archivo.name);
    
    % Llamar a la función
    % El último argumento 'false' evita que se generen gráficos en CADA paso
    ejecutar_toppra(robot_a_ejecutar, ruta_completa, false);
    
    pause(0.2); % Breve pausa entre segmentos
end

fprintf('\n--- ¡Secuencia optimizada completada para %s! ---\n', prefijo_robot);

% (Opcional) Llama al último gráfico si lo deseas
disp('Mostrando gráficos del último segmento...');
ejecutar_toppra(robot_a_ejecutar, ruta_completa, true);