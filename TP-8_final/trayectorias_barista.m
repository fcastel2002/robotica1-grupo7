clear all;
close all;
clc;
run ('robot_v2.m')
%% OPCIONES DE DEBUG
modelSTL = false;

%%

%% 0) Parámetros de animación
TF = 0.3;     % [s] por segmento
n  = 10;      % muestras por segmento (suavidad)
N  = 20;      % puntos de interpolación cartesiana6

%% ACLARACIONES
% Para tener en cuenta, la mesa donde se encuentra la cafetera, moledora de
% cafe etc estan en las cordenadas NEGATIVAS Y del robot 1 y en las
% coordenadas POSITIVAS Y del robot 2, ambos robots se dirigen el uno al
% otro mediante coordenadas X POSITIVAS es decir que desde el robot 2 hasta
% el robot 1 esta en la direcio +X y viceversa desde el robot 1 hasta el 2
% también estan en +X del robot 2.
% ===
% Además por simplicidad las coordenadas se dan tomando en cuenta el cero
% de cada robot, por eso es que luego se multiplican por R.base
% correspondiente.

%% TRAYECTORIAS
    %% ROBOT LECHE R1 
plist_R1 = {
    %   0. Reposo
    struct('pose',[0.490+0.09    0           0.6     0   pi/1.5   0],'tipo',''),
    %   1. Agarra Jarra de con leche
    struct('pose',[0.05,    -0.6,      0.3     0   pi/2    -pi/2],'tipo','articular'),
    %   2. Lleva jarra a lanceta
    struct('pose',[0.4      -0.6       0.33    0   pi/2    -pi/2],'tipo','cartesiana'),
    %   3. Sube jarra introduciendo lanceta dentro de la misma
    struct('pose',[0.4      -0.6       0.4     0   pi/2    -pi/2],'tipo','articular'),
    %   4. Baja la jarra para sacarla de la lanceta
    struct('pose',[0.4      -0.6      0.33    0   pi/2    -pi/2],'tipo','articular'),
    %   5. Lleva jarra sobre taza con cafe
    struct('pose', [0.490+0.09,      0,     0.55,  0, pi/2-0.01, 0], 'tipo', 'cartesiana'),
    %   6. Vuelca la leche sobre el cafe
    struct('pose', [0.490+0.09, 0,     0.55,  0.01, pi/2+0.5, 0], 'tipo', 'cartesiana'),
};
    
    %% ROBOT CAFETERO R2
    % Se tienen los siguientes supuestos:
    %   1. Portafiltro se encuentra en molino con cafe molido y pisado en el mismo
    %   2. Se encuentra una Taza esperando el cafe con mango hacia fuera de la maquina cafetera
plist_R2 = {
    %   0. Reposo
    struct('pose',[0.490+0.09   0    0.6     0   pi/1.5         0],'tipo',''),
    %   1. Recoge portafiltro (de maquina molino de cafe)
    struct('pose',[-0.2,  0.6   0.3     0   -pi/2      -pi/2],'tipo','articular'),
    %       1.2. Trayectoria
    struct('pose',[-0.2,  0.5  0.3     0   -pi/2      -pi/2],'tipo','cartesiana'),
    %       1.3. Trayectoria
    struct('pose',[0.25,  0.5  0.4     0   -pi/2      -pi/2],'tipo','cartesiana'),
    %   2. Lleva portafiltro a cafetera 
    struct('pose',[0.25   0.6   0.4     0  -pi/2   -pi/2],'tipo','cartesiana'),
    %   3. Introduce Portafiltro a Cafetera (sube 7cm)
    struct('pose',[0.25   0.6   0.45    0  -pi/2    -pi/2],'tipo','cartesiana'),
    %   4. Encastra Portafiltro (gira 30°)
    struct('pose',[0.25   0.6   0.45    0  -pi/2    -pi/2-pi/6],'tipo','cartesiana'),
    %       4.1. Trayectoria
    struct('pose',[0.25   0.5   0.475     0  -pi/2    -pi/2],'tipo','articular'),
    %   5. Presiona boton (boton para producir el expresso)
    struct('pose',[0.25   0.6   0.5     0  -pi/2    -pi/2],'tipo','articular'),
    %       5.1. Trayectoria
    struct('pose',[0.25   0.6   0.45     0  -pi/2    -pi/2],'tipo','articular'),
    %   6. Retira la taza con cafe 
    struct('pose',[0.25   0.6   0.35    0  -pi/2    -pi/2],'tipo','articular'),
    %   7. Pre-Entrega (Orientar taza nivelada)
    struct('pose', [0.4, -0.15, 0.35, 0.001, -pi/2, pi], 'tipo', 'cartesiana'),
    %   8. Punto de Entrega (Handover)
    struct('pose', [0.4, -0.15, 0.35, 0.001, -pi/2, pi], 'tipo', 'cartesiana'), 
};

plist_R1_poses = cellfun(@(s) s.pose, plist_R1, 'UniformOutput', false);
plist_R2_poses = cellfun(@(s) s.pose, plist_R2, 'UniformOutput', false);

%% transformamos a SE3 con transl y rpy2tr (ROLL-PITCH-YAW to SE(3)
for i=1:length(plist_R1)
    Tlist_R1{i} = transl(plist_R1_poses{i}(1:3)) * rpy2tr(plist_R1_poses{i}(4:6),'zyx');
    Tlist_R1{i} = R1.base.double * Tlist_R1{i};
end
for i=1:length(plist_R2)
    Tlist_R2{i} = transl(plist_R2_poses{i}(1:3)) * rpy2tr(plist_R2_poses{i}(4:6),'zyx');
    Tlist_R2{i} = R2.base.double * Tlist_R2{i};
end

%% 2) Resolver IK secuencialmente (usa la solución anterior como semilla)
qseq_R1 = zeros(6, numel(Tlist_R1));
qseq_R2 = zeros(6, numel(Tlist_R2));

q_curr_R1 = zeros(6,1);    % postura inicial (ajustar si se desea otra q)
q_curr_R2 = zeros(6,1);

for k = 1:numel(Tlist_R1)
    qk = ik_barista(R1, Tlist_R1{k}, q_curr_R1, true);
    qseq_R1(:,k) = qk;
    q_curr_R1 = qk;
end
for k = 1:numel(Tlist_R2)
    qk = ik_barista(R2, Tlist_R2{k}, q_curr_R2, true);
    qseq_R2(:,k) = qk;
    q_curr_R2 = qk;
end
%% 3) Inicializar visualización
figure(10); clf;

if(modelSTL)
    R1.plot3d(qseq_R1(:,1)', ...
        'workspace', workspace, ...
        'notiles', ...
        'path', modelPath);
    hold on;
    R2.plot3d(qseq_R2(:,1)','workspace', workspace, 'notiles', 'path',modelPath);
    view(135, 25);
    camtarget([0 0 0]);
    camva(8); % ángulo de vista razonable
    grid on;
else
    R1.plot(qseq_R1(:,1)','workspace', workspace, 'scale',1, 'jointdiam',1.4,'nowrist','notiles');
    hold on;
    R2.plot(qseq_R1(:,1)','workspace', workspace, 'scale',1, 'jointdiam',1.4,'nowrist','notiles');
end
%trplot(R1.base, 'frame', R1.name, 'color', 'k', 'length', 0.5,'width',0.5,'rgb','arrow');
%trplot(R2.base, 'frame', R2.name, 'color', 'k', 'length', 0.5,'width',0.5,'rgb','arrow');

title('Robots Cooperativos Baristas');

% Graficar puntos de Tlist
for k = 1:numel(Tlist_R1)
    T = Tlist_R1{k};
    pos = T(1:3,4);
    plot3(pos(1), pos(2), pos(3), 'ro', 'MarkerSize', 3, 'MarkerFaceColor', 'r');
    %text(pos(1), pos(2), pos(3), sprintf('  P%d', k), 'FontSize', 10, 'Color', 'b');
end
for k = 1:numel(Tlist_R2)
    T = Tlist_R2{k};
    pos = T(1:3,4);
    plot3(pos(1), pos(2), pos(3), 'ro', 'MarkerSize', 3, 'MarkerFaceColor', 'g');
    %text(pos(1), pos(2), pos(3), sprintf('  P%d', k), 'FontSize', 10, 'Color', 'b');
end
% Definir grupos de trayectos (índices en plist) por pasos principales
groups_R1 = { [1 2], [2 3], [3 4], [4 5], [5 6], [6 7] };
groups_R2 = { [1 2], [2 3 4 5], [5 6], [6 7 8], [7 8 9], [9 10 11], [11 12], [12 13] };

crear_gui([R1 R2], {plist_R1 plist_R2}, {Tlist_R1 Tlist_R2}, {qseq_R1 qseq_R2}, n, N, {groups_R1, groups_R2});

% Botón adicional: Arte latte (corazón) para R1
fig = gcf;
x = 0.85; w = 0.12; h = 0.06; y = 0.72; % debajo del botón "Ejecutar TODO"
uicontrol('Parent',fig,'Style','pushbutton','Units','normalized', ...
    'Position',[x y w h], 'String','Arte latte (corazón)', ...
    'FontSize',11, ...
    'Callback', @(~,~) arte_latte_corazon(R1, [0.499 0 0.55], [0.01 pi/2+0.5 0]));

% Botón de Ayuda (descripción de puntos de cada robot)
uicontrol('Parent',fig,'Style','pushbutton','Units','normalized', ...
    'Position',[0.44 0.05 0.12 0.06], 'String','Ayuda', ...
    'FontSize',11, ...
    'Callback', @(~,~) mostrar_ayuda());

function mostrar_ayuda()
    ayuda = sprintf([ ...
        'R1 (Leche):\n', ...
        '  0: Reposo\n', ...
        '  1: Agarra jarra con leche\n', ...
        '  2: Lleva jarra a lanceta (cartesiana)\n', ...
        '  3: Sube jarra (introduce lanceta)\n', ...
        '  4: Baja jarra (saca lanceta)\n', ...
        '  5: Lleva jarra sobre taza con café (cartesiana)\n', ...
        '  6: Vuelca leche sobre el café (cartesiana)\n', ...
        '  + Arte latte: trayectoria de corazón\n\n', ...
        'R2 (Cafetero):\n', ...
        '  0: Reposo\n', ...
        '  1: Recoge portafiltro (desde molino)\n', ...
        '  1.2: Trayectoria intermedia (cartesiana)\n', ...
        '  1.3: Trayectoria intermedia (cartesiana)\n', ...
        '  2: Lleva portafiltro a cafetera (cartesiana)\n', ...
        '  3: Introduce portafiltro (sube 7 cm)\n', ...
        '  4: Encastra portafiltro (gira 30°)\n', ...
        '  4.1: Trayectoria intermedia\n', ...
        '  5: Presiona botón de espresso\n', ...
        '  5.1: Trayectoria intermedia\n', ...
        '  6: Retira la taza con café\n', ...
        '  7: Pre-entrega (taza nivelada)\n', ...
        '  8: Punto de entrega (handover)\n' ...
    ]);
    msgbox(ayuda, 'Ayuda - Secuencia de tareas');
end