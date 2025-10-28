clear all;
close all;
clc;
run ('robot_v2.m')
%% OPCIONES DE DEBUG
modelSTL = true;

%%

%% 0) Parámetros de animación
TF = 0.5;     % [s] por segmento
n  = 30;      % muestras por segmento (suavidad)
N  = 40;      % puntos de interpolación cartesiana

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
    struct('pose',[0.499   0  0.6     0  pi/1.5         0],'tipo',''),
    struct('pose',[0.05,  -0.45, 0.3     0   pi/2      -pi/2],'tipo','articular'),
    struct('pose',[0.4   -0.45  0.33     0  pi/2   -pi/2],'tipo','cartesiana'),
    struct('pose',[0.4   -0.45  0.4     0  pi/2    -pi/2],'tipo','articular'),
    struct('pose',[0.4   -0.45  0.33     0  pi/2   -pi/2],'tipo','articular'),
    % salimos de la singularidad bajando un poco el pitch
    struct('pose', [0.499, 0,     0.55,  0, pi/2-0.01, 0], 'tipo', 'cartesiana'),
    % sirve la leche
    struct('pose', [0.499, 0,     0.55,  0.01, pi/2+0.5, 0], 'tipo', 'cartesiana'),
    % falta el arte latte pero buscaremos una herramienta de generacion de
    % trayectorias mas visual y prácica.
};
    
    %% ROBOT CAFETERO R2
plist_R2 = {
    struct('pose',[0.499   0  0.6     0  pi/1.5         0],'tipo',''),
    struct('pose',[0.05,  0.45-0.15 0.5     0   -pi/2      -pi/2],'tipo','articular'),
    struct('pose',[0.45   0.45-0.15  0.33     0  -pi/2   -pi/2],'tipo','cartesiana'),
    struct('pose',[0.45   0.45-0.15  0.4     0  -pi/2    -pi/2],'tipo','articular'),
    % encastre cafe en cafetera
    struct('pose',[0.45   0.45-0.15  0.4     0  -pi/2    -pi/2-0.8],'tipo','cartesiana'),
    %--
    % espera al cafe
    struct('pose',[0.45-0.1   0.45-0.15-0.1  0.33     0  -pi/2   -pi/2-0.8],'tipo','articular'),
    % vuelve a sacarlo
    struct('pose',[0.45   0.45-0.15  0.33     0  -pi/2    -pi/2-0.8],'tipo','cartesiana'),

    % salimos de la singularidad bajando un poco el roll en este caso
    struct('pose', [0.499, -0.15,     0.30,  0.01, -pi/2, pi], 'tipo', 'cartesiana'),
    struct('pose', [0.499, -0.15,     0.45,  0.01, -pi/2-0.5, pi], 'tipo', 'cartesiana'),
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
trplot(R1.base, 'frame', R1.name, 'color', 'k', 'length', 0.5,'width',0.5,'rgb','arrow');
trplot(R2.base, 'frame', R2.name, 'color', 'k', 'length', 0.5,'width',0.5,'rgb','arrow');

title('Trayectoria automática (1 segundo entre puntos)');

% Graficar puntos de Tlist
for k = 1:numel(Tlist_R1)
    T = Tlist_R1{k};
    pos = T(1:3,4);
    plot3(pos(1), pos(2), pos(3), 'ro', 'MarkerSize', 3, 'MarkerFaceColor', 'r');
    text(pos(1), pos(2), pos(3), sprintf('  P%d', k), 'FontSize', 10, 'Color', 'b');
end
for k = 1:numel(Tlist_R2)
    T = Tlist_R2{k};
    pos = T(1:3,4);
    plot3(pos(1), pos(2), pos(3), 'ro', 'MarkerSize', 3, 'MarkerFaceColor', 'g');
    text(pos(1), pos(2), pos(3), sprintf('  P%d', k), 'FontSize', 10, 'Color', 'b');
end
crear_gui([R1 R2], {plist_R1 plist_R2}, {Tlist_R1 Tlist_R2}, {qseq_R1 qseq_R2}, n, N);