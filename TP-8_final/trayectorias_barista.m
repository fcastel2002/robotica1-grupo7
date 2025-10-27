clear; clc;
run ('robot_v2.m')

%% 0) Parámetros de animación
tf = 0.5;     % [s] por segmento
n  = 30;      % muestras por segmento (suavidad)
N  = 20;      % puntos de interpolación cartesiana
R = R1;

%% 1) Lista de puntos cartesianos (con orientaciones)
plist = {
    struct('pose',[0.499   0  0.6     0  pi/1.5         0],'tipo',''), 
    struct('pose',[0.05,  -0.45, 0.3     0   pi/2      -pi/2],'tipo','articular'),
    struct('pose',[0.3   -0.45  0.33     0  pi/2   -pi/2],'tipo','cartesiano'),
    struct('pose',[0.3   -0.45  0.4     0  pi/2    -pi/2],'tipo','articular'),
    struct('pose',[0.3   -0.45  0.33     0  pi/2   -pi/2],'tipo','articular'),
    struct('pose',[0.499   0  0.45     0  pi/2         0],'tipo','articular'), 
};

plist_poses = cellfun(@(s) s.pose, plist, 'UniformOutput', false);

%% transformamos a SE3 con transl y rpy2tr (ROLL-PITCH-YAW to SE(3)
for i=1:length(plist)
    Tlist{i} = transl(plist_poses{i}(1:3)) * rpy2tr(plist_poses{i}(4:6),'zyx');
    Tlist{i} = R.base.double * Tlist{i};
end

%% 2) Resolver IK secuencialmente (usa la solución anterior como semilla)
qseq = zeros(6, numel(Tlist));
q_curr = zeros(6,1);    % postura inicial (ajustar si se desea otra q)

for k = 1:numel(Tlist)
    qq_all = ik_barista(R, Tlist{k}, q_curr, false);
    qk = filtrar_q_lim(R, qq_all, q_curr, true);
    qseq(:,k) = qk;
    q_curr = qk;
end

%% 3) Inicializar visualización
figure(10); clf;
R.plot(qseq(:,1)','workspace', workspace, 'scale',1, 'jointdiam',1.4,'nowrist','notiles'); 
hold on; grid on;
trplot(R.base, 'frame', R.name, 'color', 'k', 'length', 0.5,'width',0.5,'rgb','arrow');
title('Trayectoria automática (1 segundo entre puntos)');

% Graficar puntos de Tlist
for k = 1:numel(Tlist)
    T = Tlist{k};
    pos = T(1:3,4);
    plot3(pos(1), pos(2), pos(3), 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'r');
    text(pos(1), pos(2), pos(3), sprintf('  P%d', k), 'FontSize', 10, 'Color', 'b');
end

crear_gui(R, plist, Tlist, qseq, n, N);