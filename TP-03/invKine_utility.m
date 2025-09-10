clc, clear, close all

if exist('robot_v2.m', 'file') == 2
    run('robot_v2.m');
end
%% === Verificación de robots ===
assert(isa(R1,'SerialLink'), 'R1 debe ser un objeto SerialLink (RTB de Corke).');
assert(isa(R2,'SerialLink'), 'R2 debe ser un objeto SerialLink (RTB de Corke).');
assert(R1.mdh == 0, 'Se espera DH estándar (R1.mdh == 0).');

%% === Configuración articular de ejemplo ===
q1 = [0, 0, 0, 0, 0, 0];
q2 = [0, 0, 0, 0, 0, 0];
 

% sistemas1 = true(1, R1.n + 1);
% sistemas2 = true(1, R2.n + 1);
sistemas1 = [1, 0, 0, 0, 0, 0,0];
sistemas2 = [1, 0, 0, 0, 0, 0,0];

%% === Calcular marcos DH R1 ===
Tlist1 = cell(R1.n+1,1);
T = R1.base; Tlist1{1} = T;
for i = 1:R1.n
    Ai = R1.links(i).A(q1(i));
    T = T * Ai;
    Tlist1{i+1} = T;
end

%% === Calcular marcos DH R2 ===
Tlist2 = cell(R2.n+1,1);
T = R2.base; Tlist2{1} = T;
for i = 1:R2.n
    Ai = R2.links(i).A(q2(i));
    T = T * Ai;
    Tlist2{i+1} = T;
end



%% === Graficar robots en la misma figura ===
figure('Color','w'); 
axis(workspace);
axis equal


%% === Graficar marcos DH ===
Laxis = 0.2 * workspace(2);
for i = 0:R1.n
    if sistemas1(i+1)
        trplot(Tlist1{i+1}, 'frame', sprintf('{%d}_1',i), ...
               'length',Laxis,'rgb','arrow','width',0.5, ...
               'text_opts',{'FontSize',14});
    end
end
hold on;
for i = 0:R2.n
    if sistemas2(i+1)
        trplot(Tlist2{i+1}, 'frame', sprintf('{%d}_2',i), ...
               'length',Laxis,'rgb','arrow','width',0.5, ...
               'text_opts',{'FontSize',14});
    end
end
hold off;
R1.plot(q1, 'workspace', workspace, 'scale',0.8, 'jointdiam',1.4, 'notiles','nowrist');hold on;

R2.plot(q2, 'workspace', workspace, 'scale',0.8, 'jointdiam',1.4, 'notiles','nowrist');hold off;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Robots colaborativos');
grid on; 
R1.teach(q1);
