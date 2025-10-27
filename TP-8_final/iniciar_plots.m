function [fig, ax] = iniciar_plots(R, Tlists_cell, workspace_dims)
% INICIALIZAR_PLOTS - Crea y configura la figura para el robot.
%
% Entradas:
%   - R: El objeto SerialLink del robot.
%   - Tlists_cell: Un cell array donde cada celda contiene el Tlist de una trayectoria.
%   - workspace_dims: Las dimensiones del workspace para el plot.
%
% Salidas:
%   - fig: Handle de la figura creada.
%   - ax: Handle de los ejes (axes) del plot.
% 1) Crear y limpiar la figura
fig = figure(10);
clf;
ax = gca; % Obtener los ejes actuales

% 2) Graficar el robot en su postura inicial (usando la primera q de la primera trayectoria)
%   (Asumimos que la primera pose es la de reposo)
q0 = zeros(1, R.n); % Postura inicial en home, o puedes calcularla
R.plot(q0, 'workspace', workspace_dims, 'scale', 1, 'jointdiam', 1.4, 'nowrist', 'notiles');
trplot(R.base, 'frame', R.name, 'color', 'k', 'length', 0.5,'width',0.5,'rgb','arrow');

hold on; grid on;

% 3) Graficar el sistema de coordenadas base
trplot(R.base, 'frame', R.name, 'color', 'k', 'length', 0.5, 'width', 0.5, 'rgb', 'arrow');
title('Control de Trayectorias de Robot Barista');
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');

% 4) Graficar todos los waypoints de todas las trayectorias
colors = ['r', 'g', 'm', 'c']; % Colores para diferenciar trayectorias
for j = 1:length(Tlists_cell)
    Tlist = Tlists_cell{j};
    color = colors(mod(j-1, length(colors)) + 1);
    for k = 1:numel(Tlist)
        T = Tlist{k};
        pos = T(1:3,4); % Extraer posición [x; y; z]
        plot3(pos(1), pos(2), pos(3), 'o', 'MarkerSize', 8, 'MarkerFaceColor', color, 'MarkerEdgeColor', 'k');
        text(pos(1), pos(2), pos(3), sprintf('  T%d-P%d', j, k), 'FontSize', 10, 'Color', 'b');
    end
end

hold off;
end