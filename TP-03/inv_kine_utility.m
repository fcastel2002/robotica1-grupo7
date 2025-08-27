if exist('robot.m', 'file') == 2
    run('robot.m');
end
if ~exist('R','var')
    % Caso 2: robot.m es función que retorna R
    try
        R = robot();  
    catch
        error(['No se encontró el objeto R. Asegurate de que robot.m ' ...
               'exista y defina (o retorne) un SerialLink llamado R.']);
    end
end
assert(isa(R,'SerialLink'), 'R debe ser un objeto SerialLink (RTB de Corke).');
assert(R.mdh == 0, 'Se espera DH estándar (R.mdh == 0).');

q = [pi/2, -pi/4, 0, pi/3, pi, 0];

sistemas = true(1, R.n + 1);
assert(numel(sistemas) == R.n + 1, 'La longitud de "sistemas" debe ser R.n+1 (marcos {0..n}).');

R.teach(q);
Tlist = cell(R.n + 1, 1);
R.base = trotz(deg2rad(30)) * transl(0.30, -0.20, 0.10);
R.tool = trotx(pi/2) * transl(0, 0, 0.12);
T = R.base;           % marco {0}
Tlist{1} = T;
figure('Color','w'); 
R.plot(q, ...
    'workspace', workspace, ...
    'scale',     1, ...     % reduce tamaño de elementos gráficos del robot
    'jointdiam', 1.4, ...     % reduce el diámetro de las juntas
    'notiles');               % elimina las baldosas del suelo
camzoom(1.5);
%view(135, 25); axis equal; grid on;
 hold on;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Robot y marcos DH seleccionados');
for i = 1:R.n
    % Transformación homogénea del eslabón i (i-1 -> i) con DH estándar
    Ai = R.links(i).A(q(i));
    T  = T * Ai;
    Tlist{i+1} = T;   % marco {i}
end

% Longitud de los ejes dibujados para trplot (en función del alcance estimado)
Laxis = 0.9 * workspace(2);

% Graficar marcos seleccionados
for i = 0:R.n
    if sistemas(i+1)
        trplot(Tlist{i+1}, ...
            'frame', sprintf('{%d}', i), ...  % etiqueta del marco
            'length', Laxis, ...              % longitud de ejes
            'rgb', ...                        % ejes X,Y,Z en colores estándar
            'arrow', ...
            'width', 0.5, ...
            'text_opts', {'FontSize', 16});                         % puntas de flecha en los ejes
    end
end
%========= Tool ================%
% T_tool = Tlist{end} * R.tool;
% trplot(T_tool, 'frame', '{tool}', 'length', 0.2*workspace(2), 'rgb', 'arrow');

hold off;