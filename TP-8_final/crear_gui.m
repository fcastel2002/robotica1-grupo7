function crear_gui(R, plist, Tlist, qseq, n, N, groups)
% Crea botones en la figura activa para ejecutar:
%   - Todo el recorrido
%   - Un segmento por vez (k-1 -> k)
% Usa sublistas para no alterar lógica interna de 'ejecutar_trayectorias'.

if nargin < 5, n = 30; end
if nargin < 6, N = 20; end
if nargin < 7, groups = []; end

fig = gcf;
 
% Layout derecho simple robot 1
x = 0.85; w = 0.12; h = 0.06; dy = 0.08; y0 = 0.80;

% Botón "Todo" R1
uicontrol('Parent',fig,'Style','pushbutton','Units','normalized', ...
    'Position',[x y0 w h], 'String','Ejecutar TODO R1', ...
    'FontSize',11, ...
    'Callback', @(~,~) ejecutar_todo_r1(R(1), plist{1}, Tlist{1}, qseq{1}, n, N));

% Botones agrupados o por segmento
y = y0 - dy;
if ~isempty(groups)
    % groups{1}: celdas con índices de plist{1}
    for g = 1:numel(groups{1})
        idxs = groups{1}{g};
        etiqueta = sprintf('%d -> %d', g-1, g);
        uicontrol('Parent',fig,'Style','pushbutton','Units','normalized', ...
            'Position',[x y w h], 'String',etiqueta, 'FontSize',10, ...
            'Callback', @(~,~) ejecutar_grupo(R(1), plist{1}, Tlist{1}, qseq{1}, idxs));
        y = y - dy;
    end
else
    for k = 2:numel(plist{1})
        txt = sprintf('Punto %d → %d', k-1, k);
        uicontrol('Parent',fig,'Style','pushbutton','Units','normalized', ...
            'Position',[x y w h], 'String',txt, 'FontSize',10, ...
            'Callback', @(~,~) ejecutar_tramo(k,R(1),plist{1},Tlist{1},qseq{1}));
        y = y - dy;
    end
end

% Layout izquierdo simple robot 2
x = 0.03; w = 0.12; h = 0.06; dy = 0.08; y0 = 0.80;

% Botón "Todo"
uicontrol('Parent',fig,'Style','pushbutton','Units','normalized', ...
    'Position',[x y0 w h], 'String','Ejecutar TODO R2', ...
    'FontSize',11, ...
    'Callback', @(~,~) ejecutar_trayectorias(R(2), plist{2}, Tlist{2}, qseq{2}, n, N));

% Botones agrupados o por segmento para R2
y = y0 - dy;
if ~isempty(groups)
    for g = 1:numel(groups{2})
        idxs = groups{2}{g};
        etiqueta = sprintf('R2 %d -> %d', g-1, g);
        uicontrol('Parent',fig,'Style','pushbutton','Units','normalized', ...
            'Position',[x y w h], 'String',etiqueta, 'FontSize',10, ...
            'Callback', @(~,~) ejecutar_grupo(R(2), plist{2}, Tlist{2}, qseq{2}, idxs));
        y = y - dy;
    end
else
    for k = 2:numel(plist{2})
        txt = sprintf('Punto R2 %d → %d', k-1, k);
        uicontrol('Parent',fig,'Style','pushbutton','Units','normalized', ...
            'Position',[x y w h], 'String',txt, 'FontSize',10, ...
            'Callback', @(~,~) ejecutar_tramo(k,R(2),plist{2},Tlist{2},qseq{2}));
        y = y - dy;
    end
end


%% helpers
    function ejecutar_tramo(k,R,plist,Tlist,qseq)
        % Subconjuntos consistentes para mantener la lógica intacta
        plist_sub = plist(k-1:k);
        Tlist_sub = Tlist(k-1:k);
        qseq_sub  = qseq(:,k-1:k);

        ejecutar_trayectorias(R, plist_sub, Tlist_sub, qseq_sub, n, N);
    end
    function ejecutar_todo_r1(R, plist, Tlist, qseq, n, N)
        % Ejecuta todo R1 y luego el arte latte como final
        ejecutar_trayectorias(R, plist, Tlist, qseq, n, N);
        try
            arte_latte_corazon(R, [0.499 0 0.55], [0.01 pi/2+0.5 0]);
        catch ME
            warning('Fallo al ejecutar arte latte: %s', ME.message);
        end
    end
    function ejecutar_grupo(R, plist, Tlist, qseq, idxs)
        % Ejecuta una lista de índices consecutivos como un bloque
        plist_sub = plist(idxs);
        Tlist_sub = Tlist(idxs);
        qseq_sub  = qseq(:, idxs);
        ejecutar_trayectorias(R, plist_sub, Tlist_sub, qseq_sub, n, N);
    end
    

end
