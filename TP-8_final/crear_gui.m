function crear_gui(R, plist, Tlist, qseq, n, N)
% Crea botones en la figura activa para ejecutar:
%   - Todo el recorrido
%   - Un segmento por vez (k-1 -> k)
% Usa sublistas para no alterar lógica interna de 'ejecutar_trayectorias'.

if nargin < 5, n = 30; end
if nargin < 6, N = 20; end

fig = gcf;

% Layout derecho simple
x = 0.85; w = 0.12; h = 0.06; dy = 0.08; y0 = 0.80;

% Botón "Todo"
uicontrol('Parent',fig,'Style','pushbutton','Units','normalized', ...
    'Position',[x y0 w h], 'String','Ejecutar TODO', ...
    'FontSize',11, ...
    'Callback', @(~,~) ejecutar_trayectorias(R, plist, Tlist, qseq, n, N));

% Botones por segmento
y = y0 - dy;
for k = 2:numel(plist)
    txt = sprintf('Punto %d → %d', k-1, k);
    uicontrol('Parent',fig,'Style','pushbutton','Units','normalized', ...
        'Position',[x y w h], 'String',txt, 'FontSize',10, ...
        'Callback', @(~,~) ejecutar_tramo(k));
    y = y - dy;
end

    function ejecutar_tramo(k)
        % Subconjuntos consistentes para mantener la lógica intacta
        plist_sub = plist(k-1:k);
        Tlist_sub = Tlist(k-1:k);
        qseq_sub  = qseq(:,k-1:k);

        ejecutar_trayectorias(R, plist_sub, Tlist_sub, qseq_sub, n, N);
    end
end
