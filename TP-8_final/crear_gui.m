function crear_gui(R, plist, Tlist, qseq, n, N)
% Crea botones en la figura activa para ejecutar:
%   - Todo el recorrido
%   - Un segmento por vez (k-1 -> k)
% Usa sublistas para no alterar lógica interna de 'ejecutar_trayectorias'.

if nargin < 5, n = 30; end
if nargin < 6, N = 20; end

fig = gcf;
 
% Layout derecho simple robot 1
x = 0.85; w = 0.12; h = 0.06; dy = 0.08; y0 = 0.80;

% Botón "Todo"
uicontrol('Parent',fig,'Style','pushbutton','Units','normalized', ...
    'Position',[x y0 w h], 'String','Ejecutar TODO', ...
    'FontSize',11, ...
    'Callback', @(~,~) ejecutar_trayectorias(R(1), plist{1}, Tlist{1}, qseq{1}, n, N));

% Botones por segmento
y = y0 - dy;
for k = 2:numel(plist{1})
    txt = sprintf('Punto %d → %d', k-1, k);
    
    uicontrol('Parent',fig,'Style','pushbutton','Units','normalized', ...
        'Position',[x y w h], 'String',txt, 'FontSize',10, ...
        'Callback', @(~,~) ejecutar_tramo(k,R(1),plist{1},Tlist{1},qseq{1}));
    y = y - dy;
end

% Layout izquierdo simple robot 2
x = 0.03; w = 0.12; h = 0.06; dy = 0.08; y0 = 0.80;

% Botón "Todo"
uicontrol('Parent',fig,'Style','pushbutton','Units','normalized', ...
    'Position',[x y0 w h], 'String','Ejecutar TODO R2', ...
    'FontSize',11, ...
    'Callback', @(~,~) ejecutar_trayectorias(R(2), plist{2}, Tlist{2}, qseq{2}, n, N));

% Botones por segmento
y = y0 - dy;
for k = 2:numel(plist{2})
    txt = sprintf('Punto R2 %d → %d', k-1, k);
    uicontrol('Parent',fig,'Style','pushbutton','Units','normalized', ...
        'Position',[x y w h], 'String',txt, 'FontSize',10, ...
        'Callback', @(~,~) ejecutar_tramo(k,R(2),plist{2},Tlist{2},qseq{2}));
    y = y - dy;
end


%% helpers
    function ejecutar_tramo(k,R,plist,Tlist,qseq)
        % Subconjuntos consistentes para mantener la lógica intacta
        plist_sub = plist(k-1:k);
        Tlist_sub = Tlist(k-1:k);
        qseq_sub  = qseq(:,k-1:k);

        ejecutar_trayectorias(R, plist_sub, Tlist_sub, qseq_sub, n, N);
    end
    


end
