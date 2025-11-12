function crear_gui(R, plist, Tlist, qseq, n, N, groups)
% --- (El inicio de la función es igual) ---
if nargin < 5, n = 30; end
if nargin < 6, N = 20; end
if nargin < 7, groups = []; end
fig = gcf;

global demo_tareas; %
% ====================================================================
% --- ¡NUEVO! Cargar todos los handles de la lógica ---
% ====================================================================
% (Esto se mantiene igual, se cargan todos los handles)
try
    handles = barista_callbacks(fig, R, plist, Tlist, qseq, n, N, groups); %
catch ME
    if strcmp(ME.identifier, 'MATLAB:UndefinedFunction')
        msgbox('Error: No se encuentra "barista_callbacks.m". Asegúrate de que esté en el path de MATLAB.', 'Error de Función', 'error');
    else
        rethrow(ME);
    end
    return;
end

% ====================================================================
% --- ¡NUEVA LÓGICA DE LAYOUT CONDICIONAL! ---
% ====================================================================

% Definir variables de layout comunes
h = 0.06; dy = 0.08; y0 = 0.1;

if ~isempty(demo_tareas) && demo_tareas == true
    %% ---------------- MODO DEMO (5 Botones) ----------------
    
    disp('GUI en modo DEMO.');
    
    % --- Botón "Ambos (sync)" (Centro) ---
    y_sync = y0 - dy/2; %
    uicontrol('Parent',fig,'Style','pushbutton','Units','normalized', ...
        'Position',[0.45 y_sync 0.18 h], 'String','Ejecutar Ambos (sync)', ... %
        'FontSize',11, 'FontWeight', 'bold', 'ForegroundColor', [0 0.5 0], ...
        'Callback', @handles.ejecutar_ambos_sync); %

    % --- Botones R1 (Derecha) ---
    x_r1 = 0.85; w_r1 = 0.12; %
    uicontrol('Parent',fig,'Style','pushbutton','Units','normalized', ...
        'Position',[x_r1 y0 w_r1 h], 'String','Ejecutar R1', ... %
        'FontSize',11, ...
        'Callback', @handles.ejecutar_todo_r1); %
        
    y_r1 = y0 - dy;
    uicontrol('Parent',fig,'Style','pushbutton','Units','normalized', ...
        'Position',[x_r1 y_r1 w_r1 h], 'String','Ejecutar Optimizado R1', ... %
        'FontSize',11, 'FontWeight', 'bold', 'ForegroundColor', [0 0 0.8], ... %
        'Callback', @handles.optimizado_r1); %

    % --- Botones R2 (Izquierda) ---
    x_r2 = 0.03; w_r2 = 0.12; %
    uicontrol('Parent',fig,'Style','pushbutton','Units','normalized', ...
        'Position',[x_r2 y0 w_r2 h], 'String','Ejecutar R2', ... %
        'FontSize',11, ...
        'Callback', @handles.ejecutar_todo_r2); %

    y_r2 = y0 - dy; %
    uicontrol('Parent',fig,'Style','pushbutton','Units','normalized', ...
        'Position',[x_r2 y_r2 w_r2 h], 'String','Ejecutar Optimizado R2', ... %
        'FontSize',11, 'FontWeight', 'bold', 'ForegroundColor', [0 0 0.8], ... %
        'Callback', @handles.optimizado_r2); %
        
else
    %% ---------------- MODO COMPLETO (Default) ----------------
    
    % ====================================================================
    % Layout derecho simple robot 1
    % ====================================================================
    x = 0.85; w = 0.12; %
    % Botón "Todo" R1 (Raw)
    uicontrol('Parent',fig,'Style','pushbutton','Units','normalized', ...
        'Position',[x y0 w h], 'String','Ejecutar R1', ... %
        'FontSize',11, ...
        'Callback', @handles.ejecutar_todo_r1); %

    % Botón "Ambos (sync)"
    y_sync = y0 - dy/2; %
    uicontrol('Parent',fig,'Style','pushbutton','Units','normalized', ...
        'Position',[0.45 y_sync 0.18 h], 'String','Ejecutar Ambos (sync)', ... %
        'FontSize',11, 'FontWeight', 'bold', 'ForegroundColor', [0 0.5 0], ...
        'Callback', @handles.ejecutar_ambos_sync); %


    % --- 'y' se inicializa para el botón Optimizado ---
    y = y0 - dy; %

    uicontrol('Parent',fig,'Style','pushbutton','Units','normalized', ...
        'Position',[x y w h], 'String','Arte latte (corazón)', ... %
        'FontSize',11, 'FontWeight', 'bold', 'ForegroundColor', [0.8 0 0.4], ...
        'Callback', @handles.arte_latte); %

    y = y - dy; %

    % --- Botón: Ejecutar Optimizado R1 ---
    uicontrol('Parent',fig,'Style','pushbutton','Units','normalized', ...
        'Position',[x y w h], 'String','Ejecutar Optimizado R1', ... %
        'FontSize',11, 'FontWeight', 'bold', 'ForegroundColor', [0 0 0.8], ... %
        'Callback', @handles.optimizado_r1); %

    % Botones agrupados o por segmento (raw)
    y = y - dy; %

    % ===================================================================
    % --- LÓGICA DE GRUPOS R1 (Modificada para usar handles) ---
    % ===================================================================
    use_groups_r1 = ~isempty(groups) && numel(groups) >= 1 && ~isempty(groups{1}); %

    if use_groups_r1
        for g = 1:numel(groups{1}) %
            idxs = groups{1}{g}; %
            if isempty(idxs), continue; end
            i0 = idxs(1); i1 = idxs(end); %
            etiqueta = sprintf('R1 %d -> %d', i0, i1); %
            uicontrol('Parent',fig,'Style','pushbutton','Units','normalized', ...
                'Position',[x y w h], 'String',etiqueta, 'FontSize',10, ... %
                'Callback', @(~,~) handles.grupo_r1(g)); %
            y = y - dy; %
        end
    else
        % Fallback (aunque ahora los grupos lo manejan todo)
        for k = 2:numel(plist{1}) %
        txt = sprintf('Punto %d → %d', k-1, k); %
        uicontrol('Parent',fig,'Style','pushbutton','Units','normalized', ...
            'Position',[x y w h], 'String',txt, 'FontSize',10, ... %
            'Callback', @(~,~) handles.grupo_r1(k-1)); %
        y = y - dy; %
        end
    end
    % ===================================================================
    % --- FIN DE MODIFICACIÓN LÓGICA (R1) ---
    % ===================================================================


    % ====================================================================
    % Layout izquierdo simple robot 2
    % ====================================================================
    x = 0.03; w = 0.12; %
    % Botón "Todo" R2 (Raw)
    uicontrol('Parent',fig,'Style','pushbutton','Units','normalized', ...
        'Position',[x y0 w h], 'String','Ejecutar R2', ... %
        'FontSize',11, ...
        'Callback', @handles.ejecutar_todo_r2); %

    % --- Botón: Ejecutar Optimizado R2 ---
    y = y0 - dy; %
    uicontrol('Parent',fig,'Style','pushbutton','Units','normalized', ...
        'Position',[x y w h], 'String','Ejecutar Optimizado R2', ... %
        'FontSize',11, 'FontWeight', 'bold', 'ForegroundColor', [0 0 0.8], ... %
        'Callback', @handles.optimizado_r2); %

    % Botones agrupados o por segmento para R2 (raw)
    y = y - dy; %

    % ===================================================================
    % --- LÓGICA DE GRUPOS R2 (Modificada para usar handles) ---
    % ===================================================================
    use_groups_r2 = ~isempty(groups) && numel(groups) >= 2 && ~isempty(groups{2}); %

    if use_groups_r2
        for g = 1:numel(groups{2}) %
            idxs = groups{2}{g}; %
            if isempty(idxs), continue; end
            i0 = idxs(1); i1 = idxs(end); %
            etiqueta = sprintf('R2 %d -> %d', i0, i1); %
            uicontrol('Parent',fig,'Style','pushbutton','Units','normalized', ...
                'Position',[x y w h], 'String',etiqueta, 'FontSize',10, ... %
                'Callback', @(~,~) handles.grupo_r2(g)); %
            y = y - dy; %
        end
    else
        % Fallback
        for k = 2:numel(plist{2}) %
            txt = sprintf('Punto R2 %d → %d', k-1, k); %
            uicontrol('Position',[x y w h],'String',txt, ...
                'Callback', @(~,~) handles.grupo_r2(k-1),'Units','normalized'); %
            y = y - dy; %
        end
    end
    % ===================================================================
    % --- FIN DE MODIFICACIÓN LÓGICA (R2) ---
    % ===================================================================

end % Fin del if/else demo_tareas

%% ====================================================================
%% =========== ¡¡TODAS LAS FUNCIONES HELPER HAN SIDO MOVIDAS!! =========
%% ====================================================================

end