function esp_trab_v2(nSamples, nSteps, maxCombPerJoint)
% ... (tus encabezados y validaciones iguales)

    % ------- Defaults seguros --------
    if nargin < 1 || isempty(nSamples) || ~isscalar(nSamples) || nSamples <= 0
        nSamples = 5000;
    else
        nSamples = double(nSamples);
    end
    if nargin < 2 || isempty(nSteps) || ~isscalar(nSteps) || nSteps <= 1
        nSteps = 60;
    else
        nSteps = double(nSteps);
    end

    if nargin < 3 || isempty(maxCombPerJoint) || ~isscalar(maxCombPerJoint) || maxCombPerJoint <= 0
        maxCombPerJoint = 16;   % evita explotar si Rob.n es grande
    end


    % ------- Importar robot y validar qlim (igual que antes) -------
    run('robot.m');
    % ... (detección de Rob y validaciones de qlim iguales)
        % Detectar cuál robot usar: R > R1 > robots{1}
    if exist('R','var')==1 && isa(R,'SerialLink')
        Rob = R;
    elseif exist('R1','var')==1 && isa(R1,'SerialLink')
        Rob = R1;
    elseif exist('robots','var')==1 && iscell(robots) && ~isempty(robots) && isa(robots{1},'SerialLink')
        Rob = robots{1};
    else
        error('robot.m no define R, R1 ni robots{1} como SerialLink.');
    end

    % ------- Validaciones --------
    if isempty(Rob.qlim)
        error('Definí qlim: Rob.qlim = [qmin qmax] por junta (n x 2).');
    end
    if size(Rob.qlim,1) ~= Rob.n || size(Rob.qlim,2) ~= 2
        error('Rob.qlim debe ser de tamaño [Rob.n x 2].');
    end
    qmin = Rob.qlim(:,1)'; qmax = Rob.qlim(:,2)';

    % ================= RELLENO: muestreo aleatorio =================
    % muestreo uniforme en cada q_i dentro de [qmin(i), qmax(i)]
    Qrand = rand(nSamples, Rob.n).* (qmax - qmin) + qmin;  % nSamples x n
    P = zeros(nSamples,3);
    % for i = 1:nSamples
    %     Ti = Rob.fkine(Qrand(i,:));
    %     P(i,:) = transl(Ti);
    % end

    % ===== BORDE (versión 'j barre' y solo 'j+1' en min/max) =====
    curves = {}; labels = {};
    
    qmid = 0.5*(qmin + qmax);    % valor nominal para el resto (puede ser zeros(size(qmid)) si prefieres)
    for j = 1:Rob.n              % junta que barre
        j2 = mod(j, Rob.n) + 1;  % la siguiente (cíclica)
    
        for sgn = [0 1]          % 0 -> min, 1 -> max para la junta j+1
            qfix = qmid;         % todas al nominal
            qfix(j)  = NaN;                      % j barre
            qfix(j2) = (1-sgn)*qmin(j2) + sgn*qmax(j2);   % j+1 en min o max
    
            qa = qfix; qb = qfix;
            qa(j) = qmin(j);  qb(j) = qmax(j);
    
            Qtraj = jtraj(qa, qb, nSteps);
    
            Pc = zeros(nSteps,3);
            for k = 1:nSteps
                T = Rob.fkine(Qtraj(k,:));
                Pc(k,:) = transl(T);
            end
            curves{end+1} = Pc; %#ok<AGROW>
            labels{end+1} = sprintf('J%d[min→max] @ J%d=%s', j, j2, ternary(sgn==0,'min','max'));
        end
    end

    % ===================== Graficado =====================
    figure('Name','Espacio de trabajo - Vistas XY / XZ','Color','w');

    % paleta consistente para todas las curvas
    cmap = lines(numel(curves));

    % --- XY ---
    subplot(1,2,1); hold on; grid on; axis equal;
    title('Vista superior XY'); xlabel('X [m]'); ylabel('Y [m]');
    scatter(P(:,1), P(:,2), 6, '.', 'MarkerEdgeAlpha',0.25);
    h_xy = gobjects(numel(curves),1);
    for i = 1:numel(curves)
        h_xy(i) = plot(curves{i}(:,1), curves{i}(:,2), 'LineWidth', 1.2, 'Color', cmap(i,:));
    end
    try
        Kxy = convhull(P(:,1), P(:,2));
        patch(P(Kxy,1), P(Kxy,2), [0.2 0.2 0.2], 'FaceAlpha',0.06, 'EdgeColor','k');
    catch, end

    % Filtrar los identificadores de gráficos y sus etiquetas correspondientes
    % Considerar tanto la validez del gráfico como que la etiqueta no esté vacía
    % Usar 'isgraphics' para verificar que sea un objeto gráfico 'line'
    % Usar 'isvalid' para verificar que el objeto gráfico no se haya borrado
    % Usar '~cellfun('isempty', labels)' para verificar que la etiqueta no esté vacía
    
    % Primero, combinamos todos los criterios de filtro en un solo paso antes de sub-indexar
    final_filter_xy = arrayfun(@(h, s) isgraphics(h,'line') && isvalid(h) && ~isempty(s{1}), h_xy, labels', 'UniformOutput', true);
    
    h_xy_final = h_xy(final_filter_xy);
    lab_xy_final = labels(final_filter_xy);
    
    % Asegurarse de que las etiquetas se conviertan a 'char 1xN'
    lab_xy_final = cellfun(@(s) char(join(string(s), "")), lab_xy_final, 'UniformOutput', false);

    % Solo llamar a legend si hay elementos para mostrar
    if ~isempty(h_xy_final)
        %legend(h_xy_final, lab_xy_final, 'Interpreter','none', 'Location','bestoutside');
    end
    hold off;

    % --- XZ --- (Aplicar la misma lógica aquí)
    subplot(1,2,2); hold on; grid on; axis equal;
    title('Vista lateral XZ'); xlabel('X [m]'); ylabel('Z [m]');
    scatter(P(:,1), P(:,3), 6, '.', 'MarkerEdgeAlpha',0.25);
    h_xz = gobjects(numel(curves),1);
    for i = 1:numel(curves)
        h_xz(i) = plot(curves{i}(:,1), curves{i}(:,3), 'LineWidth', 1.2, 'Color', cmap(i,:));
    end
    try
        Kxz = convhull(P(:,1), P(:,3));
        patch(P(Kxz,1), P(Kxz,3), [0.2 0.2 0.2], 'FaceAlpha',0.06, 'EdgeColor','k');
    catch, end

    final_filter_xz = arrayfun(@(h, s) isgraphics(h,'line') && isvalid(h) && ~isempty(s{1}), h_xz, labels', 'UniformOutput', true);

    h_xz_final = h_xz(final_filter_xz);
    lab_xz_final = labels(final_filter_xz);
    
    lab_xz_final = cellfun(@(s) char(join(string(s), "")), lab_xz_final, 'UniformOutput', false);

    if ~isempty(h_xz_final)
        %legend(h_xz_final, lab_xz_final, 'Interpreter','none', 'Location','bestoutside');
    end
    hold off;

    % ======== Resumen ========
    xmin = min(P(:,1)); xmax = max(P(:,1));
    ymin = min(P(:,2)); ymax = max(P(:,2));
    zmin = min(P(:,3)); zmax = max(P(:,3));
    rXY = sqrt(P(:,1).^2 + P(:,2).^2);
    rmin = min(rXY); rmax = max(rXY);

    fprintf('\n=== RESUMEN ESPACIO DE TRABAJO (aprox.) ===\n');
    fprintf('X: [%.3f, %.3f] m   (ancho ~ %.3f m)\n', xmin, xmax, xmax-xmin);
    fprintf('Y: [%.3f, %.3f] m   (fondo ~ %.3f m)\n', ymin, ymax, ymax-ymin);
    fprintf('Z: [%.3f, %.3f] m   (alto  ~ %.3f m)\n', zmin, zmax, zmax-zmin);
    fprintf('Radio XY aprox.: r_min=%.3f m, r_max=%.3f m\n\n', rmin, rmax);
end

% ---- utilitario mínimo ----
function out = ternary(cond, a, b), if cond, out=a; else, out=b; end, end
