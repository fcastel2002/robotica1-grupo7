function Q = filtrar_q_lim(R, qq, q0, q_mejor)
% FILTRAR_Q_LIM Filtra soluciones IK según límites articulares (R.qlim)
% Uso:
%   Q = filtrar_q_lim(R, qq, q0, q_mejor)
% Entradas:
%   R        : objeto SerialLink con campo .qlim definido (nx2)
%   qq       : matriz 6xN de soluciones (una por columna)
%   q0       : vector 6x1 actual
%   q_mejor  : true -> devuelve la más cercana dentro de límites
%              false -> devuelve todas las válidas
% Salida:
%   Q        : 6x1 o 6xM con soluciones dentro de límites

    if isempty(R.qlim)
        warning('El robot no tiene límites definidos en R.qlim');
        Q = qq;
        return;
    end

    % Verificar límites
    inlim = all( qq >= R.qlim(:,1)-1e-9 & qq <= R.qlim(:,2)+1e-9, 1 );
    qq_val = qq(:, inlim);

    if isempty(qq_val)
        error('No hay soluciones dentro de los límites articulares.');
        % Elegir la más cercana a q0 aunque esté fuera de rango
        [~, pos] = min(vecnorm(qq - q0, 2, 1));
        Q = qq(:, pos);
        return;
    end

    if q_mejor
        % Escoger la válida más cercana a q0
        [~, pos] = min(vecnorm(qq_val - q0, 2, 1));
        Q = qq_val(:, pos);
    else
        % Devolver todas las válidas
        Q = qq_val;
    end
end


