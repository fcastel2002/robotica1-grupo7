function Q13 = pieper_posicion_3R_seriallink(R, T)
% Devuelve hasta 4 ternas [q1 q2 q3] que ubican la muñeca en pc = p6 - d6*z6
% R : SerialLink (6R con muñeca esférica, d6 > 0)
% T : matriz homogénea objetivo (base->tool)
%
% 2 valores de q1 (atan2 y ±pi), y para cada uno
%       "codo arriba/abajo" en el 2R del plano x1-y1. (4 soluciones)
%
% Paso a paso: 
% cf. TP5B Ej.1 y Material Adicional (desacople Pieper). 
% - Cálculo de pc: pc = p6 - d6*z6
% - q1 = atan2(yc, xc) y (q1±pi)  (ítems 1 y 2)
% - Referenciar pc al sistema {1} correspondiente y resolver 2R:
%   r = hypot(x1,y1), beta = atan2(y1,x1)
%   c3 = (r^2 - L2^2 - L3^2)/(2 L2 L3),  th3 = ±acos(c3)
%   th2 = beta - atan2(L3 sin th3, L2 + L3 cos th3)
%
% L2 = R.links(2).a ; L3 = R.links(3).a (unitarios)

    % d6 (distancia efector → muñeca)
    d6 = R.links(6).d;    % aquí = 1 (DH unitario del script)
    p6 = T(1:3,4);
    z6 = T(1:3,3);
    pc = p6 - d6*z6;      % centro de muñeca

    % Dos valores de q1
    q1_a = atan2(pc(2), pc(1));
    q1_b = wrapToPi(q1_a + pi);
    q1_all = [q1_a; q1_b];

    L2 = R.links(2).a;    % =1
    L3 = R.links(3).a;    % =1

    sols = [];

    for i=1:2
        q1 = q1_all(i);

        % T01 usando solo rot Z(q1), α1=+pi/2, a1=0, d1=0: usamos el A del link1
        T01 = R.links(1).A(q1).double;

        % pc en {1}
        p1 = invHomog(T01) * [pc; 1];
        p1 = p1(1:3);

        % plano x1-y1
        x1 = p1(1);  y1 = p1(2);
        r  = hypot(x1, y1);
        beta = atan2(y1, x1);

        % Alcanzabilidad
        c3 = (r^2 - L2^2 - L3^2)/(2*L2*L3);
        if abs(c3) > 1+1e-12
            % fuera del workspace; no agregamos soluciones
            continue;
        end
        c3 = max(min(c3,1), -1); % clamp numérico
        s3 = sqrt(max(0, 1 - c3^2));

        % Dos codos: +s3 y -s3
        th3_list = [atan2(+s3, c3), atan2(-s3, c3)];

        for th3 = th3_list
            th2 = beta - atan2(L3*sin(th3), L2 + L3*cos(th3));
            sols(end+1,:) = [wrapToPi(q1), wrapToPi(th2), wrapToPi(th3)]; %#ok<AGROW>
        end
    end

    % Quitar duplicados numéricos (poco frecuente, pero prolijo)
    if isempty(sols)
        Q13 = zeros(0,3);
    else
        Q13 = uniquetol(sols, 1e-10, 'ByRows', true);
    end
end
