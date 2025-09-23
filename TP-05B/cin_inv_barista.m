function Q = cin_inv_barista(R, T, q0, q_mejor)
% CIN_INV_BARISTA  Cinemática inversa (6R, muñeca esférica) para tu robot tipo IRB-120.
% Uso:
%   Q = cin_inv_barista(R, T, q0, q_mejor)
% Entradas:
%   R        : objeto SerialLink (tu R1 de robot.m)
%   T        : SE3 o 4x4 objetivo (Base->Tool)  (puede incluir base/tool)
%   q0       : 6x1 vector articular actual (para elegir "la mejor" y para caso degenerado)
%   q_mejor  : booleano; true=devuelve 6x1 (solución más cercana a q0); false=todas (6xN)
% Salida:
%   Q        : 6x1 o 6xN con soluciones por columnas (normalizadas a [-pi,pi], offsets aplicados)

    % ----- sanity -----
    if nargin < 4, q_mejor = false; end
    if isa(T,'SE3'), T = T.double; end
    if size(T,1)~=4 || size(T,2)~=4, error('T debe ser 4x4'); end
    if numel(q0)~=6, q0 = zeros(6,1); end
    q0 = q0(:);

    % ===== 0) neutralizar base/tool y offsets (como recomienda la cátedra) =====
    T = invHomog(double(R.base)) * T * invHomog(double(R.tool));                    % 
    offsets = R.offset;  R.offset = zeros(size(offsets));                           % 

    % ===== 1) Pieper - Problema de POSICIÓN: q1,q2,q3 (hasta 4 soluciones) =====
    d6  = R.links(6).d;
    p6  = T(1:3,4);
    z6  = T(1:3,3);
    pc  = p6 - d6*z6;                                                               % 

    q1_a = atan2(pc(2),pc(1));                          % primer plano
    q1_b = wrapToPi(q1_a + pi);                         % plano opuesto             
    q1_all = [q1_a; q1_b];

    L2 = R.links(2).a;  % 0.270 en tu robot.m
    L3 = R.links(3).a;  % 0.070 en tu robot.m

    sols13 = [];  % filas: [q1 q2 q3]
    for i=1:2
        q1 = q1_all(i);
        T01 = R.links(1).A(q1).double;                  % DH real del link 1
        p1  = invHomog(T01) * [pc;1];  p1 = p1(1:3);    % referenciar pc a {1}      

        x1 = p1(1); y1 = p1(2);
        r  = hypot(x1,y1);
        beta = atan2(y1,x1);

        c3 = (r^2 - L2^2 - L3^2)/(2*L2*L3);             % ley de cosenos 2R         
        if abs(c3) > 1+1e-12, c3 = sign(c3); end
        c3 = max(min(c3,1),-1);
        s3 = sqrt(max(0,1-c3^2));

        th3s = [atan2(+s3,c3), atan2(-s3,c3)];          % codo arriba/abajo
        for th3 = th3s
            th2 = beta - atan2(L3*sin(th3), L2 + L3*cos(th3));
            sols13(end+1,:) = [wrapToPi(q1) wrapToPi(th2) wrapToPi(th3)]; %#ok<AGROW>
        end
    end
    sols13 = uniquetol(sols13,1e-12,'ByRows',true);

    % Si no hay alcanzabilidad, restaurar y salir vacío
    if isempty(sols13)
        R.offset = offsets; Q = zeros(6,0); return;
    end

    % ===== 2) Pieper - Problema de ORIENTACIÓN: q4,q5,q6 (2 por cada terna previa) =====
    QQ = [];   % 6xN
    for k=1:size(sols13,1)
        q1 = sols13(k,1);  q2 = sols13(k,2);  q3 = sols13(k,3);

        T1 = R.links(1).A(q1).double;
        T2 = T1*R.links(2).A(q2).double;
        T3 = T2*R.links(3).A(q3).double;

        T36 = invHomog(T3) * T;                                                 % 

        % q4 base: usando proyección del eje Z de T36 sobre plano X3-Y3
        q4_1 = atan2(T36(2,3), T36(1,3));
        q4_2 = wrapToPi(q4_1 + pi);                                             % simétrico muñeca       

        q4cands = [q4_1 q4_2];
        for q4 = q4cands
            T4  = R.links(4).A(q4).double;
            T46 = invHomog(T4) * T36;

            % Chequeo degenerado: q5 ~ 0 (Z4 || Z6)                              
            if abs(T36(3,3) - 1) < eps
                % caso degenerado: fijar q4 con q0(4) y resolver sólo q6
                q4d = q0(4);                             % estrategia recomendada
                T4d = R.links(4).A(q4d).double;
                T46d= invHomog(T4d) * T36;
                q5d = 0;
                q6d = atan2(T46d(2,1), T46d(1,1));       % ajustar por q4 ya aplicado
                QQ(:,end+1) = wrap_all([q1;q2;q3;q4d;q5d;q6d]); %#ok<AGROW>
                continue
            end

            % q5 desde proyección del Z del extremo respecto a {4}
            q5 = atan2(T46(2,3), T46(1,3)) - pi/2;                                % 

            % q6 desde X6 en {5}
            T5  = R.links(5).A(q5).double;
            T56 = invHomog(T5) * T46;
            q6  = atan2(T56(2,1), T56(1,1));                                      % 

            QQ(:,end+1) = wrap_all([q1;q2;q3;q4;q5;q6]); %#ok<AGROW>
        end
    end

    % ===== 3) Reaplicar offsets y filtrar por límites articulares =====
    if isempty(QQ), R.offset = offsets; Q = zeros(6,0); return; end

    QQ = uniquetol(QQ.',1e-12,'ByRows',true).';           % quitar duplicados
    QQ = QQ - offsets(:)*ones(1,size(QQ,2));              % re-aplicar offsets       

    % Filtrar por qlim (si están definidos)
    if ~isempty(R.qlim)
        keep = true(1,size(QQ,2));
        for j=1:size(QQ,2)
            for i=1:6
                if QQ(i,j) < R.qlim(i,1)-1e-9 || QQ(i,j) > R.qlim(i,2)+1e-9
                    keep(j) = false; break
                end
            end
        end
        QQ = QQ(:,keep);
    end

    % Si no queda ninguna por límites, restauro y devuelvo vacío
    R.offset = offsets;
    if isempty(QQ), Q = zeros(6,0); return; end

    % ===== 4) Elegir "la mejor" vs "todas" =====
    if q_mejor
        % distancia euclídea angular modulo 2pi (como sugiere el material)         
        D = vecnorm(angleDiff(QQ, q0), 2, 1);
        [~,pos] = min(D);
        Q = QQ(:,pos);
    else
        Q = QQ;
    end
end

% ===== Helpers locales =====
function Ti = invHomog(T)
    if isa(T,'SE3'), T = T.double; end
    R = T(1:3,1:3); p = T(1:3,4);
    Ti = [R.', -R.'*p; 0 0 0 1];
end

function y = wrapToPi(x), y = atan2(sin(x),cos(x)); end

function M = wrap_all(M), M = atan2(sin(M),cos(M)); end

function D = angleDiff(QQ, q0)
    % resta angular columna a columna, modulo 2pi
    D = atan2(sin(QQ - q0(:)), cos(QQ - q0(:)));
end
