function Q = ik_barista(R, T, q0, q_mejor)
% CIN_INV_BARISTA  Cinemática inversa (6R, muñeca esférica) tipo IRB-120.
% Uso:
%   Q = cin_inv_barista(R, T, q0, q_mejor)
% Entradas:
%   R        : SerialLink (tu R1 de robot.m)
%   T        : SE3 o 4x4 objetivo (Base->Tool)  (puede incluir base/tool)
%   q0       : 6x1 vector articular actual (para elegir la "mejor" y casos degenerados)
%   q_mejor  : booleano; true=devuelve 6x1 (más cercana a q0); false=todas (6xN)
% Salida:
%   Q        : 6x1 o 6xN con soluciones por columnas (normalizadas a [-pi,pi], offsets aplicados)
%
% Resuelve POSICIÓN (q1..q3) y ORIENTACIÓN (q4..q6) mediante desacople
% cinemático (método de Pieper) para muñeca esférica.
    if nargin == 3
        q_mejor = false;
    elseif nargin ~= 4
        error('Argumentos incorrectos')
    end
    
    if isprop(T, 'double')
        T = T.double;
    end

    % ------- anular offsets y desacoplar base/tool -------
    offsets = R.offset;
    R.offset = zeros(6,1);

    T = invHomog(R.base.double) * T * invHomog(R.tool.double);


    % punto de muñeca

    pwrist = T(1:3,4) - R.links(6).d * T(1:3,3);
    L2 = R.links(2).a; %0.27
    L3 = hypot(R.links(3).a,R.links(4).d); 
    phi = atan2(R.links(4).d,R.links(3).a); 

    %% == q1 ==
    q1_list(1) = atan2(pwrist(2), pwrist(1));

    if(q1_list(1) > 0 )
        q1_list(2) = q1_list(1) - pi;
    else
        q1_list(2) = q1_list(1) + pi;
    end
    %% == q3 ==
    r = hypot(pwrist(2),pwrist(1));
    s = pwrist(3) - R.links(1).d;
    arg = (s*s + r*r - L2*L2 - L3*L3)/(2*L2*L3); %cos(theta3)
    if abs(arg)>1
        warning("argumento imaginario, punto posiblemente fuera de alcance de muñeca")
    end
    arg = real(arg);
    q3_list(1) = atan2(sqrt(1-arg*arg), arg); % arcocoseno pero con atan2

    q3_list(2) = atan2(-sqrt(1-arg*arg), arg); 


    %% == q2 == 
    q2_list = [0 0 0 0];
    
    
    q2_list(1) = atan2(-s,r) - atan2(L3*sin(q3_list(1)),L2+L3*cos(q3_list(1)));
    q2_list(2) = atan2(-s,r) - atan2(L3*sin(q3_list(2)),L2+L3*cos(q3_list(2)));
    q2_list(3) = atan2(-s,-r) - atan2(L3*sin(q3_list(1)),L2+L3*cos(q3_list(1)));
    q2_list(4) = atan2(-s,-r) - atan2(L3*sin(q3_list(2)),L2+L3*cos(q3_list(2)));
    
    %% correcion q3 
    q3(1) = q3_list(1)-phi;
    q3(2) = q3_list(2)-phi;

    sols_13(1,:) = [q1_list(1) q1_list(1) q1_list(1) q1_list(1) q1_list(2) q1_list(2) q1_list(2) q1_list(2)];
    sols_13(2,:) = [q2_list(1) q2_list(1) q2_list(2) q2_list(2) q2_list(3) q2_list(3) q2_list(4) q2_list(4)];
    sols_13(3,:) = [     q3(1)      q3(1)      q3(2)      q3(2)      q3(1)      q3(1)      q3(2)      q3(2)];
    
    %% Primer verificacion posicion muñeca
    disp('Verificación centro de muñeca :')
    fprintf('> pwrist:'); disp(pwrist')
    for i=1:8
        T03 = eye(4);
        for j=1:3
            T03 = T03 * R.links(j).A(sols_13(j,i)).double;
        end
        % p_pred = (T03 * [0;0;R.links(4).d;1]);
        % Si el origen de la muñeca está a distancia d4 a lo largo de z3:
        T03 = T03 * R.links(4).A(0).double ;       % si el centro coincide con el origen de {3}
        fprintf('> %d:', i); 
        disp(T03(1:3,4)');
    end
    
    %% = Problema de orientación =
    T1 = R.links(1).A(q1).double;

end
% ===== Helpers locales =====
function [q4,q5,q6] = calcular_orient(R, q1,q2,q3, T,q0)
    
end
function Ti = invHomog(T)
    if isa(T,'SE3'), T = T.double; end
    R = T(1:3,1:3); p = T(1:3,4);
    Ti = [R.', -R.'*p; 0 0 0 1];
end

function y = wrapToPi(x), y = atan2(sin(x),cos(x)); end
function M = wrap_all(M), M = atan2(sin(M),cos(M)); end
function D = angleDiff(QQ, q0)
    D = atan2(sin(QQ - q0(:)), cos(QQ - q0(:)));
end

% ---- error de centro de muñeca para escoger el signo correcto de phi ----
function e = pc_error(q13, R, T, a3, d4, d6)
    % T3 con DH real (sin offsets)
    T1 = R.links(1).A(q13(1)).double;
    T2 = T1*R.links(2).A(q13(2)).double;
    T3 = T2*R.links(3).A(q13(3)).double;

    % Centro de muñeca objetivo (en base), usando d6 del modelo:
    pc_obj_base = T(1:3,4) - d6*T(1:3,3);

    % Expresá ese pc en {3}
    pc3 = invHomog(T3) * [pc_obj_base; 1]; pc3 = pc3(1:3);

    % En {3}, el centro de muñeca ideal es [a3; 0; d4] (DH standard)
    e = norm( [ pc3(1) - a3;   pc3(2) - 0;   pc3(3) - d4 ] );
end
