%% TP5B - Ejercicio 1 (Pieper) con R1 (ABB IRB120-like del robot.m)
% Ítems 6 y 7: DH + SerialLink + verificación por CD
% SOLO depende de robot.m (carga R1)

clear; clc; close all;

%% 1) Cargar RRobot.m
if exist('robot.m','file')~=2
    error('No se encontró robot.m en el path.');
end
run('robot.m');                 % define R1, R2, offsets y qlim
R = R1;                         % trabajamos con el primer brazo

% Fallback: wrapToPi local si no existe
if ~exist('wrapToPi','file')
    wrapToPi = @(x) atan2(sin(x),cos(x));
end

%% 2) Configuración de referencia (para verificar CD – ítem 7)
q_ref = [30 -40 20 15 -25 60]*pi/180;    % (se puede cambiar)
T_ref = R.fkine(q_ref);                   % CD completa
if isa(T_ref,'SE3'), T_ref = T_ref.double; end

% Desacople de base/tool (por si los usás en otros T):
T_ref = invHomog(double(R.base)) * T_ref * invHomog(double(R.tool));  
p6  = T_ref(1:3,4);
z6  = T_ref(1:3,3);
d6  = R.links(6).d;                       % en tu DH: 0.072
pc_ref = p6 - d6*z6;                      % centro de muñeca (Pieper)

%% 3) Resolver el PRIMER PROBLEMA (q1,q2,q3) – cuatro soluciones
Q13 = pieper_posicion_3R_con_R(R, T_ref); % con DH (R1) y devuelve 4 filas

fprintf('\nSoluciones (q1,q2,q3) del primer problema (rad):\n');
disp(Q13);

%% 4) Verificación del ítem 7: misma muñeca y recuperación de (q1..q3)
recupera = false; tol_pc = 1e-8; tol_ang = 1e-6;

for k = 1:size(Q13,1)
    qk = [Q13(k,:) q_ref(4:6)];          % completamos con la misma muñeca de q_ref
    Tk = R.fkine(qk); if isa(Tk,'SE3'), Tk = Tk.double; end

    pk = Tk(1:3,4);
    zk = Tk(1:3,3);
    pc_k = pk - d6*zk;

    err_pc = norm(pc_k - pc_ref);
    fprintf('Sol %d: ||pc - pc_ref|| = %.3e\n', k, err_pc);

    dq = angleDiff(Q13(k,:).', q_ref(1:3).');
    if err_pc < tol_pc && norm(dq) < tol_ang
        recupera = true;
    end
end

if recupera
    fprintf('\n✔ Se recuperó una terna (q1..q3) del q_{ref}.\n');
else
    fprintf('\n⚠ No se recuperó exactamente (q1..q3); revisar límites/offsets/tolerancias.\n');
end

%% ===== Helpers locales =====

function Q13 = pieper_posicion_3R_con_R(R, T)
    % Neutralizar offsets durante el cálculo (como sugiere la cátedra)
    offsets = R.offset; 
    R.offset = zeros(size(offsets));

    % Centro de muñeca pc = p6 - d6*z6
    if isa(T,'SE3'), T = T.double; end
    T = invHomog(double(R.base)) * T * invHomog(double(R.tool));
    p6 = T(1:3,4); z6 = T(1:3,3);
    d6 = R.links(6).d;
    pc = p6 - d6*z6;

    % Dos valores de q1: atan2 y ±pi
    q1_a = atan2(pc(2), pc(1));
    q1_b = wrapToPi(q1_a + pi);
    q1_all = [q1_a; q1_b];

    L2 = R.links(2).a;               %  DH: 0.270
    L3 = R.links(3).a;               %  DH: 0.070 ( muñeca esférica+offsets ya neutralizados)

    sols = [];
    for i = 1:2
        q1 = q1_all(i);

        % T01 (DH real del link1)
        T01 = R.links(1).A(q1).double;

        % pc en {1}
        p1 = invHomog(T01)*[pc;1]; p1 = p1(1:3);

        % plano x1-y1
        x1 = p1(1); y1 = p1(2);
        r  = hypot(x1,y1);
        beta = atan2(y1,x1);

        % ley de cosenos en 2R (codo ±)
        c3 = (r^2 - L2^2 - L3^2)/(2*L2*L3);
        if abs(c3) > 1+1e-12,  c3 = sign(c3); end
        c3 = max(min(c3,1),-1);
        s3 = sqrt(max(0,1-c3^2));

        th3_list = [atan2(+s3,c3), atan2(-s3,c3)];
        for th3 = th3_list
            th2 = beta - atan2(L3*sin(th3), L2 + L3*cos(th3));
            sols(end+1,:) = [wrapToPi(q1), wrapToPi(th2), wrapToPi(th3)]; %#ok<AGROW>
        end
    end

    % Reaplicar offsets al resultado final (como en el material)
    if ~isempty(sols)
        sols = uniquetol(sols,1e-12,'ByRows',true);
        Q13 = sols - offsets(1:3);  % compatibilidad con el R original (con offsets)
    else
        Q13 = zeros(0,3);
    end

    % Restaurar offsets del robot
    R.offset = offsets;

    % --- helpers anidados ---
    function y = wrapToPi(x), y = atan2(sin(x),cos(x)); end
end

function Ti = invHomog(T)
    % Inversa eficiente de homogénea [R p; 0 1]
    if isa(T,'SE3'), T = T.double; end
    R = T(1:3,1:3); p = T(1:3,4);
    Rt = R.'; Ti = [Rt, -Rt*p; 0 0 0 1];
end

function d = angleDiff(a,b)
    d = a - b;
    d = atan2(sin(d), cos(d));
end
