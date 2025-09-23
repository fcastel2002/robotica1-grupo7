%% TP5B - Ejercicio 1 (Pieper) - It. 6 y 7
% Requiere Robotics Toolbox (Peter Corke)

clear; clc; close all;

%% -------------------------------
%  (a) Definición DH (unidades = 1)
% - Brazo:  q1 (rot Z, α=+pi/2), q2 (a2=1), q3 (a3=1)
% - Muñeca: q4, q5, q6 con ejes que se cortan (a4=a5=a6=0)
%           d6 = 1 (distancia efector → muñeca)
%
% Formato DH de SerialLink: [theta d a alpha sigma]
% sigma=0 (rotacional)
dh = [ ...
    0     0    0     pi/2   0;   % 1
    0     0    1     0      0;   % 2 (L2=1)
    0     0    1     0      0;   % 3 (L3=1)
    0     0    0     pi/2   0;   % 4
    0     0    0    -pi/2   0;   % 5
    0     1    0     0      0];  % 6 (d6=1)

R = SerialLink(dh, 'name', 'Robot6R_Unit');
R.offset = zeros(1,6);           % sin offsets para que sea genérico
disp(R);

% Plot opcional (posición cero)
try
    R.plot(zeros(1,6));
catch
    warning('No se pudo plotear (entorno gráfico no disponible).');
end

%% ---------------------------------------------------------------------------
%% ---------------------------------------------------------------------------
%% ---------------------------------------------------------------------------
% 7 - Verificación con CD
% 1) Elegimos una configuración articular "propuesta"
q_ref = [30 -40 20 15 -25 60]*pi/180;  % ejemplo
T_ref  = R.fkine(q_ref).double;               % CD completa
p6     = T_ref(1:3,4);
z6     = T_ref(1:3,3);
d6     = dh(6,2);                      % = 1
pc_ref = p6 - d6*z6;                   % centro de muñeca (Pieper)

% 2) Resolvemos el "primer problema" para las 4 soluciones (q1..q3)
Q13 = pieper_posicion_3R_seriallink(R, T_ref);

fprintf('\nSoluciones (q1,q2,q3) del primer problema (rad):\n');
disp(Q13);

% 3) Verificamos: todas las soluciones deben dar la MISMA muñeca
%    y una de ellas debe recuperar (q1,q2,q3) de q_ref (salvo wrap/2pi)
tol = 1e-9;
recupera_ref = false;

for k = 1:size(Q13,1)
    qk = [Q13(k,:) q_ref(4:6)];      % completamos con los mismos q4..q6 de q_ref
    Tk = R.fkine(qk).double;
    pk = Tk(1:3,4);
    zk = Tk(1:3,3);
    pc_k = pk - d6*zk;

    err_pc = norm(pc_k - pc_ref);
    fprintf('Sol %d: ||pc - pc_ref|| = %.3e\n', k, err_pc);

    if err_pc < 1e-8
        % chequeo "recupera" (comparación módulo 2*pi)
        dq = angleDiff(Q13(k,:).', q_ref(1:3).');
        if norm(dq) < 1e-6
            recupera_ref = true;
        end
    end
end

if recupera_ref
    fprintf('\n✔ Se recuperó una de las ternas (q1..q3) del q_ref.\n');
else
    fprintf('\n⚠ No se detectó recuperación exacta de (q1..q3); revisar tolerancias o límites.\n');
end

%% Función auxiliar para diferencias angulares modulo 2*pi
function d = angleDiff(a, b)
    % d = wrapToPi(a-b) elemento a elemento
    d = a - b;
    d = atan2(sin(d), cos(d));
end
