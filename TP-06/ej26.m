%% Jacobiano y determinante simbólico del IRB 140

% - CASO 1: Jacobiano y determinante "convencional" (lento)
% - CASO 2: Factorización por bloques (muñeca esférica) en centro de muñeca (rápido)
% - CASO 3: Test numérico para comparar ceros (singularidades) entre dos expresiones

clc; clear; close all

% Variables simbólicas:
syms q1 q2 q3 q4 q5 q6 real
q = [q1 q2 q3 q4 q5 q6];

% IRB 140:
dh = [
    q1  0.352  0.070 -pi/2  0;
    q2  0.000  0.360  0.000 0;
    q3  0.000  0.000 -pi/2  0;
    q4  0.380  0.000  pi/2  0;
    q5  0.000  0.000 -pi/2  0;
    q6  0.065  0.000  0.000 0];

% % Creamos el Robot:
R = SerialLink(dh);

% Fuerzo base/tool/offset neutros para partir de una referencia limpia
R.base = eye(4);
R.tool = eye(4);
R.offset = [0 0 0 0 0 0];

%% Selección de caso a ejecutar
% 1 = determinante "completo" (puede ser MUY lento)
% 2 = factorizar en 3x3*3x3 en el centro de muñeca (rápido)
% 3 = test numérico de igualdad de ceros entre dos expresiones
CASO = 2;

fprintf('\n=== Ejecutando CASO %d ===\n', CASO);

if CASO == 1
    %% CASO 1 — Determinante completo (convencional)
    % Advertencia: calcular y simplificar det(J) simbólico 6x6 puede tardar MUCHO.
    tic

    J = R.jacob0(q)
    Jsimple = simplify(J)
    detJ = det(Jsimple) % esto puede ser muy largo → no manejable
    detJsimple = simplify(detJ) % esto puede demorar demasiado, hasta horas

    toc
    % Resultado:
    %     -(171*cos(q3)*sin(q5)*(36*cos(q2) - 38*sin(q2 + q3) + 7))/125000
    %
    % Elapsed time is 925.268873 seconds.

elseif CASO == 2
    %% CASO 2 — Factorización por bloques en el centro de muñeca (muñeca esférica)
    % Idea: mover el punto de cálculo al centro de la muñeca (intersección ejes 4–6).
    % Con eso, J(1:3,4:6) = 0 y det(J) = det(Jv(1:3,1:3)) * det(Jw(4:6,4:6)).
    % Esto separa singularidades de brazo y muñeca y acelera muchísimo.
    tic

    R.tool = transl(0,0,-0.065); % traslado el extremo al centro de la muñeca
    J = R.jacob0(q);

    fprintf('> Verificación bloque superior derecho J(1:3,4:6):\n');
    disp(simplify(J(1:3,4:6)));

    Jv = J(1:3,1:3);
    Jw = J(4:6,4:6);

    detJ_arm = det(Jv);
    detJ_wrist = det(Jw);

    % Simplificaciones previas
    detJ_arm   = simplify(detJ_arm);
    detJ_wrist = simplify(detJ_wrist);

    detJ_fact = simplify(detJ_arm * detJ_wrist)
    toc
    % Resultado:
    %     -(171*cos(q3)*sin(q5)*(36*cos(q2) - 38*sin(q2 + q3) + 7))/125000
    %
    % Elapsed time is 4.435477 seconds.
end





