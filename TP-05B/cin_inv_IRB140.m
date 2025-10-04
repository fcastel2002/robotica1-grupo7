function Q = cin_inv_IRB140(R, T, q0, mejor)
if nargin == 3
    mejor = false;
elseif nargin ~= 4
    error('Argumentos incorrectos')
end

% Eliminación de offsets --------------------------------------------------
offsets = R.offset;
R.offset = zeros(6,1);

% Desacople de base y tool ------------------------------------------------
T =  invHomog(R.base.double) * T.double * invHomog(R.tool.double);

% Punto (x,y,z) de la muñeca ----------------------------------------------
p = T(1:3,4) - R.links(6).d * T(1:3,3);

% Cálculo de q1 -----------------------------------------------------------
%   > 2 posibles soluciones
q1 = calcular_q1(p); % > q1(1) y q1(2)

% Cálculo de q2 -----------------------------------------------------------
%   > 2 posibles soluciones para cada q1
q21 = calcular_q2(R, q1(1), p); % > q21(1) y q21(2)
q22 = calcular_q2(R, q1(2), p); % > q22(1) y q22(2)

% Cálculo de q3 -----------------------------------------------------------
%   > 1 valor posible por cada par (q1,q2)
q311 = calcular_q3(R, q1(1), q21(1), p);
q312 = calcular_q3(R, q1(1), q21(2), p);
q321 = calcular_q3(R, q1(2), q22(1), p);
q322 = calcular_q3(R, q1(2), q22(2), p);

% Acomodo de soluciones en qq ---------------------------------------------
qq(1,:) = [q1(1)  q1(1)  q1(1)  q1(1)  q1(2)  q1(2)  q1(2)  q1(2)];
qq(2,:) = [q21(1) q21(1) q21(2) q21(2) q22(1) q22(1) q22(2) q22(2)];
qq(3,:) = [q311   q311   q312   q312   q321   q321   q322   q322];

% Verificación parcial ----------------------------------------------------
disp('Verificación parcial:')
fprintf('> p:')
disp(p')
for i=1:8
    Taux = eye(4);
    for j=1:3
        Taux = Taux * R.links(j).A(qq(j,i)).double;
    end
    Taux = Taux * R.links(4).A(0).double;
    fprintf('> %d:', i)
    disp(Taux(1:3,4)')
end

% Cálculo de q4, q5 y q6 --------------------------------------------------
for i=1:2:7
    q1 = qq(1, i);
    q2 = qq(2, i);
    q3 = qq(3, i);
    [q4, q5, q6] = calcular_qm(R, q1, q2, q3, T, q0);
    qq(4:6,i:i+1) = [q4; q5; q6];
end
% Offset ------------------------------------------------------------------
R.offset = offsets;
qq = qq - R.offset' * ones(1,8);

% Verificación total ------------------------------------------------------
disp('Verificación total:')
fprintf('> T:')
disp([tr2rpy(T),transl(T)'])
for i=1:8
    Taux = R.fkine(qq(:,i));
    fprintf('> %d:', i)
    disp([Taux.tr2rpy, Taux.transl])
end

% Devolución --------------------------------------------------------------
if mejor
    Qaux = qq  - q0' * ones(1,8);
    normas = zeros(1,8);
    for i=1:8
        normas(i) = norm(Qaux(:,i));
    end
    [~,pos] = min(normas);
    Q = qq(:, pos);
else
    Q = qq;
end
end
%=========================================================================%
%=========================================================================%
% Inversa de matriz homogénea
function iT = invHomog(T)
iT = eye(4);
iT(1:3, 1:3) = T(1:3, 1:3)';
iT(1:3, 4) = - iT(1:3, 1:3) * T(1:3, 4);
end
%=========================================================================%
%=========================================================================%
% Cálculo de q1
function q1 = calcular_q1(p)
q1(1) = atan2(p(2), p(1)); % -pi >= atan2 <= pi
if q1(1) > 0, q1(2) = q1(1) - pi; else, q1(2) = q1(1) + pi; end
end
%=========================================================================%
%=========================================================================%
% Cálculo de q2
function q2 = calcular_q2(R, q1, p)
T1 = R.links(1).A(q1).double;
p = invHomog(T1) * [p; 1]; % p respecto de {1}

B = atan2(p(2), p(1));
r = sqrt(p(1)^2 + p(2)^2);
L2 = R.links(2).a; % a2
L3 = R.links(4).d; % d4
G = acos((L2^2 + r^2 - L3^2) / (2 * r * L2));
if (imag(G)>0)
    warning("IMAGINARIO!")
end
q2(1) = B - real(G);
q2(2) = B + real(G);
end
%=========================================================================%
%=========================================================================%
% Cálculo de q3
function q3 = calcular_q3(R, q1, q2, p)
disp('>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> T2')
T1 = R.links(1).A(q1).double;
T2 = T1 * R.links(2).A(q2).double
p = invHomog(T2) * [p; 1] % p respecto de {2}

q3 = atan2(p(2), p(1)) - pi/2;
end
%=========================================================================%
%=========================================================================%
% Cálculo de q4,q5,q6
function [q4,q5,q6] = calcular_qm(R, q1, q2, q3, T, q0)
T1 = R.links(1).A(q1).double;
T2 = R.links(2).A(q2).double;
T3 = R.links(3).A(q3).double;

T36 = invHomog(T3) * invHomog(T2) * invHomog(T1) * T;
disp(T36(3,3))
if abs(T36(3,3) - 1) < eps
    % solución degenerada:
    %   > z3 y z5 alineados (y z6)
    %   > q4 y q6 generan el mismo movimiento
    %   > q5 = 0 (o q5 = 180º)
    %   > se asume q4 = q4_anterior
    warning('Caso degenerado')
    q4(1) = q0(4);
    q5(1) = 0;
    q6(1) = atan2(T36(2,1), T36(1,1)) - q4(1);
    q4(2) = q4(1);
    q5(2) = 0;
    q6(2) = q6(1);
else
    % solución normal:
    q4(1) = atan2(T36(2,3), T36(1,3)); %atan2(-T36(2,3), -T36(1,3))
    if q4(1) > 0, q4(2) = q4(1) - pi; else, q4(2) = q4(1) + pi; end
    q5 = zeros(1,2);
    q6 = q5;
    for i=1:2
        T4 = R.links(4).A(q4(i)).double;
        T6 = invHomog(T4) * T36;
        q5(i) = atan2(T6(2,3), T6(1,3)) - pi/2;
        
        T5 = R.links(5).A(q5(i)).double;
        T6 = invHomog(T5) * T6;
        q6(i) = atan2(T6(2,1),T6(1,1));
    end
end
end
%=========================================================================%
%=========================================================================%