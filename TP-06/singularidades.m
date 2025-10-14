% --- Transforms de los 3 primeros eslabones ---
A1 = R1.A(1, symq);  if isa(A1,'SE3'), A1 = A1.T; end
A2 = R1.A(2, symq);  if isa(A2,'SE3'), A2 = A2.T; end
A3 = R1.A(3, symq);  if isa(A3,'SE3'), A3 = A3.T; end
T01 = A1; T02 = simplify(A1*A2); T03 = simplify(T02*A3);

% --- Centro de muñeca: O4 = O3 + d4*z3 ---
d4  = sym(0.302);
o3  = transl(T03);
z3  = T03(1:3,3);
owc = simplify(o3 + d4*z3);            % NO depende de q4,q5,q6

% --- rho^2 y singularidad de hombro ---
rho2 = simplify(owc(1)^2 + owc(2)^2);  % rho es el modulo del vector posicion (distancia)
rho2_fact = factor(rho2);

% --- Jacobiano lineal del brazo en O4 ---
o0 = sym([0;0;0]);  z0 = sym([0;0;1]);
o1 = transl(T01);   z1 = T01(1:3,3);
o2 = transl(T02);   z2 = T02(1:3,3);

Jv_arm = simplify([ cross(z0, owc-o0), ...
                    cross(z1, owc-o1), ...
                    cross(z2, owc-o2) ]);

detJ_arm = simplify(det(Jv_arm));
[num,~]  = numden(detJ_arm);
detJ_arm = simplify(num);
detJ_arm_fact = factor(detJ_arm);

% el numerador de rho2
% da->(a2/2*cos(q2+q3)+a3/2*sin(q2+q3)+d4/2*sin(q2)^2)^2
% por lo tanto para buscar los ceros...
a2 = sym(0.270);
a3 = sym(0.07);
d4 = sym(0.302);
% f == 0 ;
q2 = symq(2);
q3 = symq(3);
% la funcion f sirve para reemplazar valores de q2 y q3 
% singularidad puntual (brazo)
[num,~] = numden(simplify(rho2));
f = simplify(sqrt(num));


%% curva q3
% tambien se puede establecer las siguientes relaciones.(codo arriba codo
% abajo)
% sirve para ver que valor de q3 produce una singularidad en funcion de un
% valor de q2
R = sqrt(d4^2 + a3^2);
alpha = atan2(a3,d4);
arg = -a2/R * sin(q2);
q3_plus = simplify(alpha + acos(arg)-q2);
q3_minus = simplify(alpha - acos(arg)-q2);

%% Muñeca - Tool
A4 = R1.A(4, symq); if isa(A4,'SE3'), A4=A4.T; end
A5 = R1.A(5, symq); if isa(A5,'SE3'), A5=A5.T; end
T04 = simplify(T03*A4);
T05 = simplify(T04*A5);
z3 = T03(1:3,3);
z4 = T04(1:3,3);
z5 = T05(1:3,3);
Jw_ang = simplify([z3 z4 z5]);
detJ_wrist = simplify(det(Jw_ang));
[num_w, ~] = numden(detJ_wrist);
% se supone que detJ_wrist deberia quedar proporcional a sin(q5)
q5 = symq(5);
assert( simplify(subs(detJ_wrist, q5, 0)) == 0 );
assert( simplify(subs(detJ_wrist, q5, pi)) == 0 );

vars_w = symvar(detJ_wrist);
assert(isempty(setdiff(vars_w, sym('q',[1 6]))));
%% Determinante total
% El determinante total ahora se puede obtener mediante el producto
% de detJ_arm y detJ_wrist, pero no tiene mucho sentido expresarlo 
% todo junto.
%
% --- Verificación estricta ---
assert(isempty(setdiff(symvar(owc),        sym('q',[1 3]))));
assert(isempty(setdiff(symvar(rho2_fact),  sym('q',[1 3]))));
assert(isempty(setdiff(symvar(detJ_arm),   sym('q',[1 3]))));

disp("Determinante Jacobiano brazo (lineal):")
disp(detJ_arm == 0);
disp("Determinante Jacobiano muñeca (angular):")
disp(detJ_wrist == 0);