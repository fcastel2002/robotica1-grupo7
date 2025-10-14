clear;
syms L1 L2 L3 q1 q2 q3 real
q = [q1 q2 q3];

dh = [0 ,0 ,L1 , 0, 1;
      0 ,0 ,L2, 0, 0;
      0 ,0 ,L3, 0, 0];


R = SerialLink(dh, 'name', 'planar3Rprismatic');

T = R.fkine(q);
Ts = simplify(T.T);

px = simplify(Ts(1,4));
py = simplify(Ts(2,4));
phi = simplify(q1 + q3);

pose = [px; py; phi];


J = simplify(R.jacob0(q));
disp("Jacobiano:");
disp(J);
Jr = J(1:3,:); %aca si uso las primeras 3 para el calculo del det porque es la tarea que voy a poder realizar
%== Ej 4 ==%
Jr = subs(Jr,[L1,L2,L3],[1,1,1]);
disp("Det sym:");
detJr = simplify(det(Jr)); 
eqJr = detJr == 0;
disp("Ecuacion singularidad (det(J)=0):")
disp(eqJr);
disp("Variables: ")
disp(symvar(eqJr));
disp("Solucion:")
disp(solve(eqJr, symvar(eqJr)));