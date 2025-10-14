clear;
syms L1 L3 q1 q2 q3 real
q = [q1 q2 q3];

dh = [0 ,0 ,0 , pi/2, 0;
      0 ,0 ,0 , -pi/2, 1;
      0 ,0 ,L3, 0, 0];

R = SerialLink(dh, 'name', 'planar3Rprismatic');

T = R.fkine(q);
Ts = simplify(T.T);

px = simplify(Ts(1,4));
py = simplify(Ts(2,4));
gamma = simplify(q1 + q3);

pose = [px; py; gamma];


J = simplify(R.jacob0(q));
disp("Jacobiano:");
disp(J);
Jr = J([1,2, end],:); %uso x, y, gamma (el robot es planar)
%== Ej 4 ==%
Jr = subs(Jr,[L1,L3],[1,1]);
disp("Det sym:");
detJr = simplify(det(Jr)); 
eqJr = detJr == 0;
disp("Ecuacion singularidad (det(J)=0):")
disp(eqJr);
disp("Variables: ")
disp(symvar(eqJr));
disp("Solucion:")
disp(solve(eqJr, symvar(eqJr)));
