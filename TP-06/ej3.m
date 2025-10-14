syms q1 q2 a1 a2 real
q = [q1 q2];
dh = [0 0 a1 0 0;
      0 0 a2 0 0];
R = SerialLink(dh);
J = simplify(R.jacob0(q));
disp(J);