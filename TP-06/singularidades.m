clear;
run irb120_sym_data.m

R1_brazo = SerialLink(R1.links(1:4), 'name','brazoBarista');
R1_brazo.offset = R1.offset(1:4);
symq_brazo = symq(1:4);
J_brazo = R1_brazo.jacob0(symq_brazo);
% Tomamos solo la parte lineal
Jv_brazo = J_brazo(1:3,1:3); 
% responderemos a la pregunta de bajo que qué condiciones las
% articulaciones pierden la capacidad de mover el centro de la muñeca
% luego la muñeca se encarga de orientar el efector final
detJ_brazo = det(Jv_brazo);
detJ_brazo = expand(detJ_brazo);
detJ_brazo = simplify(detJ_brazo);
detJ_brazo_fact = factor(detJ_brazo);

disp("---------------------------------------------------------")
disp("Determinante del Jacobiano del Brazo:")
disp(detJ_brazo_fact)
disp("---------------------------------------------------------")