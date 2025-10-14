% Ejemplos para el TP6

%=========================================================================%
clc, clear, close all
%=========================================================================%
fprintf('######################################################\n')
fprintf('#                Ejemplos para el TP6                #\n')
fprintf('######################################################\n\n')
fprintf('1: Ejercicio 1\n')
fprintf('2: Ejercicio 3\n')
fprintf('3: Ejercicio 4\n')
OPT = input('\nOpción: ');
%=========================================================================%
if OPT == 1
    fprintf('> Ejercicio 1)\n\n')
    syms x y z A B G real
    
    syms q1 q2 a1 a2 real
    q = [q1 q2];
    
    dh = [0,0,a1,0,0;
        0,0,a2,0,0];
    R = SerialLink(dh);
    J = simplify(R.jacob0(q));
    fprintf('J =\n')
    disp(J)
    
    Jr = J(1:2,:);
    
    DJr = simplify(det(Jr));
    fprintf('det(J) = ')
    disp(DJr)
    
    fprintf('(presione enter para continuar)\n')
    pause
    fprintf('> 2)\n\n')
    a1 = 1
    a2 = 1
    q = [pi/6 pi/6]'
    qd = [-1 0]'
    
    dh = [0,0,a1,0,0;
        0,0,a2,0,0];
    R = SerialLink(dh);
    J = R.jacob0(q);
    pd = J*qd
    
    fprintf('(presione enter para continuar)\n')
    pause
    fprintf('> 3)\n\n')
    Jr = J(1:2,:);
    qd = inv(Jr) * pd(1:2,1)
elseif OPT == 2
    fprintf('> Ejercicio 3)\n\n')
    syms q1 q2 a1 a2 real
    q = [q1 q2];
    dh = [0,0,a1,0,0;
        0,0,a2,0,0];
    R = SerialLink(dh);
    J = simplify(R.jacob0(q));
    disp(J)
elseif OPT == 3
    fprintf('> Ejercicio 4)\n\n')
    syms q1 q2 a1 a2 real
    q = [q1 q2];
    dh = [0,0,a1,0,0;
        0,0,a2,0,0];
    R = SerialLink(dh);
    J = simplify(R.jacob0(q));
    fprintf('J =\n')
    disp(J)
    
    Jr = J(1:2,:);
    DJr = simplify(det(Jr));
    fprintf('det(J) = ')
    disp(DJr)
end
%=========================================================================%
fprintf('\n######################################################\n')
fprintf('#            Fin de Ejemplos para el TP6             #\n')
fprintf('######################################################\n')
%=========================================================================%