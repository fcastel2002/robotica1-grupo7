% Ejemplos para el TP5

%=========================================================================%
clc, clear, close all
%=========================================================================%
fprintf('######################################################\n')
fprintf('#                Ejemplos para el TP5                #\n')
fprintf('######################################################\n\n')
fprintf('Solución para el ejercicio 4.\n')
fprintf('> Definición simbólica de matriz de parámetros DH\n')
fprintf('> Creación de objeto SerialLink simbólico\n')
fprintf('> Matrices de DH simbólicas\n')
fprintf('> Cinemática Directa simbólica\n')
fprintf('> Presentación de las 16 ecuaciones\n')
fprintf('> Trabajo algebraico para despeje de incógnitas\n')
%=========================================================================%
syms q1 q2 q3 a1 a2 a3 real
n = sym('n',[3,1],'real');
o = sym('o',[3,1],'real');
a = sym('A',[3,1],'real');
p = sym('p',[3,1],'real');

TD = [n o a p];
TD(4,:) = [0 0 0 1];

dh = [0,0,a1,0,1;
    0,0,a2,0,0;
    0,0,a3,0,0];

R = SerialLink(dh);
fprintf('Robot:\n')
R

fprintf('\n----------------------------------------------------\n')
fprintf('Presione Enter para continuar.\n')
pause

fprintf('Matriz 1 respecto a 0:\n')
T1 = R.links(1).A(q1).double;
T1
fprintf('\nMatriz 2 respecto a 1:\n')
T2 = R.links(2).A(q2).double;
T2
fprintf('\nMatriz 3 respecto a 2:\n')
T3 = R.links(3).A(q3).double;
T3

fprintf('\n----------------------------------------------------\n')
fprintf('Presione Enter para continuar.\n')
pause

fprintf('Matriz total (fkine):\n')
T03 = R.fkine([q1 q2 q3]).double;
T03

fprintf('\n----------------------------------------------------\n')
fprintf('Presione Enter para continuar.\n')
pause

fprintf('Matriz total (producto):\n')
T03 = T1 * T2 * T3;
T03

fprintf('\n----------------------------------------------------\n')
fprintf('Presione Enter para continuar.\n')
pause

fprintf('Matriz total (producto con simplify):\n')
T03 = simplify(T03);
T03

fprintf('\n----------------------------------------------------\n')
fprintf('Presione Enter para continuar.\n')
pause

fprintf('Matriz total como dato de entrada:\n')
fprintf('TD:\n')
TD

fprintf('\n----------------------------------------------------\n')
fprintf('Presione Enter para continuar.\n')
pause

fprintf('Ecuación principal, elemento a elemento:\n')
fprintf('TD = T1 * T2 * T3\n')
[(1:16)',TD(:), T03(:)]

fprintf('\n----------------------------------------------------\n')
fprintf('Presione Enter para continuar.\n')
pause

fprintf('Despejo T3 y comparo:\n')
fprintf('inv(T2) * inv(T1) * TD = T3:\n')
Ta1 = simplify(inv(T2) * inv(T1) * TD);
Ta2 = simplify(T3);
[(1:16)',Ta1(:), Ta2(:)]

fprintf('\n----------------------------------------------------\n')
fprintf('Presione Enter para continuar.\n')
pause

fprintf('> Ec 15:\n')
fprintf('q1 =\n')
ec15 = Ta1(15) == Ta2(15);
q1_sol = solve(ec15,q1);
q1_sol

fprintf('\n----------------------------------------------------\n')
fprintf('Presione Enter para continuar.\n')
pause

fprintf('> Ec 13 y 14: \n')
ec13 = Ta1(13) == Ta2(13);
ec14 = Ta1(14) == Ta2(14);
ec13
ec14
ec13 = isolate(ec13,'p1*cos(q2) - a1*cos(q2) + p2*sin(q2)');

fprintf('\n----------------------------------------------------\n')
fprintf('Presione Enter para continuar.\n')
pause

fprintf('\n> Ec 13 y 14 al cuadrado: \n')
ec13_2 = simplify(ec13^2);
ec14_2 = simplify(ec14^2);
ec13_2
ec14_2

fprintf('\n----------------------------------------------------\n')
fprintf('Presione Enter para continuar.\n')
pause

fprintf('\n> Sumadas: \n')
ec_suma = simplify(ec13_2 + ec14_2);
ec_suma

fprintf('\n----------------------------------------------------\n')
fprintf('Presione Enter para continuar.\n')
pause

fprintf('\n> Despejando cos(q3): \n')
q3_sol = isolate(ec_suma,cos(q3));
q3_sol
fprintf('\n> Nota: acos(x) tiene solución positiva y negativa\n')

fprintf('\n----------------------------------------------------\n')
fprintf('Presione Enter para continuar.\n')
pause

fprintf('Despejo T2 y comparo:\n')
fprintf('inv(T1) * T * inv(T3) = T2:\n')
Ta1 = simplify(inv(T1) * TD * inv(T3));
Ta2 = simplify(T2);
[(1:16)',Ta1(:), Ta2(:)]

fprintf('\n----------------------------------------------------\n')
fprintf('Presione Enter para continuar.\n')
pause

fprintf('> Ec 13: \n')
ec13 = Ta1(13) == Ta2(13);
ec13

fprintf('\n----------------------------------------------------\n')
fprintf('Presione Enter para continuar.\n')
pause

fprintf('\n> Despejando cos(q2): \n')
q2_sol = isolate(ec13,cos(q2));
q2_sol
fprintf('\n> Nota: acos(x) tiene solución positiva y negativa\n')

fprintf('\n----------------------------------------------------\n')
fprintf('Presione Enter para continuar.\n')
pause

fprintf('> Resumen:\n\n')
fprintf('\n')
q1_sol
fprintf('\n')
q2_sol
fprintf('\n')
q3_sol
fprintf('\n')
%=========================================================================%
fprintf('\n######################################################\n')
fprintf('#            Fin de Ejemplos para el TP5             #\n')
fprintf('######################################################\n')
%=========================================================================%