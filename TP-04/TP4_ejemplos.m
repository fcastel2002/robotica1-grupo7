% Ejemplos para el TP4

%=========================================================================%
clc, clear, close all
%=========================================================================%
fprintf('######################################################\n')
fprintf('#                Ejemplos para el TP4                #\n')
fprintf('######################################################\n\n')
fprintf('Verificaciones para el ejercicio 1.\n')
fprintf('> solución mediante SerialLink.fkine(), el ejercicio \n')
fprintf('  pide hacer uso de transformaciones elementales.\n')
fprintf('> declaración de vectores articulares\n')
fprintf('> subplot con cada posición articular y con ejes de \n')
fprintf('  referencia\n')
fprintf('> cinemática directa mediante fkine\n')
fprintf('> print de datos formateados\n')
fprintf('> print de cinemática directa\n\n')
%=========================================================================%
disp('FANUC Paint Mate 200iA')
%=========================================================================%
dh = [
    0      0.45   0.075 -pi/2  0;
    0      0      0.3    0     0;
    0      0      0.075 -pi/2  0;
    0      0.32   0      pi/2  0;
    0      0      0     -pi/2  0;
    0      0.008  0      0     0];
R = SerialLink(dh);
%=========================================================================%
q1 = [0,0,0,0,0,0];
q2 = [pi/4,-pi/2,0,0,0,0];
q3 = [pi/5,-2*pi/5,-pi/10,pi/2,3*pi/10,-pi/2];
q4 = [-.61 -.15 -.3 1.4 1.9 -1.4];
%=========================================================================%
R.name = 'q1';
subplot(2,2,1),R.plot(q1),campos([10 15 10])
hold on,trplot(diag(ones(4,1)),'length',1.5,'frame','0')
%=========================================================================%
R2 = SerialLink(R,'name','q2');
subplot(2,2,2),R2.plot(q2),campos([10 15 10])
hold on,trplot(diag(ones(4,1)),'length',1.5,'frame','0')
%=========================================================================%
R3 = SerialLink(R,'name','q3');
subplot(2,2,3),R3.plot(q3),campos([10 15 10])
hold on,trplot(diag(ones(4,1)),'length',1.5,'frame','0')
%=========================================================================%
R4 = SerialLink(R,'name','q4');
subplot(2,2,4),R4.plot(q4),campos([10 15 10])
hold on,trplot(diag(ones(4,1)),'length',1.5,'frame','0')
%=========================================================================%
fprintf('\nq1 = [%6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f]\n',q1)
fprintf('T = \n')
T = R.fkine(q1);
disp(T)
fprintf('\nq2 = [%6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f]\n',q2)
fprintf('T = \n')
T = R.fkine(q2);
disp(T)
fprintf('\nq3 = [%6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f]\n',q3)
fprintf('T = \n')
T = R.fkine(q3);
disp(T)
fprintf('\nq4 = [%6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f]\n',q4)
fprintf('T = \n')
T = R.fkine(q4);
disp(T)
%=========================================================================%
fprintf('\n######################################################\n')
fprintf('#            Fin de Ejemplos para el TP4             #\n')
fprintf('######################################################\n')
%=========================================================================%