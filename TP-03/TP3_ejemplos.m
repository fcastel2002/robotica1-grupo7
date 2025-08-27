% Ejemplos para el TP3

%=========================================================================%
clc, clear, close all
%=========================================================================%
fprintf('######################################################\n')
fprintf('#                Ejemplos para el TP3                #\n')
fprintf('######################################################\n\n')
fprintf('Ejemplo para ABB IRB910 SC\n')
fprintf('> definición de matriz de parámetros DH\n')
fprintf('> creación de objeto SerialLink con nombre\n')
fprintf('> vector inicial\n')
fprintf('> límites articulares\n')
fprintf('> matriz de base\n')
fprintf('> matriz de herramienta\n')
fprintf('> offsets articulares\n')
fprintf('> plot\n')
fprintf('> teach\n')
%=========================================================================%
dh = [
    0.000  0.195  0.300  0.000 0;
    0.000  0.000  0.250  pi    0;
    0.000  0.000  0.000  0.000 1;
    0.000  0.000  0.000  0.000 0];

R = SerialLink(dh,'name','ABB IRB910 SC');
q = [0,0,0,0];

R.qlim(1,1:2) = [-140,  140]*pi/180;
R.qlim(2,1:2) = [-150,  150]*pi/180;
R.qlim(3,1:2) = [0, 0.18];
R.qlim(4,1:2) = [-400,  400]*pi/180;

%R.base = transl(-0.5,0,0); % * trotx(pi);
%R.tool = transl(.2, .2, 0);
R.offset = [pi/4 0 0 0];

% R.plot(q,'workspace',[-1 1 -1 1 -1 1])
R.plot(q,'scale',0.8,'trail',{'r', 'LineWidth', 2})
R.teach()
%=========================================================================%
fprintf('\n######################################################\n')
fprintf('#            Fin de Ejemplos para el TP3             #\n')
fprintf('######################################################\n')
%=========================================================================%