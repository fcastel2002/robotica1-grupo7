%=====================================================
%               ARCHIVO DE DEFINICIÓN DEL ROBOT
%=====================================================
% Trabajo practico N° 3: definición del robot
clc; clear; close all;

% Matriz DH para el ABB IRB 120
% Formato: [theta, d, a, alpha, sigma] (sigma=0 para rotacional)
dh = [ 0    0.290   0      -pi/2   0;
       0    0       0.270   0      0;
       0    0       0.070  -pi/2   0;
       0    0.302   0       pi/2   0;
       0    0       0      -pi/2   0;
       0    0.072   0       0      0 ];

% Creación de los objetos SerialLink para los dos robots
R1 = SerialLink(dh,'name','Robot Barista 1');
R2 = SerialLink(dh,'name','Robot Barista 2');

% Límites articulares en radianes
R1.qlim = deg2rad([ -165  165;
                    -110  110;
                    -110   70;
                    -160  160;
                    -120  120;
                    -400  400 ]);
R2.qlim = R1.qlim;

% Desfase (offset) para corregir la posición cero de las articulaciones
R1.offset = [0, -pi/2, 0, 0, 0, 0];
R2.offset = R1.offset;

% Límites del área de graficación [Xmin Xmax Ymin Ymax Zmin Zmax]
limx = 1.0; limy = 1.0; limz = 1.2;
ws_limites = [-limx, limx, -limy, limy, 0, limz];
