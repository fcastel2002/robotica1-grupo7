%=====================================================
%               ARCHIVO DE DEFINICIÓN DEL ROBOT
%=====================================================
% Trabajo practico N° 3: definición del robot
clc; clear; close all;

% Matriz DH para el ABB IRB 120
% Formato: [theta, d, a, alpha, sigma] (sigma=0 para rotacional)
=======
%Trabajo practico N° 3: definición del robot

%definimos matriz DH

% dh = [tita d a alfa sigma]
% 1. tita_i: Ángulo alrededor del eje Z_i-1 , desde el eje X_i-1 hasta el eje Z_i.
% 2. d_i: Distancia a lo largo del eje Z_i-1, desde el origen del sistema i-1 hasta el eje X_i.
% 3. a_i: Distancia a lo largo del eje X_i, desde el eje Z_i-1 hasta el eje Z_i.
% 4. alfa_i: Ángulo alrededor del eje X_i, desde el eje Z_i-1 hasta el eje Z_i.

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


R1 = SerialLink(dh,'name','ABB IRB120 SC #1');
R2 = SerialLink(dh, 'name', 'ABB IRB120 SC #2');
R1.qlim = deg2rad([ -165  165;
                   -110  110;
                   -110  70;
                   -160  160;
                   -120  120;
                   -400  400 ]);
R2.qlim = R1.qlim;
R1.base = transl(0.490+0.09,0,0)*trotz(pi); % la medida de separcion de 980mm divido/2 sumado el radio de la base.
R2.base = transl(-0.490-0.09,0,0);
R1.tool = transl(0,0,0.15);
R2.tool = transl(0,0,0.15);
R1.offset = [0, -pi/2, 0, 0, 0, 0];
R2.offset = R1.offset;
robots = {R1, R2};
limx = 1.3;
limy = 1.3;
limz = 1.3;
workspace = [-limx limx -limy limy 0 limz];

