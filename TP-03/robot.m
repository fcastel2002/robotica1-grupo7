%Trabajo practico N° 3: definición del robot

clc, clear, close all

%definimos matriz DH

% dh = [tita d a alfa sigma]

dh = [ 0    0.103   0      -pi/2   0;
       0    0       0.270   0      0;
       0    0       0.070  -pi/2   0;
       0    0.302   0       pi/2   0;
       0    0       0      -pi/2   0;
       0    0.072   0       0      0 ];

% dh = [  0.000 0.187 0.000 0.000 0;
%         0.000 0.103 0.000 -pi/2 0;
%         0.000 0.000 0.270 0.000 0; 
%         0.000 0.000 0.070 -pi/2 0;
%         0.000 0.302 0.000 0.000 0;
%         0.000 0.072 0.000 0.000 0];

R = SerialLink(dh,'name','ABB IRB120 SC');
q = [0,-pi/4,0,0,0,0];
R.qlim = deg2rad([ -165  165;
                   -90  50;
                   -90  70;
                   -160  160;
                   -60  120;
                   -120  120 ]);
limx = 0.8;
limy = 0.8;
limz = 1;
workspace = [-limx limx -limy limy -limz limz];

R.offset = [pi/2, 0, 0, 0, 0, 0];
T = R.fkine(q);
%R.plot(q,'scale',0.8,'workspace', workspace)
%disp(T.t);
R.teach(q)
