
%definimos matriz DH

% dh = [tita d a alfa sigma]
dbstop if error;
dh = [ 0    0.290   0       -pi/2   0;
       0    0       0.270   0      0;
       0    0       0.070  -pi/2   0;
       0    0.302   0       pi/2   0;
       0    0       0      -pi/2   0;
       0    0.072   0       0      0 ];

scriptDir = fileparts(mfilename('fullpath'));
modelPath = fullfile(scriptDir, 'IRB120_STL_1');
modelPath2 = fullfile(scriptDir, 'IRB120_STL_2');

%assert(isfolder(modelPath), 'Carpeta IRB120_STL no encontrada: %s', modelPath);



%R1 = SerialLink(dh,'name','ABB IRB120 SC #1');
%R2 = SerialLink(dh, 'name', 'ABB IRB120 SC #2');

R1 = SerialLink(dh,'name','Leche');
R2 = SerialLink(dh, 'name', 'Cafe');


R1.qlim = deg2rad([ -165  165;
                    -110  110;
                    -110   70;
                    -160  160;
                    -120  120;
                    -400  400 ]);

R2.qlim = R1.qlim;
R1.base = transl(0.490+0.09,0,0)*trotz(pi); % la medida de separcion de 980mm divido/2 sumado el radio de la base.
R2.base = transl(-0.490-0.09,0.15,0);
R1.tool = transl(0,0,0.15);
R2.tool = transl(0,0,0.15);
R1.offset = [0, -pi/2, 0, 0, 0, 0];
R2.offset = R1.offset;
robots = {R1, R2};
limx = 2;
limy = 2;
limz = 1.3;
workspace = [-limx limx -limy limy 0 limz];
