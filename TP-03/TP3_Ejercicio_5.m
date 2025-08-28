%% TP – Ejercicio 5 ( matrices DH)
% Formato DH por fila: [theta  d      a      alpha   sigma]
% sigma = 0 (revoluta), 1 (prismática). Unidades: metros y rad.

close all; clc;

%% ===================== 1) SCARA ABB IRB 910SC (R-R-P-R) =====================
%              th     d       a      al    sg
dh_scara = [    0   0.262   0.20     pi     0;   % eje 1
                0   0       0.25     0      0;   % eje 2
                0   0       0        0      1;   % eje 3 (P)
                0   0       0        0      0];  % eje 4
qlim_scara = [deg2rad([-170 170]);
              deg2rad([-170 170]);
              0 0.20;                    % carrera 0–20 cm
              deg2rad([-360 360])];

SCARA = make_robot_from_dh('ABB\_IRB910SC', dh_scara, qlim_scara);
figure('Name','SCARA ABB IRB 910SC');
SCARA.plot([0 0 0.10 0], 'workspace', [-0.7 0.7 -0.7 0.7 -0.1 0.6], 'scale', 0.8);
SCARA.teach('callback', []);

%% ================= 2) FANUC Paint Mate 200iA (6R compacto) ==================
%               th     d       a       al     sg
dh_fanuc = [     0   0.330   0.000    pi/2     0;   % 1
                 0   0.000   0.250    0        0;   % 2
                 0   0.000   0.100   -pi/2     0;   % 3
                 0   0.300   0.000    pi/2     0;   % 4
                 0   0.000   0.000   -pi/2     0;   % 5
                 0   0.080   0.000    0        0];  % 6
qlim_fanuc = [deg2rad([-170 170]);
              deg2rad([-120 120]);
              deg2rad([-155 155]);
              deg2rad([-185 185]);
              deg2rad([-120 120]);
              deg2rad([-360 360])];

PAINTMATE = make_robot_from_dh('FANUC\_PaintMate200iA', dh_fanuc, qlim_fanuc);
figure('Name','FANUC Paint Mate 200iA');
PAINTMATE.plot(zeros(1,6), 'workspace', [-0.8 0.8 -0.8 0.8 -0.1 1.0], 'scale', 0.8);
PAINTMATE.teach('callback', []);

%% =================== 3) KUKA LBR iiwa 7 R800 (7R cobot) =====================
%               th     d       a       al     sg
dh_iiwa = [      0   0.157   0.000    pi/2     0;  % 1
                 0   0.000   0.200   -pi/2     0;  % 2
                 0   0.000   0.200   -pi/2     0;  % 3
                 0   0.000   0.080    pi/2     0;  % 4
                 0   0.000   0.120    pi/2     0;  % 5
                 0   0.000   0.040   -pi/2     0;  % 6
                 0   0.126   0.000    0        0]; % 7
qlim_iiwa = [deg2rad([-170 170]);
             deg2rad([-120 120]);
             deg2rad([-170 170]);
             deg2rad([-120 120]);
             deg2rad([-170 170]);
             deg2rad([-120 120]);
             deg2rad([-360 360])];

IIWA = make_robot_from_dh('KUKA\_LBR\_iiwa7\_R800', dh_iiwa, qlim_iiwa);
figure('Name','KUKA LBR iiwa 7 R800');
IIWA.plot(zeros(1,7), 'workspace', [-0.9 0.9 -0.9 0.9 -0.2 1.2], 'scale', 0.8);
IIWA.teach('callback', []);

%% ========================== Función auxiliar ==========================
% Construye un SerialLink desde una matriz DH estándar.
% dh: Nx5  [theta d a alpha sigma]; qlims opcional Nx2 en rad o m.
function R = make_robot_from_dh(name, dh, qlims)
    L(1:size(dh,1)) = Link; % prealocar
    for i = 1:size(dh,1)
        L(i) = Link(dh(i,:), 'standard');   % DH estándar
        if nargin >= 3 && ~isempty(qlims)
            L(i).qlim = qlims(i,:);
        end
    end
    R = SerialLink(L, 'name', name);
end
