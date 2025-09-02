%% == FANUC == %%
DH = [ 0    0.45    0.075      -pi/2   0;
       0    0       0.3   0      0;
       0    0       0.075  -pi/2   0;
       0    0.32    0       pi/2   0;
       0    0       0      -pi/2   0;
       0    0.008   0       0      0 ];
R = SerialLink(DH);
Q = [0, 0, 0, 0, 0, 0;
     pi/4, -pi/2, 0 ,0 ,0 ,0;
    pi/5,-2*pi/5,-pi/10,pi/2,3*pi/10,-pi/2;
    -0.61, -0.15, -0.3, 1.4,1.9,-1.4];
disp("Paint Mate FANUC Ej1")
for k=1:size(Q,1)

    fprintf('Configuracion %d\n',k);
    T = R.fkine(Q(k,:));
    disp(T);

end

%% == SCARA TP4 == %%
% DH = [
% 0.000 0.195 0.300 0.000 0;
% 0.000 0.000 0.250 0.000 0;
% 0.000 0.000 0.000 pi 1;
% 0.000 0.000 0.000 0.000 0];
% R = SerialLink(DH);
% q = [0,0,0,0];
% T = R.fkine(q);
% disp("SCARA Ej2")
% disp(T)
%% == SCARA TP3 == %%
% 
% dh_scara = [    0   0.262   0.300     pi     0;   % eje 1
%                 0   0       0.250     0      0;   % eje 2
%                 0   0       0        0      1;   % eje 3 (P)
%                 0   0       0        0      0];  % eje 4
% R1 = SerialLink(dh_scara);
% q = [0,0,0,0];
% T = R1.fkine(q);
% disp("Scara EJ5 tp3");
% disp(T)