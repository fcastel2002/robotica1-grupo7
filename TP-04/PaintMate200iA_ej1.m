%Tp 4: Cinematica directa %
%definimos matriz DH

% dh = [tita d a alfa sigma]
clc, close all;
dh = [ 0    0.45    0.075      -pi/2   0;
       0    0       0.3   0      0;
       0    0       0.075  -pi/2   0;
       0    0.32    0       pi/2   0;
       0    0       0      -pi/2   0;
       0    0.008   0       0      0 ];


function A = A_dh(theta, d, a, alpha)
    A = trotz(theta)*transl(0,0,d)*transl(a,0,0)*trotx(alpha);
end
Q = [0, 0, 0, 0, 0, 0;
     pi/4, -pi/2, 0 ,0 ,0 ,0;
    pi/5,-2*pi/5,-pi/10,pi/2,3*pi/10,-pi/2;
    -0.61, -0.15, -0.3, 1.4,1.9,-1.4];

for k = 1:size(Q,1)
    T = eye(4);
    for i=1:6
        d = dh(i,2);
        a = dh(i,3);
        alpha = dh(i,4);
        T = T*A_dh(Q(k,i),d,a,alpha);
    end
    fprintf('Configuracion %d\r\n', k);
    dist_base = sqrt(T(1,4)^2+T(2,4)^2+T(3,4)^2);
    fprintf('distancia a la base:%f\r',dist_base)
    disp(T);
end
