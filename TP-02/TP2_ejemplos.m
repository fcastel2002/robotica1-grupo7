% Ejemplos para el TP2

%=========================================================================%
clc, clear, close all
%=========================================================================%
fprintf('######################################################\n')
fprintf('#                Ejemplos para el TP2                #\n')
fprintf('######################################################\n\n')
fprintf('1: Uso de atan2.\n')
fprintf('2: Plot de Transformaciones Homogéneas\n')
fprintf('3: Usos de Transformaciones Homogéneas\n')
OPT = input('\nOpción: ');
%=========================================================================%
if OPT == 1
    fprintf('> Función atan(y/x) en los 4 cuadrantes:\n')
    fprintf('  atan( 1 /  1) = %d\n',atan(1/1) * 180/pi)
    fprintf('  atan(-1 /  1) = %d\n',atan(-1/1) * 180/pi)
    fprintf('  atan( 1 / -1) = %d\n',atan(1/-1) * 180/pi)
    fprintf('  atan(-1 / -1) = %d\n',atan(-1/-1) * 180/pi)
    fprintf('  (resultados en el 1º y 4º cuadrante)\n\n')
    
    fprintf('> Función atan2(y, x) en los 4 cuadrantes:\n')
    fprintf('  atan2( 1, 1) = %d\n',atan2(1,1) * 180/pi)
    fprintf('  atan2(-1, 1) = %d\n',atan2(-1,1) * 180/pi)
    fprintf('  atan2( 1,-1) = %d\n',atan2(1,-1) * 180/pi)
    fprintf('  atan2(-1,-1) = %d\n',atan2(-1,-1) * 180/pi)
    fprintf('  (resultados en los 4 cuadrantes)\n')
elseif OPT == 2
    fprintf('> Función trplot de RTB (ver ''help trplot''):\n')
    
    T0 = eye(4);
    T1 = trotz(pi/4);
    
    trplot(T0,'color','b','frame','0','length',1.8)
    hold on
    trplot(T1,'color','r','frame','1','length',1)
    
    grid on
    rotate3d on
    axis([-2 2 -2 2 -2 2])
    view(130,30)
elseif OPT == 3
    T0_2D_homog = eye(3)
    p_2D_0 = [1; 1]
    p_2D_0_homog = [p_2D_0; 1]
    T1_2D_homog = rotz(pi/6)
    p_2D_1_homog = T1_2D_homog \ p_2D_0_homog
    % p_2D_1_homog = inv(T1_2D_homo) * p_2D_0_homog
    p_2D_1 = p_2D_1_homog(1:2,1)
    
    
    trplot(T0_2D_homog,'color','b','frame','0','length',1.8)
    hold on
    trplot(T1_2D_homog,'color','r','frame','1','length',1.8)    
    grid on
    axis([-1.5 2 -.5 2 -2 2])
    view(0,90)
    
    % p_2D_0:
    plot(p_2D_0(1),p_2D_0(2),'*k','markersize',20,'LineWidth',1)
    plot([p_2D_0(1),p_2D_0(1)],[0,p_2D_0(2)],'--k')
    plot([0,p_2D_0(1)],[p_2D_0(2),p_2D_0(2)],'--k')
    
    fprintf('> Verificar en el gráfico que las coordenadas p_2D_1\n')
    fprintf('  sean las correctas en el sistema {1}.')
end
%=========================================================================%
fprintf('\n######################################################\n')
fprintf('#            Fin de Ejemplos para el TP2             #\n')
fprintf('######################################################\n')
%=========================================================================%