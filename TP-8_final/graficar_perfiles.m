function graficar_perfiles(t, q, qd, tipo_movimiento)
% Grafica los perfiles de posición y velocidad articular.
% Entradas:
%   t               -> vector de tiempo o de muestras
%   q               -> matriz de posiciones articulares (MxN)
%   qd              -> matriz de velocidades articulares (MxN)
%   tipo_movimiento -> string para el título del gráfico

% Usamos una figura dedicada para no interferir con la animación
figure(20);
clf; % Limpia la figura antes de dibujar nuevos perfiles

% --- Gráfico de Posición Articular ---
subplot(2,1,1);
plot(t, rad2deg(q));
title(['Perfil de Posición (' tipo_movimiento ')']);
xlabel('Muestras');
ylabel('Ángulo (deg)');
grid on;
legend('q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'Location', 'eastoutside');

% --- Gráfico de Velocidad Articular ---
subplot(2,1,2);
plot(t, qd);
title(['Perfil de Velocidad (' tipo_movimiento ')']);
xlabel('Muestras');
ylabel('Velocidad (rad/s)');
grid on;
legend('qd1', 'qd2', 'qd3', 'qd4', 'qd5', 'qd6', 'Location', 'eastoutside');

drawnow; % Asegura que los gráficos se actualicen inmediatamente
end