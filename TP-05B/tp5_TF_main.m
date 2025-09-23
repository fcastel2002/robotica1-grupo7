% 1. Cargar robot
run robot.m

% 2. Generar una pose alcanzable con CD
q_test = [10 -40 20 15 -25 60]*pi/180;  % config articular de prueba
T = R1.fkine(q_test); if isa(T,'SE3'), T = T.double; end

% 3. Supongamos que el robot está en q0 = [0 0 0 0 0 0]
q0 = zeros(6,1);

% 4. Cinemática inversa
Qall   = cin_inv_barista(R1, T, q0, false);   % todas
Qmejor = cin_inv_barista(R1, T, q0, true);    % más cercana

% 5. Mostrar resultados
disp('Todas las soluciones (cada columna es una):');
disp(Qall);

disp('Solución más cercana a q0:');
disp(Qmejor);

% 6. Verificar CD
Tcheck = R1.fkine(Qmejor); if isa(Tcheck,'SE3'), Tcheck=Tcheck.double; end
disp('Error de verificación ||T - Tcheck||:');
disp(norm(T - Tcheck));
