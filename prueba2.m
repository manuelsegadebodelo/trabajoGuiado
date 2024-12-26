% Inicializar la posición del robot en (0,0) con orientación 0 rad
pos_inicial = [0, 0, 0];

% Aqui meter el planificador para consegir los puntos objetivo.
% planificador()

% Definir una lista de puntos objetivo [x, y]
puntos_objetivo = [
    2, 2;
    -1,1
];

% Definir las orientaciones deseadas para cada punto objetivo (en radianes)
orientaciones_objetivo = [
    0;   % Orientación para el primer punto
%     pi/3;   % Orientación para el segundo punto
%     pi/2;  % Orientación para el tercer punto
];

% Colocar y resetear el robot en la simulación
apoloPlaceMRobot('Marvin', pos_inicial, 0);
apoloResetOdometry('Marvin');
apoloUpdate();

% Llamar a la función para seguir el camino con las orientaciones especificadas
[x_history, y_history, t_history, v_history, w_history, odometria_history, laser_history] = controlador_diferencial_reactivo(pos_inicial, puntos_objetivo);

graficar_trayectoria(t_history, x_history, y_history, v_history, w_history,laser_history, puntos_objetivo)
