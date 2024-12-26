function graficar_trayectoria(t_history, x_history, y_history, v_history, w_history, laser_history, puntos_objetivo)
    % Función para graficar la trayectoria, velocidades y lecturas láser del robot.
    %
    % Parámetros:
    % - t_history: Vector con los tiempos registrados.
    % - x_history: Vector con las posiciones X registradas.
    % - y_history: Vector con las posiciones Y registradas.
    % - v_history: Vector con las velocidades lineales registradas.
    % - w_history: Vector con las velocidades angulares registradas.
    % - laser_history: Matriz con las lecturas de los sensores láser registradas.
    % - puntos_objetivo: Matriz con los puntos objetivo [x_goal, y_goal].

    figure;

    % Trayectoria en X
    subplot(3, 1, 1);
    plot(t_history, x_history, 'b');
    xlabel('Tiempo (s)');
    ylabel('Posición X (m)');
    title('Trayectoria en X');
    grid on;

    % Trayectoria en Y
    subplot(3, 1, 2);
    plot(t_history, y_history, 'r');
    xlabel('Tiempo (s)');
    ylabel('Posición Y (m)');
    title('Trayectoria en Y');
    grid on;

    % Velocidades Lineal y Angular
    subplot(3, 1, 3);
    plot(t_history, v_history, 'g', 'LineWidth', 1.5);
    hold on;
    plot(t_history, w_history, 'm', 'LineWidth', 1.5);
    xlabel('Tiempo (s)');
    ylabel('Velocidad');
    title('Velocidades Lineal y Angular');
    legend('Velocidad Lineal (v)', 'Velocidad Angular (w)');
    grid on;

    figure;
    % Lecturas de los Sensores Láser
    subplot(1, 1, 1);
    plot(t_history, laser_history(:,1),t_history,laser_history(:,2),t_history,laser_history(:,3));
    xlabel('Tiempo (s)');
    ylabel('Distancia Láser (m)');
    title('Lecturas de los Sensores Láser');
    legend('u0', 'ul1', 'ul2');
    grid on;
    


    % Figura de la trayectoria en el plano XY
    figure;
    plot(x_history, y_history, 'b.-');
    hold on;
    plot(puntos_objetivo(:, 1), puntos_objetivo(:, 2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
    xlabel('Posición X (m)');
    ylabel('Posición Y (m)');
    title('Trayectoria del Robot en el Plano XY');
    legend('Trayectoria', 'Puntos objetivo');
    grid on;
end
