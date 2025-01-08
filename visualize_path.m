function visualize_path(map, path, start, goal)
    % Visualizar el mapa, los obstáculos, las celdas libres y la trayectoria
    figure;
    colormap(gray); % Usar escala de grises para mapa binario
    imagesc(~map); % Mostrar el mapa (negro para obstáculos, blanco para libres)
    hold on;

    % Resaltar el punto inicial (en azul)
    plot(start(2), start(1), 'bo', 'MarkerSize', 10, 'LineWidth', 2);

    % Resaltar el punto final (en rojo)
    plot(goal(2), goal(1), 'ro', 'MarkerSize', 10, 'LineWidth', 2);

    % Si hay un camino, dibujarlo en verde
    if ~isempty(path)
        plot(path(:, 2), path(:, 1), 'g-', 'LineWidth', 2);
    end

    % Configuración de la gráfica
    axis equal;
    axis tight;
    grid on;
    title('Mapa y Trayectoria Calculada');
    hold off;
end
