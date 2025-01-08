function path = advanced_a_star(map, start, goal, connecting_distance)
    % Advanced A* Algorithm adapted to the current setup
    % map: Binary occupancy grid (0 = free, 1 = obstacle)
    % start: [row, col] starting point
    % goal: [row, col] goal point
    % connecting_distance: Distance to connect neighbors (default = 1)

    % Verificar si las posiciones inicial y final son válidas
    if map(start(1), start(2)) == 1 || map(goal(1), goal(2)) == 1
        error('Start or goal point is inside an obstacle!');
    end

    % Dimensiones del mapa
    [height, width] = size(map);

    % Inicialización de matrices
    g_score = zeros(height, width);            % Costos desde el inicio a cada celda
    f_score = inf(height, width, 'single');   % Costos estimados (g_score + heurística)
    h_score = zeros(height, width, 'single'); % Heurística (distancia al objetivo)
    open_set = int8(zeros(height, width));    % Lista de nodos abiertos
    closed_set = int8(zeros(height, width));  % Lista de nodos cerrados
    closed_set(map == 1) = 1;                 % Obstáculos en lista cerrada
    parent_x = zeros(height, width, 'int16'); % Nodo padre (x)
    parent_y = zeros(height, width, 'int16'); % Nodo padre (y)

    % Generar matriz de vecinos según la distancia de conexión
    neighbor_mask = ones(2 * connecting_distance + 1);
    neighbor_mask(connecting_distance + 1, connecting_distance + 1) = 0; % No incluir el nodo actual
    [neighbor_row, neighbor_col] = find(neighbor_mask == 1);
    neighbors = [neighbor_row - connecting_distance - 1, ...
                 neighbor_col - connecting_distance - 1];

    % Precalcular matriz de heurística (distancia Euclidiana al objetivo)
    for i = 1:height
        for j = 1:width
            if map(i, j) == 0
                h_score(i, j) = sqrt((i - goal(1))^2 + (j - goal(2))^2);
            end
        end
    end

    % Inicializar nodo inicial
    f_score(start(1), start(2)) = h_score(start(1), start(2));
    open_set(start(1), start(2)) = 1;

    % Bucle principal de A*
    while any(open_set(:))
        % Seleccionar el nodo con el menor f_score
        [min_f, idx] = min(f_score(:));
        if min_f == inf
            disp('No path found.');
            path = [];
            visualize_path(map, path, start, goal);
            return;
        end

        [current_y, current_x] = ind2sub(size(f_score), idx);

        % Comprobar si llegamos al objetivo
        if current_y == goal(1) && current_x == goal(2)
            path = reconstruct_advanced_path(parent_x, parent_y, current_x, current_y, start);
            visualize_path(map, path, start, goal);
            return;
        end

        % Mover nodo actual de abierto a cerrado
        open_set(current_y, current_x) = 0;
        f_score(current_y, current_x) = inf; % Marcar como procesado
        closed_set(current_y, current_x) = 1;

        % Explorar vecinos
        for i = 1:size(neighbors, 1)
            ny = current_y + neighbors(i, 1);
            nx = current_x + neighbors(i, 2);

            % Comprobar si el vecino está dentro del mapa y es válido
            if ny < 1 || ny > height || nx < 1 || nx > width || closed_set(ny, nx) == 1
                continue;
            end

            % Evitar que el camino pase por un obstáculo
            if map(ny, nx) == 1
                continue;
            end

            % Calcular g_score tentativo
            tentative_g = g_score(current_y, current_x) + sqrt(neighbors(i, 1)^2 + neighbors(i, 2)^2);

            if open_set(ny, nx) == 0 || tentative_g < g_score(ny, nx)
                % Actualizar nodo si es mejor camino
                parent_x(ny, nx) = current_x;
                parent_y(ny, nx) = current_y;
                g_score(ny, nx) = tentative_g;
                f_score(ny, nx) = tentative_g + h_score(ny, nx);

                % Añadir nodo al conjunto abierto
                open_set(ny, nx) = 1;
            end
        end
    end

    % Si no se encontró un camino
    disp('No path found.');
    path = [];
    visualize_path(map, path, start, goal);
end

function path = reconstruct_advanced_path(parent_x, parent_y, current_x, current_y, start)
    % Reconstruir el camino desde el objetivo hasta el inicio
    path = [current_y, current_x];
    while ~(current_x == start(2) && current_y == start(1))
        temp_x = parent_x(current_y, current_x);
        current_y = parent_y(current_y, current_x);
        current_x = temp_x;
        path = [current_y, current_x; path];
    end
end
