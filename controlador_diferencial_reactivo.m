function [x_history, y_history, t_history, v_history, w_history, odometria_history, laser_history] = controlador_diferencial_reactivo(pos_inicial, puntos_objetivo)
    % Controlador diferencial para un robot que sigue varios puntos objetivo con control reactivo.

    % Par�metros del robot
    v_max = 3.0;       % Velocidad lineal m�xima (m/s)
    w_max = pi;        % Velocidad angular m�xima (rad/s)
    v_min = 0.2;       % Velocidad lineal m�nima (m/s)
    dt = 0.05;         % Tiempo de muestreo
    tolerance = 0.1;   % Tolerancia para considerar que se alcanz� el objetivo

    % Inicializaci�n de variables
    x = pos_inicial(1);
    y = pos_inicial(2);
    theta = pos_inicial(3);
    integral_error_v = 0;
    integral_error_theta = 0;
    % Historias para almacenar datos
    t_history = 0;
    x_history = x;
    y_history = y;
    v_history = 0;
    w_history = 0;
    odometry = [y, x, theta];
    odometria_history = odometry;
    laser_history = [0, 0, 0];

    % Iterar sobre todos los puntos objetivo
    for i = 1:size(puntos_objetivo, 1)
        x_goal = puntos_objetivo(i, 1);
        y_goal = puntos_objetivo(i, 2);

        fprintf('Dirigi�ndose al punto (%.2f, %.2f)\n', x_goal, y_goal);
        while true
            % Calcular errores y controlar el movimiento
            [error_x, error_y, rho, error_theta] = calcular_errores(x, y, theta, x_goal, y_goal);
            laser_data = apoloGetAllultrasonicSensors('Marvin');
            laser_history(end + 1, :) = laser_data;

            % Control reactivo para evitar obst�culos
            [v, w, integral_error_v,integral_error_theta] = control_reactivo(rho, error_theta, laser_data, v_max, w_max, v_min, dt, integral_error_v,integral_error_theta);

            % Mueve el robot en la simulaci�n y actualiza su posici�n.
            apoloMoveMRobot('Marvin', [v, w], dt);
            apoloUpdate();

            % Coger la odometria
            %odometry = apoloGetOdometry('Marvin');
            % Actualizar la posici�n y orientaci�n del robot
            
            % variables de estado teoricas
            x = x + v * cos(theta) * dt;
            y = y + v * sin(theta) * dt;
            theta = theta + w * dt;
            odometria_history(end + 1, :) = [y,x,theta];
            
            % Coger odometria
            %x = odometry(2);
            %y = odometry(1);
            %theta = odometry(3);
            % Aqui sustituir x,y,z por los valores de la odometria y
            % acutalizar con el filtro de kalman
            
            %FIltro de KALMAN
            
            % Almacenar datos hist�ricos
            [t_history, x_history, y_history, v_history, w_history] = registrar_datos(t_history, x_history, y_history, v_history, w_history, dt, x, y, v, w);

            % Mostrar estado actual
            fprintf('Posici�n: (%.2f, %.2f), Orientaci�n: %.2f rad, Distancia al objetivo: %.2f\n', x, y, theta, rho);

            % Condici�n de parada
            if rho < tolerance
                fprintf('Objetivo alcanzado en (%.2f, %.2f)\n', x, y);
                % Ajustar la orientacion TODO
                break;
            end
            pause(dt);
        end
        % Pausa entre puntos
        pause(2);
    end
end

%% Funciones Auxiliares

function [error_x, error_y, rho, error_theta] = calcular_errores(x, y, theta, x_goal, y_goal)
    % Calcula los errores de posici�n y orientaci�n.
    error_x = x_goal - x;
    error_y = y_goal - y;
    rho = sqrt(error_x^2 + error_y^2);
    theta_goal = atan2(error_y, error_x);
    error_theta = mod(theta_goal - theta + pi, 2 * pi) - pi;
end

function [v, w, integral_error_v,integral_error_theta] = control_reactivo(rho, error_theta, laser_data, v_max, w_max, v_min, dt, integral_error_v,integral_error_theta)
    % Control reactivo basado en atractores y repulsores utilizando sensores l�ser.

    % Par�metros del controlador
    Kp_v = 4.0;        % Ganancia proporcional para la velocidad lineal
    Ki_v = 0.05;        % Ganancia integral para la velocidad lineal
    Kp_w = 5.0;        % Ganancia proporcional para la velocidad angular
    Ki_w = 0.02;       % Ganancia integral para la velocidad angular
    % Calcular repulsi�n de obst�culos
    [repulsion_x, repulsion_y] = calcular_repulsion(laser_data);

    % Control de colisi�n
    if repulsion_x < 1 && (laser_data(2) < 1 || laser_data(3) < 1)
        
        % Colisi�n inminente: detenerse y girar
        v = 0;

        % Decidir direcci�n de giro basada en las lecturas de los sensores laterales
        if laser_data(2) < laser_data(3)
            w = w_max;  % Girar a la derecha si el obst�culo est� m�s cerca en el lado izquierdo
        else
            w = -w_max; % Girar a la izquierda si el obst�culo est� m�s cerca en el lado derecho
        end
    else
        if repulsion_x > 1 || repulsion_y > 1
            % Si la repulsi�n es alta, reducir la velocidad lineal y ajustar la angular
            v = max(v_min, v_min - 0.5);
            w = max(min(w_max * (repulsion_x + repulsion_y), w_max), -w_max);
        else
            %% Control PI normal
            
            % Control PI para la velocidad lineal
            integral_error_v = integral_error_v + rho * dt;
            v = max(Kp_v * rho + Ki_v * integral_error_v, v_min);
            v = min(v, v_max);

            % Control PI para la velocidad angular
            integral_error_theta = integral_error_theta + error_theta * dt;
            w = max(min(Kp_w * error_theta + Ki_w * integral_error_theta, w_max), -w_max);
        end
    end
end

function [repulsion_x, repulsion_y] = calcular_repulsion(laser_data)
    % Calcula el campo de repulsi�n basado en los 3 sensores l�ser.
    repulsion_x = 0;
    repulsion_y = 0;
    sensor_distances = [laser_data(1), laser_data(2), laser_data(3)];
    sensor_angles = [0, pi/4, -pi/4];

    for j = 1:length(sensor_distances)
        % Solo consideramos obst�culos cercanos (distancia menor a 1 metro)
        if sensor_distances(j) < 1
            strength = 1 / (sensor_distances(j)^2);
            repulsion_x = repulsion_x + strength * cos(sensor_angles(j));
            repulsion_y = repulsion_y + strength * sin(sensor_angles(j));
        end
    end
end

function [t_history, x_history, y_history, v_history, w_history] = registrar_datos(t_history, x_history, y_history, v_history, w_history, dt, x, y, v, w)
    % Registra datos hist�ricos para su posterior an�lisis.
    t_history(end + 1) = t_history(end) + dt;
    x_history(end + 1) = x;
    y_history(end + 1) = y;
    v_history(end + 1) = v;
    w_history(end + 1) = w;
end





