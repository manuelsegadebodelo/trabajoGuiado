function [x_history, y_history, t_history, v_history, w_history, odometria_history, ultrasonic_history] = control_reactivo_kalman(pos_inicial, puntos_objetivo)
    % Controlador diferencial para un robot que sigue varios puntos objetivo con control reactivo.

    % Parámetros del robot
    v_max = 3.0;       % Velocidad lineal máxima (m/s)
    w_max = pi;        % Velocidad angular máxima (rad/s)
    v_min = 0.2;       % Velocidad lineal mínima (m/s)
    dt = 0.05;         % Tiempo de muestreo
    tolerance = 0.1;   % Tolerancia para considerar que se alcanzó el objetivo

    % Inicialización de variables
    x = pos_inicial(1);
    y = pos_inicial(2);
    theta = pos_inicial(3);
    integral_error_v = 0;
    integral_error_theta = 0;

    theta0 = 3*pi/4;
    

    % Historias para almacenar datos
    t_history = 0;
    x_history = x;
    y_history = y;
    v_history = 0;
    w_history = 0;
    odometry = [x, y, theta];
    odometria_history = odometry;
    ultrasonic_history = [0, 0, 0];

    % Cargar parámetros de Kalman
    load("calibracion/Q.mat");
    load("calibracion/R.mat");

    % Inicializamos la posición inicial y su covarianza
    pos0 = apoloGetLocationMRobot('Marvin');
    xini = pos0(1);
    yini = pos0(2);
    thetaini = pos0(4);
    Xk = [x; y; theta]; % Estimación inicial
    Pk = eye(3);  % Covarianza inicial (simplificada)

    % Iterar sobre todos los puntos objetivo
    iter=0;
    for i = 1:size(puntos_objetivo, 1)
        x_goal = puntos_objetivo(i, 1);
        y_goal = puntos_objetivo(i, 2);

        fprintf('Dirigiéndose al punto (%.2f, %.2f)\n', x_goal, y_goal);
        while true
        iter= iter +1 ;
        fprintf("Iteracion: %.2f",iter);
        if iter> 70
            break;
        end
            % Calcular errores y controlar el movimiento
            [error_x, error_y, rho, error_theta] = calcular_errores(x, y, theta, x_goal, y_goal);

            % Calcula la distancia a los obstaculos
            ultrasonics = apoloGetAllultrasonicSensors('Marvin');
            ultrasonic_history(end + 1, :) = ultrasonics;

            % Control reactivo para evitar obstáculos
            [v, w, integral_error_v, integral_error_theta] = control_reactivo(rho, error_theta, ultrasonics, v_max, w_max, v_min, dt, integral_error_v, integral_error_theta);

            % Mueve el robot en la simulación y actualiza su posición
            apoloMoveMRobot('Marvin', [v, w], dt);
            apoloUpdate();

            % Coger la odometría
            Uk = apoloGetOdometry('Marvin');
            odometria_history(end + 1, :) = Uk;
            
            % Llamada al filtro de Kalman para actualizar la estimación de la posición
            [Xk, Pk] = EKF(Xk, Pk, Uk, Q, R);
             
            x = Xk(1);
            y = Xk(2);
            theta = Xk(3);
            % Almacenar datos históricos
            [t_history, x_history, y_history, v_history, w_history] = registrar_datos(t_history, x_history, y_history, v_history, w_history, dt, x, y, v, w);

            % Mostrar estado actual
            fprintf('Posición: (%.2f, %.2f), Orientación: %.2f rad, Distancia al objetivo: %.2f\n', x, y, theta, rho);

            % Condición de parada
            if rho < tolerance
                fprintf('Objetivo alcanzado en (%.2f, %.2f)\n', x, y);
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
    % Calcula los errores de posición y orientación.
    error_x = x_goal - x;
    error_y = y_goal - y;
    rho = sqrt(error_x^2 + error_y^2);
    theta_goal = atan2(error_y, error_x);
    error_theta = mod(theta_goal - theta + pi, 2 * pi) - pi;
end

function [v, w, integral_error_v,integral_error_theta] = control_reactivo(rho, error_theta, ultrasonics, v_max, w_max, v_min, dt, integral_error_v,integral_error_theta)
    % Control reactivo basado en atractores y repulsores utilizando sensores láser.

    % Parámetros del controlador
    Kp_v = 4.0;        % Ganancia proporcional para la velocidad lineal
    Ki_v = 0.05;        % Ganancia integral para la velocidad lineal
    Kp_w = 5.0;        % Ganancia proporcional para la velocidad angular
    Ki_w = 0.02;       % Ganancia integral para la velocidad angular
    
    % Calcular repulsión de obstáculos
    [repulsion_x, repulsion_y] = calcular_repulsion(ultrasonics);

    % Control de colisión
    if repulsion_x > 1 && (ultrasonics(2) < 1 || ultrasonics(3) < 1)
        
        % Colisión inminente: detenerse y girar
        v = 0;

        % Decidir dirección de giro basada en las lecturas de los sensores laterales
        if ultrasonics(2) < ultrasonics(3)
            w = w_max/2;  % Girar a la derecha si el obstáculo está más cerca en el lado izquierdo
        else
            w = -w_max/2; % Girar a la izquierda si el obstáculo está más cerca en el lado derecho
        end
    else
        if repulsion_x > 1 || repulsion_y > 1
            % Si la repulsión es alta, reducir la velocidad lineal y ajustar la angular
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

function [repulsion_x, repulsion_y] = calcular_repulsion(ultrasonics)
    % Calcula el campo de repulsión basado en los 3 sensores láser.
    repulsion_x = 0;
    repulsion_y = 0;
    sensor_distances = [ultrasonics(1), ultrasonics(2), ultrasonics(3)];
    sensor_angles = [0, pi/4, -pi/4];

    for j = 1:length(sensor_distances)
        % Solo consideramos obstáculos cercanos (distancia menor a 1 metro)
        if sensor_distances(j) < 1
            strength = 1 / (sensor_distances(j)^2);
            repulsion_x = repulsion_x + strength * cos(sensor_angles(j));
            repulsion_y = repulsion_y + strength * sin(sensor_angles(j));
        end
    end
end

function [t_history, x_history, y_history, v_history, w_history] = registrar_datos(t_history, x_history, y_history, v_history, w_history, dt, x, y, v, w)
    % Registra datos históricos para su posterior análisis.
    t_history(end + 1) = t_history(end) + dt;
    x_history(end + 1) = x;
    y_history(end + 1) = y;
    v_history(end + 1) = v;
    w_history(end + 1) = w;
end