function [Xk, Pk] = filtro_kalman(Xk, Pk, Zk, Qk_1, Rk, dt,v,w)
    % Filtro de Kalman para estimar la posición y orientación del robot
    t1x = 4;
t1y = 8;
t2x = 1;
t2y = 1;
t3x = 11;
t3y = 3;
    % Predicción del estado
    X_k = [(Xk(1) + v * cos(Xk(3)) * dt);
           (Xk(2) + v * sin(Xk(3)) * dt);
           (Xk(3) + w * dt)];

    Ak = [1 0 -v * sin(Xk(3)) * dt;
          0 1  v * cos(Xk(3)) * dt;
          0 0  1];
          
    Bk = [cos(Xk(3)) * dt, 0;
          sin(Xk(3)) * dt, 0;
          0, dt];
          
    P_k = Ak * Pk * Ak' + Bk * Qk_1 * Bk';

    % Medición del sistema
    Zk_ = [atan2(t1y - X_k(2), t1x - X_k(1)) - X_k(3);
           atan2(t2y - X_k(2), t2x - X_k(1)) - X_k(3);
           atan2(t3y - X_k(2), t3x - X_k(1)) - X_k(3)];

    Hk = [((t1y - X_k(2)) / ((t1x - X_k(1))^2 + (t1y - X_k(2))^2)), (-(t1x - X_k(1)) / ((t1x - Xk(1))^2 + (t1y - X_k(2))^2)), -1;
          ((t2y - X_k(2)) / ((t2x - X_k(1))^2 + (t2y - X_k(2))^2)), (-(t2x - X_k(1)) / ((t2x - X_k(1))^2 + (t2y - X_k(2))^2)), -1;
          ((t3y - X_k(2)) / ((t3x - X_k(1))^2 + (t3y - X_k(2))^2)), (-(t3x - X_k(1)) / ((t3x - X_k(1))^2 + (t3y - X_k(2))^2)), -1];
    
    % Comparación
    Yk = Zk - Zk_;
    for r = 1:3
        if Yk(r) > pi
            Yk(r) = Yk(r) - 2 * pi;
        end
        if Yk(r) < -pi
            Yk(r) = Yk(r) + 2 * pi;
        end
    end

    % Actualización del filtro
    Sk = Hk * P_k * Hk' + Rk;
    Wk = P_k * Hk' * inv(Sk);

    % Corrección
    Xk = X_k + Wk * Yk;
    Pk = (eye(3) - Wk * Hk) * P_k;
end