function [Xk, Pk] = EKF(Xk, Pk, Q, R)
% Varianza del ruido del proceso 
Qk_1 = Q;

%N balizas y posición
landmarks = [-7.5 8; 8.6 8; -1 8; -6 0; 6.5 0; 8.5 -8.5; -2.5 -8.5];

% Algoritmo
    
    % Observación de las balizas
    tempZ = apoloGetLaserLandMarks('LMS100');
    Zk = zeros(2*length(tempZ.id), 1);
    Hk = zeros(2*length(tempZ.id), 3);
    Zk(1:2:end) = tempZ.angle;
    Zk(2:2:end) = tempZ.distance;

    Uk = (apoloGetOdometry('Marvin'))';
    apoloResetOdometry('Marvin');

    % Nuevo ciclo, k-1 = k.
    Xk_1 = Xk;
    Pk_1 = Pk;
    
    % Prediccion del estado
    X_k = [(Xk_1(1) + Uk(1)*cos(Xk_1(3)) - Uk(2)*sin(Xk_1(3))); % está mal creo
           (Xk_1(2) + Uk(1)*sin(Xk_1(3)) + Uk(2)*cos(Xk_1(3)));
           (Xk_1(3) + Uk(3))];

    Fk = [1 0 (-1)*Xk_1(1)*sin(Xk_1(3))-Xk_1(2)*cos(Xk_1(3)); %esta hay que cambiarla por el jacobiano de x
          0 1 Xk_1(1)*cos(Xk_1(3))-Xk_1(2)*sin(Xk_1(3));
          0 0 1];

    Bk = [cos(Xk_1(3)) (-1)*sin(Xk_1(3)) 0;
          sin(Xk_1(3)) cos(Xk_1(3)) 0;
          0 0 1];

    P_k = Fk*Pk_1*((Fk)') + Bk*Qk_1*((Bk)');

    % Predicción de la medida. Depende de la posición de las balizas y de la
    %predicción del estado
    for i = 1:length(tempZ.id)
        xL = landmarks(tempZ.id(i),1);
        yL = landmarks(tempZ.id(i),2);
        d = sqrt((xL-X_k(1))^2+(yL-X_k(2))^2);
        theta = atan2(yL-X_k(2), xL-X_k(1)) - X_k(3);

        Z = [d; theta];
        H(2*i-1) = [(X_k(1)-xL)/d (X_k(2)-yL)/d 0];
        H(2*i) = [(xL-X_k(1)/d^2) (yL-X_k(2))/d^2 -1];
       
        Zk_ = [Zk_; Z];
        Hk(i) = H_;
    end

    % Comparación
    Yk = Zk-Zk_;
    for r=1:length(tempZ.id) %por si el ángulo se pasa de rosca
        if Yk(2*r)>pi
            Yk(2*r) = Yk(2*r) - 2*pi;
        end
        if Yk(2*r)<(-pi)
            Yk(2*r) = Yk(2*r) + 2*pi;
        end
    end
    Rvalues = diag(R);
    Rk = diag(repmat(Rvalues, ceil(2*length(tempZ.id)/length(Rvalues)),1));
    Sk = Hk*P_k*((Hk)') + Rk;
    Wk = P_k*((Hk)')*inv(Sk);

    % Corrección
    Xk = X_k + Wk*Yk;
    Pk = (eye(3)-Wk*Hk)*P_k;
    
    %Sólo para almacenarlo
    % Xestimado(:,l) = Xk;
    % Pacumulado(1,l) = Pk(1,1);
    % Pacumulado(2,l) = Pk(2,2);
    % Pacumulado(3,l) = Pk(3,3);
% end 