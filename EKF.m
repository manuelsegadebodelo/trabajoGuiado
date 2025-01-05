% Varianza del ruido del proceso 
% Qk_1 = Q;

%N balizas y posición
% landmarks = [1 0; 2 3];
% nLandmarks = size(landmarks,1);

% Hk = zeros(nLandmarks, 3);
% Zk = zeros(2*nLandmarks, 1);
% Inicializamos la posición inicial y su covarianza
% pos0 = apoloGetLocationMRobot('Marvin');
% xini = pos0(1);
% yini = pos0(2);
% thetaini = pos0(4);
% Xk = [xini; yini; thetaini];

% Pxini = 0.0;
% Pyini = 0.0;
% Pthetaini = 0.0;
% Pk = [Pxini 0 0; 0 Pyini 0 ; 0 0 Pthetaini];

% Varianza en la medida
% Rk = R;

% Algoritmo
%Ktotal = zeros(3);   
% for l = 1:length(trayectoriaD)

    angle_col = zeros(nLandmarks, 1);
    distance_col = zeros(nLandmarks, 1);

    % Observación de las balizas
    tempZ = apoloGetLaserLandMarks('LMS100');
    angle_col(tempZ.id) = tempZ.angle';
    distance_col(tempZ.id) = tempZ.distance';
    Zk(1:2:end) = distance_col;
    Zk(2:2:end) = angle_col;
    
    % Para acortar el nombre de la variable
    apoloResetOdometry('Marvin');
    % Nuevo ciclo, k-1 = k.
    Xk_1 = Xk;
    Pk_1 = Pk;
    
    % Prediccion del estado
    X_k = [(Xk_1(1) + Uk(1));
           (Xk_1(2) + Uk(2));
           (Xk_1(3) + Uk(3))];

    Ak = [1 0 0;
          0 1 0;
          0 0 1];
    Bk = [1 0 0;
          0 1 0;
          0 0 1];

    P_k = Ak*Pk_1*((Ak)') + Bk*Qk_1*((Bk)');

    % Predicción de la medida. Depende de la posición de las balizas y de la
    %predicción del estado
    for i = 1:nLandmarks
        xL = landmarks(i,1);
        yL = landmarks(i,2);
        d = sqrt((xL-X_k(1))^2+(yL-X_k(2))^2);
        theta = atan2(yL-X_k(2), xL-X_k(1)) - X_k(3);

        if abs(theta) <= theta0
            Z = [d; theta];
            H(2*i-1) = [(X_k(1)-xL)/d (X_k(2)-yL)/d 0];
            H(2*i) = [(xL-X_k(1)/d^2) (yL-X_k(2))/d^2 -1];
        else
            Z = [0; 0];
            H(2*i-1) = [0 0 0];
            H(2*i) = [0 0 0];
        end
        Zk_ = [Zk_; Z];
        Hk(i) = H_;
    end
    % Zk_ = [(atan2(t1y-X_k(2),t1x-X_k(1)) - X_k(3));
    %       (atan2(t2y-X_k(2),t2x-X_k(1)) - X_k(3));
    %       (atan2(t3y-X_k(2),t3x-X_k(1)) - X_k(3))];
    %Matriz de observación
    % Hk = [((t1y-X_k(2))/((t1x-X_k(1))^2+(t1y-X_k(2))^2)) (-(t1x-X_k(1))/((t1x-X_k(1))^2+(t1y-X_k(2))^2)) (-1);
    %       ((t2y-X_k(2))/((t2x-X_k(1))^2+(t2y-X_k(2))^2)) (-(t2x-X_k(1))/((t2x-X_k(1))^2+(t2y-X_k(2))^2)) (-1);   
    %       ((t3y-X_k(2))/((t3x-X_k(1))^2+(t3y-X_k(2))^2)) (-(t3x-X_k(1))/((t3x-X_k(1))^2+(t3y-X_k(2))^2)) (-1)];

    % Comparación
    Yk = Zk-Zk_;
    for r=1:nLandmarks %por si el ángulo se pasa de rosca
        if Yk(2*r)>pi
            Yk(2*r) = Yk(2*r) - 2*pi;
        end
        if Yk(2*r)<(-pi)
            Yk(2*r) = Yk(2*r) + 2*pi;
        end
    end

    Sk = Hk*P_k*((Hk)') + Rk;
    Wk = P_k*((Hk)')*inv(Sk);

    % Corrección
    Xk = X_k + Wk*Yk;
    Pk = (eye(3)-Wk*Hk)*P_k;
    
    %Sólo para almacenarlo
    Xestimado(:,l) = Xk;
    Pacumulado(1,l) = Pk(1,1);
    Pacumulado(2,l) = Pk(2,2);
    Pacumulado(3,l) = Pk(3,3);
% end 