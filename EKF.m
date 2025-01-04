function EKF[]
% Varianza del ruido del proceso 
Qk_1 = Q;

% Inicializamos la posición inicial y su covarianza
pos0 = apoloGetLocationMRobot('Marvin');
xini = pos0(1);
yini = pos0(2);
thetaini = pos0(4);
Xrealk = [xini; yini; thetaini];
Xk = [; 3; pi];

Pxini = 0.0;
Pyini = 0.0;
Pthetaini = 0.0;
Pk = [Pxini 0 0; 0 Pyini 0 ; 0 0 Pthetaini];

% Varianza en la medida
Rk = R;

% Posición de las balizas
t1x = 4;
t1y = 8;
t2x = 1;
t2y = 1;
t3x = 11;
t3y = 3;

% Algoritmo
Ktotal = zeros(3);   
for l = 1:length(trayectoriaD)

    % Observación de las balizas
    Zk = apoloGetLaserLandMarks();
          
    % Para acortar el nombre de la variable
    Uk = apoloGetOdometry('Marvin');
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

    % Predicción de la medida. Depende de la posicion de las balizas y de la
    %predicción del estado
    Zk_ = [(atan2(t1y-X_k(2),t1x-X_k(1)) - X_k(3));
          (atan2(t2y-X_k(2),t2x-X_k(1)) - X_k(3));
          (atan2(t3y-X_k(2),t3x-X_k(1)) - X_k(3))];
    %Matriz de observación
    Hk = [((t1y-X_k(2))/((t1x-X_k(1))^2+(t1y-X_k(2))^2)) (-(t1x-X_k(1))/((t1x-X_k(1))^2+(t1y-X_k(2))^2)) (-1);
          ((t2y-X_k(2))/((t2x-X_k(1))^2+(t2y-X_k(2))^2)) (-(t2x-X_k(1))/((t2x-X_k(1))^2+(t2y-X_k(2))^2)) (-1);   
          ((t3y-X_k(2))/((t3x-X_k(1))^2+(t3y-X_k(2))^2)) (-(t3x-X_k(1))/((t3x-X_k(1))^2+(t3y-X_k(2))^2)) (-1)];

    % Comparacion
    Yk = Zk-Zk_;
    for r=1:3
        if Yk(r)>pi
            Yk(r) = Yk(r) - 2*pi;
        end
        if Yk(r)<(-pi)
            Yk(r) = Yk(r) + 2*pi;
        end
    end
    Sk = Hk*P_k*((Hk)') + Rk;
    Wk = P_k*((Hk)')*inv(Sk);

    % Correccion
    Xk = X_k + Wk*Yk;
    Pk = (eye(3)-Wk*Hk)*P_k;
    
    %Sólo para almacenarlo
    Xestimado(:,l) = Xk;
    Pacumulado(1,l) = Pk(1,1);
    Pacumulado(2,l) = Pk(2,2);
    Pacumulado(3,l) = Pk(3,3);
end 