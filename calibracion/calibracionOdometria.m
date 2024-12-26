%% Resetear el robot
apoloPlaceMRobot('Marvin',[0 0 0], 0);
apoloResetOdometry('Marvin');
apoloUpdate();
%% Experimento para la medida de desplazamiento
Ed = [];
vx = 0.4;
t = 0.05;
x = vx*t;
for i = 1:2 %cambia la velocidad
    t = 0.1;
    for j = 1:5 %cambia el tiempo de desplazamiento
        x = vx*t;
        for k = 1:1000 %1000 experimentos de cada
            apoloMoveMRobot('Marvin', [vx, 0], t);
            apoloUpdate();
            pos=apoloGetOdometry('Marvin');
            error = x - pos(1);
            Ed = [Ed ; error];
            apoloPlaceMRobot('Marvin',[0 0 0], 0);
            apoloResetOdometry('Marvin');
            apoloUpdate();
        end
        t = t+0.4;
    end
    vx = vx*2;
end

%% Experimento para la medida de rotacion
Er = [];
vr = 0.6;
t = 0.05;
theta = vr*t;
for i = 1:2 %cambia la velocidad
    t = 0.1;
    for j = 1:5 %cambia el tiempo de rotacion
        theta = vr*t;
        if pi < theta && theta < 3* pi
            theta = theta - 2*pi;
        end
        if 3*pi < theta && theta < 5*pi
            theta = theta - 4*pi;
        end
        for k = 1:1000 %1000 experimentos de cada
            apoloMoveMRobot('Marvin', [0, vr], t);
            apoloUpdate();
            pos=apoloGetOdometry('Marvin');
            errorR = theta - pos(3);
            Er = [Er ; errorR];
            apoloPlaceMRobot('Marvin',[0 0 0], 0);
            apoloResetOdometry('Marvin');
            apoloUpdate();
        end
        t = t+0.1;
    end
    vr = vr*2;
end

%% Cálculo de las varianzas
E = [Ed Er];
Q = cov(E);
save('Q','Q')
