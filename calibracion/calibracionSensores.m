E=[];
y = -2;
for i = 1:5
    apoloPlaceMRobot('Marvin', [0 y 0], pi/2);
    apoloUpdate();
    d = sqrt((-3.90)^2+(3.90-y)^2);
    theta1 = atan2(3.90, 3.90-y+0.1);
    theta2 = -theta1;
    for j = 1:1000
        measure = apoloGetLaserLandMarks('LMS100');
        errorD1 = d-measure.distance(1); 
        errorA1 = theta1-measure.angle(1);
        errorD2 = d-measure.distance(2); 
        errorA2 = theta2-measure.angle(2);
        E = [E ; errorD1 errorA1; errorD2 errorA2];
    end
    y = y+1;
end
%% Sensores ultrasonidos
Eu=[];
theta = 0;
x = 1.5;
for i = 1:4
    for j = 1:5
        apoloPlaceMRobot('Marvin', [x 0 0], theta);
        apoloUpdate();
        d = 4 - x - 0.2;
        for k = 1:500
            measure = apoloGetUltrasonicSensor('uc0');
            error = d-measure;
            Eu = [Eu ; error];
        end
        x = x + 0.2;
    end
    theta = theta + 0.1;
    x = 1.5;
end
%% Cálculo de las varianzas
C = cov(E)
Cu = cov(Eu)