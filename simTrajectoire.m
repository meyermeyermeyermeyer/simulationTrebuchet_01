function simTrajectoire
%% - Simulation de la trajectoire du projectile

%% - Données
k_air = 0.02;                 % coefficient de frottement
a_air = @(v) -k_air * v;      % frottement
g = 9.81;

v0 = 30;                      % vitesse initiale
theta0 = 50 * pi/180;         % angle en radians
dt = 0.01;                    % pas de temps

% Décomposition de la vitesse initiale
vx = v0 * cos(theta0);
vy = v0 * sin(theta0);

% Position initiale
x = 0;
y = 0;

% Historique pour les graphiques
X_hist = x;
Y_hist = y;
T_hist = 0;

%% - Figures
figure('Name','Graphiques','NumberTitle','off');

subplot(2,1,1);
hTraj = plot(X_hist, Y_hist, 'b', 'LineWidth', 1.5);
xlabel('X'); ylabel('Y');
title('Trajectoire XY');
axis([0 100 0 40]);
grid on;
hold on;

subplot(2,1,2);
hX = plot(T_hist, X_hist, 'r', 'LineWidth', 1.5);
xlabel('Temps'); ylabel('X');
title('Position X en fonction du temps');
axis([0 10 0 100]);
grid on;
hold on;

%% - Timer
myTimer = timer('ExecutionMode','fixedRate','Period',dt, ...
    'TimerFcn',@tick);

start(myTimer);

%% - Fonction imbriquée : accès direct aux variables ci-dessus
    function tick(~,~)

        % Accélérations
        ax = a_air(vx);
        ay = a_air(vy) - g;

        % Mise à jour des vitesses
        vx = vx + ax * dt;
        vy = vy + ay * dt;

        % Mise à jour des positions
        x = x + vx * dt;
        y = y + vy * dt;

        % Stop si le projectile touche le sol
        if y < 0
            stop(myTimer);
            delete(myTimer);
            return;
        end

        % Mise à jour des historiques
        X_hist(end+1) = x;
        Y_hist(end+1) = y;
        T_hist(end+1) = T_hist(end) + dt;

        % Mise à jour des graphiques
        set(hTraj, 'XData', X_hist, 'YData', Y_hist);
        set(hX, 'XData', T_hist, 'YData', X_hist);

        drawnow limitrate;
    end

end
