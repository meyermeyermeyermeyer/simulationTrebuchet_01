function simTrajectoire
%% - Simulation de la trajectoire du projectile

%% Initialisation
clear;clc;
%% - Données
% Données du drag
% Drag pour un ballon de basket 
rho = 1.2041; % Masse volumique de l'air à 20°C [kg/m3]
Cx = 0.45; % Coefficient de trainée d'un ballon
m = 0.6; % Masse du ballon [kg]
R = 0.12 ; % Rayon du ballon [m]
Surf = pi*R^2; % Surface de contact du ballon [m^2]

% Données de la simulation
g = 9.81;
v0 = 30;                      % vitesse initiale
theta0 = 50 * pi/180;         % angle en radians
dt = 0.01;                    % pas de temps
%% Algorithme
% Calcul du coefficient de trainé et du drag
k_air = 1/2*rho*Cx*Surf/m;
a_air = @(v) -k_air * v^2; 
% Décomposition de la vitesse initiale
vx = v0 * cos(theta0);
vy = v0 * sin(theta0);

% Position initiale
x = 0;
y = 2.5;

% Historique pour les graphiques
X_hist = x;
Y_hist = y;
T_hist = 0;

%% - Figures
figure('Name','Graphiques','NumberTitle','off');

subplot(2,1,1);
hTraj = plot(X_hist, Y_hist, '--b', 'LineWidth', 1.5);
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
