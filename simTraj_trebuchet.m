function simtrajectoire()
%% Simulation animée de la trajectoire d'un projectile avec traînée

clc; close all;

%% Données physiques
% Calcul de la trainée du ballon
rho  = 1.2041;      % Masse volumique de l'air [kg/m^3]
Cx   = 0.45;        % Coefficient de traînée
m    = 0.6;         % Masse [kg]
R    = 0.12;        % Rayon [m]
S_bal = pi * R^2;

% Calcul de la trainée du parachute 
% Estimation d'un paravchute de 0.3m
Cd = 1.5;
R_par = 0.15;
S_par = pi*R_par^2;

g = 9.81;

%% Conditions initiales
theta0 = 50 * pi/180;
dt     = 0.01;
v0 = 30;
vx = v0 * cos(theta0);
vy = v0 * sin(theta0);

x = 0;
y = 2.5;

%% Coefficient de traînée
k_bal = 0.5 * rho * Cx * S_bal / m;
k_par = 0.5 * rho * Cd * S_par / m;
%% Historique
X = x; Y = y;
Vx = vx;
T  = 0;

%% Figures
figure('Name','Simulation animée','NumberTitle','off');

subplot(3,1,1)
hTraj = plot(X, Y, 'b', 'LineWidth', 1.5);
xlabel('X [m]'); ylabel('Y [m]');
title('Trajectoire XY');
grid on;
axis([0 20 0 20]);
hold on;

subplot(3,1,2)
hX = plot(T, X, 'r', 'LineWidth', 1.5);
xlabel('Temps [s]'); ylabel('X [m]');
title('Position horizontale');
grid on;
axis([0 10 0 100]);
hold on;

subplot(3,1,3)
hVx = plot(T, Vx, 'k', 'LineWidth', 1.5);
xlabel('Temps [s]'); ylabel('Vx [m/s]');
title('Vitesse horizontale');
grid on;
axis([0 10 0 35]);
hold on;

%% Boucle temporelle animation
k = k_bal;
parachute_on = false;

y_max = y;
while y >= 0
if x >= 3 && ~parachute_on
    k = k_bal + k_par;
    parachute_on = true;
end
    v = sqrt(vx^2 + vy^2);

    ax = -k * v * vx;
    ay = -k * v * vy - g;

    vx = vx + ax * dt;
    vy = vy + ay * dt;

    x = x + vx * dt;
    y = y + vy * dt;

    X(end+1)  = x;
    Y(end+1)  = y;
    Vx(end+1) = vx;
    T(end+1)  = T(end) + dt;

    set(hTraj, 'XData', X, 'YData', Y);
    set(hX,    'XData', T, 'YData', X);
    set(hVx,   'XData', T, 'YData', Vx);

    drawnow
    pause(dt)   % C’est ça qui crée l’animation
end
