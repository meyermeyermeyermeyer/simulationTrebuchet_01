clear; clc; close all;

% =====================
% SIMULATION TRÉBUCHET
% =====================

% --- Paramètres Physiques ---
lb2kg = 0.454;

p.m = 2.0;          % Masse projectile [kg]
p.M =  250 * lb2kg;        % Masse contrepoids [kg] (2 plates 35 par coté)
p.mbeam = 10.0;     % Masse du bras [kg]
p.l1 = 0.915;         % Longueur bras côté projectile [m]
p.l2 = 0.48;         % Longueur bras côté contrepoids [m]
p.l3 = 0.435;         % Longueur bras contrepoids [m]
p.l4 = 0.784;         % Longueur corde [m]
p.g = 9.81;         % Gravité [m/s²]

% Moment d'inertie du bras autour du pivot
p.Ibeam = (1/3)*p.mbeam*(p.l1^2 + p.l2^2);

% --- Conditions Initiales ---
% Angles (en degrés puis convertis en radians)
th0  = deg2rad(60);   % Bras incliné vers le haut
ps0  = deg2rad(80);    % Fronde pend verticalement
ph0  = deg2rad(0);    % Contrepoids pend verticalement

% Vitesses angulaires initiales (TOUTES NULLES)
dth0 = 0;
dps0 = 0;
dph0 = 0;

% Vecteur d'état initial: [theta, psi, phi, dtheta, dpsi, dphi]
y0 = [th0; ps0; ph0; dth0; dps0; dph0];

% --- Résolution ODE ---
tspan = [0 0.7];  % Temps de simulation [s]
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-8);

fprintf('Résolution de l''équation différentielle...\n');
[t, y] = ode45(@(t,y) trebuchet_dynamics(t, y, p), tspan, y0, options);
fprintf('Simulation terminée: %d points calculés.\n', length(t));

% --- Visualisation des Angles ---
figure('Color', 'w', 'Position', [100 100 1000 600]);

subplot(3,1,1);
plot(t, rad2deg(y(:,1)), 'LineWidth', 2, 'Color', [0.8 0.3 0.1]);
ylabel('\theta [deg]'); grid on;
title('Évolution des Angles du Trébuchet');

subplot(3,1,2);
plot(t, rad2deg(y(:,2)), 'LineWidth', 2, 'Color', [0.2 0.4 0.9]);
ylabel('\psi [deg]'); grid on;

subplot(3,1,3);
plot(t, rad2deg(y(:,3)), 'LineWidth', 2, 'Color', [0.9 0.1 0.3]);
ylabel('\phi [deg]'); grid on;
xlabel('Temps [s]');

% --- Visualisation des Vitesses Angulaires ---
figure('Color', 'w', 'Position', [120 120 1000 600]);

subplot(3,1,1);
plot(t, rad2deg(y(:,4)), 'LineWidth', 2, 'Color', [0.8 0.3 0.1]);
ylabel('d\theta/dt [deg/s]'); grid on;
title('Vitesses Angulaires du Trébuchet');

subplot(3,1,2);
plot(t, rad2deg(y(:,5)), 'LineWidth', 2, 'Color', [0.2 0.4 0.9]);
ylabel('d\psi/dt [deg/s]'); grid on;

subplot(3,1,3);
plot(t, rad2deg(y(:,6)), 'LineWidth', 2, 'Color', [0.9 0.1 0.3]);
ylabel('d\phi/dt [deg/s]'); grid on;
xlabel('Temps [s]');


% --- Animation ---
fprintf('\nLancement de l''animation\n');
animation(t, y, p);

% ========================================
% FONCTION: Dynamique du Trébuchet
% ========================================
function dydt = trebuchet_dynamics(~, y, p)
    % Extraction des variables d'état
    th = y(1);   % Angle du bras
    ps = y(2);   % Angle de la fronde
    ph = y(3);   % Angle du contrepoids
    dth = y(4);  % Vitesse angulaire du bras
    dps = y(5);  % Vitesse angulaire de la fronde
    dph = y(6);  % Vitesse angulaire du contrepoids

    % --- MATRICE DE MASSE M (3x3) ---
    M11 = p.Ibeam + p.M*p.l2^2 + p.l1^2*p.m;
    M12 = -p.l1*p.l4*p.m*sin(ps - th);
    M13 = p.M*p.l2*p.l3*sin(ph - th);
    
    M21 = M12;  % Symétrique
    M22 = p.l4^2*p.m;
    M23 = 0;
    
    M31 = M13;  % Symétrique
    M32 = 0;
    M33 = p.M*p.l3^2;
    
    M = [M11, M12, M13;
         M21, M22, M23;
         M31, M32, M33];

    % --- VECTEUR FORCE F (3x1) ---
    % Composante pour theta
    F1 = -p.M*p.g*p.l2*cos(th) ...
         - p.M*p.l2*p.l3*cos(ph - th)*dph^2 ...
         + p.g*p.l1*p.m*cos(th) ...
         + p.g*p.l1*p.mbeam*cos(th)/2 ...
         - p.g*p.l2*p.mbeam*cos(th)/2 ...
         + p.l1*p.l4*p.m*cos(ps - th)*dps^2;
    
    % Composante pour psi
    F2 = -p.l4*p.m*(p.g*sin(ps) + p.l1*cos(ps - th)*dth^2);
    
    % Composante pour phi
    F3 = p.M*p.l3*(-p.g*sin(ph) + p.l2*cos(ph - th)*dth^2);
    
    F = [F1; F2; F3];

    % --- RÉSOLUTION: M * [ddth; ddps; ddph] = F ---
    accel = M \ F;
    
    % Vecteur dérivée d'état
    dydt = [y(4:6); accel];
end



% 1. Exécute ta simulation (assure-toi que p.g et p.l1, etc., sont définis)
[t, y] = ode45(@(t,y) trebuchet_dynamics(t, y, p), [0 0.7], y0);

% 2. Choisis l'angle que tu veux tester
angle_test = 31;

[dist, haut] = calcul_trajectoire(t, y, p, angle_test);

% --- CALCUL DE LA VITESSE DE SORTIE (PREMIER PIC LOCAL) ---
% Extraction des variables d'état
th  = y(:,1);  
ps  = y(:,2);
dth = y(:,4); 
dps = y(:,5);

% Calcul vectoriel de la vitesse du projectile
vx = p.l1 * sin(th) .* dth + p.l4 * cos(ps) .* dps;
vy = -p.l1 * cos(th) .* dth + p.l4 * sin(ps) .* dps;
v_norme = sqrt(vx.^2 + vy.^2);

% --- LOGIQUE DE DÉTECTION DU PREMIER PIC ---
% On cherche le premier index où la vitesse commence à redescendre
idx_pics = find(diff(v_norme) < 0, 1); 

if isempty(idx_pics)
    % Si aucun pic n'est trouvé, on prend la valeur maximale par défaut
    [v_max, idx] = max(v_norme);
    fprintf('Note: Aucun pic local détecté, utilisation du max absolu.\n');
else
    idx = idx_pics; % On s'arrête au premier sommet trouvé
    v_max = v_norme(idx);
end


%% ======================================
% Calcul de l'angle de sortie au pic
angle_sortie_rad = atan2(vy(idx), vx(idx));
angle_sortie_deg = rad2deg(angle_sortie_rad);

% --- AFFICHAGE DES RÉSULTATS ---
fprintf('\n====================================\n');
fprintf('   RÉSULTATS DU LANCEMENT \n');
fprintf('====================================\n');
fprintf('Vitesse de sortie     : %.2f m/s\n', v_max);
fprintf('Angle de sortie       : %.2f degrés\n', angle_sortie_deg);
fprintf('Temps du pic          : %.3f s\n', t(idx));
fprintf('====================================\n');

% --- Graphique de la Vitesse ---
figure('Color', 'w', 'Name', 'Analyse de la vitesse de sortie');
plot(t, v_norme, 'LineWidth', 2, 'Color', [0.2 0.2 0.2]); hold on;
plot(t(idx), v_max, 'ro', 'MarkerSize', 12, 'LineWidth', 2, 'MarkerFaceColor', 'r');

% Habillage du graphique
xlabel('Temps [s]'); 
ylabel('Vitesse du projectile [m/s]');
title(['Vitesse de sortie : ', num2str(v_max, '%.2f'), ' m/s à ', num2str(angle_sortie_deg, '%.1f'), '°']);
legend('Vitesse instantanée', 'Premier maximum (Lâcher optimal)', 'location', 'best');
grid on;
% ============================================================



%% ======================================
% CALCUL DE LA TENSION DANS LA CORDE
% ======================================

N = length(t);
Tension = zeros(N,1);

for i = 1:N
    
    % Extraction état instantané
    th  = y(i,1);
    ps  = y(i,2);
    ph  = y(i,3);
    dth = y(i,4);
    dps = y(i,5);
    dph = y(i,6);
    
    % --- Recalcul des accélérations angulaires ---
    
    % Matrice de masse
    M11 = p.Ibeam + p.M*p.l2^2 + p.l1^2*p.m;
    M12 = -p.l1*p.l4*p.m*sin(ps - th);
    M13 = p.M*p.l2*p.l3*sin(ph - th);
    
    M21 = M12;
    M22 = p.l4^2*p.m;
    M23 = 0;
    
    M31 = M13;
    M32 = 0;
    M33 = p.M*p.l3^2;
    
    M = [M11 M12 M13;
         M21 M22 M23;
         M31 M32 M33];
    
    % Forces généralisées
    F1 = -p.M*p.g*p.l2*cos(th) ...
         - p.M*p.l2*p.l3*cos(ph - th)*dph^2 ...
         + p.g*p.l1*p.m*cos(th) ...
         + p.g*p.l1*p.mbeam*cos(th)/2 ...
         - p.g*p.l2*p.mbeam*cos(th)/2 ...
         + p.l1*p.l4*p.m*cos(ps - th)*dps^2;
    
    F2 = -p.l4*p.m*(p.g*sin(ps) + p.l1*cos(ps - th)*dth^2);
    
    F3 = p.M*p.l3*(-p.g*sin(ph) + p.l2*cos(ph - th)*dth^2);
    
    accel = M \ [F1; F2; F3];
    
    ddth = accel(1);
    ddps = accel(2);
    
    % --- Accélération du projectile ---
    
    ax = -p.l1*cos(th)*dth^2 - p.l1*sin(th)*ddth ...
         - p.l4*sin(ps)*dps^2 + p.l4*cos(ps)*ddps;

    ay = -p.l1*sin(th)*dth^2 + p.l1*cos(th)*ddth ...
         - p.l4*cos(ps)*dps^2 - p.l4*sin(ps)*ddps;
    
    % Direction radiale de la corde
    erx = sin(ps);
    ery = -cos(ps);
    
    % Accélération projetée
    a_radial = ax*erx + ay*ery;
    
    % Tension
    Tension(i) = p.m*(a_radial + p.g*ery);
    
end

fprintf('\nTension maximale: %.1f N\n', max(Tension));
fprintf('Tension minimale: %.1f N\n', min(Tension));

figure('Color','w');
plot(t, Tension, 'LineWidth', 2);
xlabel('Temps [s]');
ylabel('Tension [N]');
title('Tension dans la corde');
grid on;
