clear; clc; close all;

% ========================================
% SIMULATION TRÉBUCHET AVEC LIBÉRATION
% ========================================
% Basé sur votre code avec animation complète en 2 phases

% --- Paramètres Physiques ---
p.m = 2.0;          % Masse projectile [kg]
p.lb2kg = 2.205; 
p.in2m = 0.0254;
p.M = 180 / p.lb2kg;        % Masse contrepoids [kg]
p.mbeam = 10.0;     % Masse du bras [kg]
p.l1 = 0.915;         % Longueur bras côté projectile [m]
p.l2 = 0.48;         % Longueur bras côté contrepoids [m]
p.l3 = 0.435;         % Longueur fil contrepoids [m]
p.l4 = 0.80;         % Longueur fronde [m]
p.g = 9.81;         % Gravité [m/s²]

% Moment d'inertie du bras
p.Ibeam = (1/3)*p.mbeam*(p.l1^2 + p.l2^2);

fprintf('========================================\n');
fprintf('PARAMÈTRES DU TRÉBUCHET\n');
fprintf('========================================\n');
fprintf('Projectile: %.1f kg\n', p.m);
fprintf('Contrepoids: %.1f kg (%.0f lb)\n', p.M, p.M * p.lb2kg);
fprintf('Bras projectile: %.2f m\n', p.l1);
fprintf('Bras contrepoids: %.2f m\n', p.l2);
fprintf('Fronde: %.2f m\n\n', p.l4);

% --- INPUT UTILISATEUR: Angle de libération ---
fprintf('CHOIX DE L''ANGLE DE LIBÉRATION\n');
fprintf('========================================\n');
fprintf('L''angle de libération détermine quand le projectile est relâché.\n');
fprintf('Cet angle est mesuré par rapport à l''horizontale.\n\n');
fprintf('Exemples:\n');
fprintf('  30° → Tir tendu (courte portée)\n');
fprintf('  45° → Tir optimal (portée maximale)\n');
fprintf('  60° → Tir en cloche (trajectoire haute)\n\n');

angle_liberation = input('Entrez l''angle de libération souhaité [degrés] (défaut: 45): ');

if isempty(angle_liberation)
    angle_liberation = 45;
    fprintf('→ Valeur par défaut utilisée: 45°\n\n');
else
    fprintf('→ Angle de libération: %.1f°\n\n', angle_liberation);
end

% --- Conditions Initiales ---
fprintf('CONFIGURATION INITIALE\n');
fprintf('========================================\n');

% Demander les angles initiaux ou utiliser défauts
reponse = input('Utiliser les angles par défaut? (o/n) [o]: ', 's');

if isempty(reponse) || strcmpi(reponse, 'o') || strcmpi(reponse, 'oui')
    th0  = deg2rad(60);   % Bras incliné
    ps0  = deg2rad(80);   % Fronde
    ph0  = deg2rad(0);    % Contrepoids
    fprintf('→ Angles par défaut: θ₀=60°, ψ₀=80°, φ₀=0°\n\n');
else
    th0_deg = input('  Angle initial du bras θ₀ [deg] (0-90): ');
    ps0_deg = input('  Angle initial de la fronde ψ₀ [deg] (-90 à 90): ');
    ph0_deg = input('  Angle initial du contrepoids φ₀ [deg] (-45 à 45): ');
    
    th0 = deg2rad(th0_deg);
    ps0 = deg2rad(ps0_deg);
    ph0 = deg2rad(ph0_deg);
    fprintf('→ Configuration personnalisée appliquée\n\n');
end

% Vitesses initiales (toutes nulles)
dth0 = 0; dps0 = 0; dph0 = 0;

% Vecteur d'état initial
y0 = [th0; ps0; ph0; dth0; dps0; dph0];

% --- Résolution ODE ---
fprintf('SIMULATION EN COURS...\n');
fprintf('========================================\n');

tspan = [0 2];  % Temps de simulation [s]
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-8);

[t, y] = ode45(@(t,y) trebuchet_dynamics(t, y, p), tspan, y0, options);

fprintf('✅ Simulation terminée: %d points calculés en %.2f s\n\n', length(t), toc);

% --- Calcul de la trajectoire avec libération ---
fprintf('CALCUL DE LA TRAJECTOIRE AVEC LIBÉRATION\n');
fprintf('========================================\n');

[dist_finale, h_max] = calcul_trajectoire(t, y, p, angle_liberation);

fprintf('✅ Calculs terminés\n\n');

% --- Visualisation des Angles ---
fprintf('Génération des graphiques...\n');

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

% --- Lancement de l'animation avec libération ---
fprintf('\n');
reponse_anim = input('Lancer l''animation complète avec libération? (o/n) [o]: ', 's');

if isempty(reponse_anim) || strcmpi(reponse_anim, 'o') || strcmpi(reponse_anim, 'y')
    fprintf('\n🎬 Lancement de l''animation...\n\n');
    animation_avec_liberation(t, y, p, angle_liberation);
end

fprintf('\n Programme terminé!\n');

% ========================================
% FONCTION: Dynamique du Trébuchet
% ========================================
function dydt = trebuchet_dynamics(~, y, p)
    th = y(1); ps = y(2); ph = y(3);
    dth = y(4); dps = y(5); dph = y(6);

    % Matrice de masse M (3x3)
    M11 = p.Ibeam + p.M*p.l2^2 + p.l1^2*p.m;
    M12 = -p.l1*p.l4*p.m*sin(ps - th);
    M13 = p.M*p.l2*p.l3*sin(ph - th);
    
    M21 = M12;
    M22 = p.l4^2*p.m;
    M23 = 0;
    
    M31 = M13;
    M32 = 0;
    M33 = p.M*p.l3^2;
    
    M = [M11, M12, M13;
         M21, M22, M23;
         M31, M32, M33];

    % Vecteur force F (3x1)
    F1 = -p.M*p.g*p.l2*cos(th) ...
         - p.M*p.l2*p.l3*cos(ph - th)*dph^2 ...
         + p.g*p.l1*p.m*cos(th) ...
         + p.g*p.l1*p.mbeam*cos(th)/2 ...
         - p.g*p.l2*p.mbeam*cos(th)/2 ...
         + p.l1*p.l4*p.m*cos(ps - th)*dps^2;
    
    F2 = -p.l4*p.m*(p.g*sin(ps) + p.l1*cos(ps - th)*dth^2);
    
    F3 = p.M*p.l3*(-p.g*sin(ph) + p.l2*cos(ph - th)*dth^2);
    
    F = [F1; F2; F3];

    accel = M \ F;
    dydt = [y(4:6); accel];
end

% ========================================
% FONCTION: Calcul de la Trajectoire
% ========================================
function [dist_x, h_max] = calcul_trajectoire(t, y, p, alpha_target_deg)
    % Basé sur votre fonction calcul_trajectoire
    
    h_pivot_sol = 2; % Pivot à 2m du sol
    g = p.g;
    
    % 1. Trouver l'instant du lâcher
    idx = 0;
    
    for k = 2:length(t)
        vx = p.l1*sin(y(k,1))*y(k,4) + p.l4*cos(y(k,2))*y(k,5);
        vy = -p.l1*cos(y(k,1))*y(k,4) + p.l4*sin(y(k,2))*y(k,5);
        angle_actuel = atan2d(vy, vx);
        
        if vy > 0 && angle_actuel <= alpha_target_deg
            idx = k;
            break;
        end
    end
    
    if idx == 0
        error('L''angle de tir cible %.1f° n''est pas atteint dans cette simulation.', alpha_target_deg);
    end
    
    % 2. Conditions initiales du vol
    x0 = -p.l1*cos(y(idx,1)) + p.l4*sin(y(idx,2));
    y0 = -p.l1*sin(y(idx,1)) - p.l4*cos(y(idx,2));
    vx0 = p.l1*sin(y(idx,1))*y(idx,4) + p.l4*cos(y(idx,2))*y(idx,5);
    vy0 = -p.l1*cos(y(idx,1))*y(idx,4) + p.l4*sin(y(idx,2))*y(idx,5);
    
    % Hauteur réelle par rapport au sol
    h_relache = y0 + h_pivot_sol;
    
    % 3. Calculs Balistiques
    t_vol = (vy0 + sqrt(vy0^2 + 2*g*h_relache)) / g;
    dist_x = x0 + vx0 * t_vol;
    h_max = h_relache + (vy0^2) / (2*g);
    
    % 4. Graphique de la parabole
    t_p = linspace(0, t_vol, 50);
    xp = x0 + vx0 * t_p;
    yp = y0 + vy0 * t_p - 0.5 * g * t_p.^2;
    
    figure('Color', 'w', 'Name', 'Trajectoire Balistique');
    hold on; grid on;
    
    plot(xp, yp, 'r-', 'LineWidth', 3);
    plot(x0, y0, 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 12);
    plot(xp(end), yp(end), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 12);
    
    % Sol
    line([-5, dist_x+5], [-h_pivot_sol, -h_pivot_sol], 'Color', 'k', 'LineWidth', 3);
    
    % Hauteur max
    yline(h_max - h_pivot_sol, 'b--', sprintf('H_{max} = %.2f m', h_max), 'LineWidth', 1.5);
    
    title(sprintf('Trajectoire Balistique | Portée = %.2f m | H_{max} = %.2f m | Angle = %.1f°', ...
                  dist_x, h_max, alpha_target_deg), 'FontSize', 13, 'FontWeight', 'bold');
    xlabel('Distance (m)', 'FontSize', 12);
    ylabel('Hauteur (m)', 'FontSize', 12);
    legend('Trajectoire', 'Libération', 'Atterrissage', 'Location', 'best');
    axis equal;
    xlim([min(xp)-1, max(xp)+1]);
    
    fprintf('📍 Libération à t = %.3f s\n', t(idx));
    fprintf('📍 Position: (%.2f, %.2f) m\n', x0, y0);
    fprintf('📍 Vitesse: %.2f m/s à %.1f°\n', sqrt(vx0^2+vy0^2), atan2d(vy0, vx0));
    fprintf('📊 Portée: %.2f m\n', dist_x);
    fprintf('📊 Hauteur max: %.2f m (au-dessus du sol)\n', h_max);
end
