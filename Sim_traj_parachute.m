%% OPTIMISATION, ANIMATION ET ANALYSE PHYSIQUE
clc; clear; close all;

%% --- SECTION 1 : CONFIGURATION ET OPTIMISATION ---
cible_Hmax = 8; % Hauteur maximale à atteindre (plafond)
cible_Dist = 20; % Distance de la cible par rapport à la cible 
p0 = [15, 60]; % v0_final, theta0_final posé arbitrairement

options = optimset('Display', 'none', 'TolX', 1e-4);
fprintf('Optimisation des paramètres en cours...\n');
[p_opt, ~] = fminsearch(@(p) fonction_cout(p, cible_Hmax, cible_Dist), p0, options);

v0_final = p_opt(1);
theta0_final = p_opt(2);

%% --- SECTION 2 : SIMULATION FINALE ET CALCULS ---
[H, D, X, Y, T, V_mag, A_mag, idx_para] = sim_physique_complete(v0_final, theta0_final);

%% --- SECTION 3 : ANIMATION ET GRAPHIQUES ---
fig = figure('Name', 'Analyse de Trajectoire Optimisée', 'Color', 'w', 'Position', [100, 100, 900, 700]);

% --- Subplot 1 : Trajectoire XY ---
subplot(2,1,1);
hold on; grid on;
xlabel('Distance X (m)'); ylabel('Hauteur Y (m)');
title(sprintf('Trajectoire Optimale : V0=%.1fm/s, Angle=%.1f°', v0_final, theta0_final));
axis([0 max(X)+0.5 0 max(Y)+1]);

yline(cible_Hmax, '--r', 'Cible Hmax', 'LabelHorizontalAlignment', 'left');
xline(cible_Dist, '--r', 'Cible Dist');

hTraj = plot(NaN, NaN, 'b', 'LineWidth', 2);
hPoint = plot(NaN, NaN, 'ko', 'MarkerFaceColor', 'r');
hParaLabel = text(NaN, NaN, ''); % Texte pour le parachute

% Boucle d'animation
for i = 1:8:length(X)
    set(hTraj, 'XData', X(1:i), 'YData', Y(1:i));
    set(hPoint, 'XData', X(i), 'YData', Y(i));
    
    % Affichage du déploiement du parachute
    if i >= idx_para && idx_para > 0
        set(hParaLabel, 'Position', [X(idx_para), Y(idx_para)+0.5], ...
            'String', '🪂 Déploiement', 'Color', [0.8 0.2 0], 'FontWeight', 'bold');
        plot(X(idx_para), Y(idx_para), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
    end
    pause(0.01)
    drawnow;
end

% --- Subplot 2 : Vitesse ---
subplot(2,2,3);
plot(X, V_mag, 'Color', [0 0.5 0], 'LineWidth', 1.5); hold on;
if idx_para > 0, xline(T(idx_para), '--', 'Parachute', 'Color', [0.8 0.2 0]); end
grid on; xlabel('Distance X (m)'); ylabel('Vitesse (m/s)');
title('Évolution de la Vitesse');axis([0 max(X)+0.1 0 max(V_mag)+1]);


% --- Subplot 3 : Accélération ---
subplot(2,2,4);
plot(X, A_mag, 'Color', [0.8 0.2 0], 'LineWidth', 1.5); hold on;
if idx_para > 0, xline(T(idx_para), '--', 'Parachute', 'Color', [0.8 0.2 0]); end
grid on; xlabel('Distance X (m)'); ylabel('Accélération (m/s²)');
title('Évolution de l''Accélération');axis([0 max(X)+0.1 0 max(A_mag)+1]);



%% --- FONCTION COÛT ---
function score = fonction_cout(p, h_target, d_target)
    [h_sim, d_sim] = sim_physique_complete(p(1), p(2));
    score = (h_sim - h_target)^2 + (d_sim - d_target)^2;
end

%% --- FONCTION PHYSIQUE AMÉLIORÉE ---
function [y_max, x_final, X, Y, T, V_mag, A_mag, idx_para] = sim_physique_complete(v0, theta0_deg)
    % Paramètres constants
    rho = 1.2041;
    g = 9.81; 
    m = 0.6; 
    dt = 0.005;
    % Ajouter les rayons
    k_bal = 0.5 * rho * 0.45 * (pi * 0.12^2) / m;
    k_par = 0.5 * rho * 1.5 * (pi * 0.15^2) / m;
    
    % Initialisation
    vx = v0 * cos(theta0_deg * pi/180); 
    vy = v0 * sin(theta0_deg * pi/180);
    x = 0; y = 2.5; t = 0;
    
    X = x; Y = y; T = t; V_mag = sqrt(vx^2+vy^2); A_mag = g;
    y_max = y;
    idx_para = 0; % Index où le parachute s'ouvre
    
    while y >= 0
        % Calcul du coefficient k
        if t >= 1.4
            k = k_bal + k_par;
            if idx_para == 0, idx_para = length(X); end
        else
            k = k_bal;
        end
        
        v = sqrt(vx^2 + vy^2);
        ax = -k * v * vx;
        ay = -k * v * vy - g;
        
        vx = vx + ax * dt; vy = vy + ay * dt;
        x = x + vx * dt; y = y + vy * dt; t = t + dt;
        
        X(end+1) = x; Y(end+1) = y; T(end+1) = t; %#ok<AGROW>
        V_mag(end+1) = sqrt(vx^2 + vy^2); %#ok<AGROW>
        A_mag(end+1) = sqrt(ax^2 + ay^2); %#ok<AGROW>
        
        if y > y_max, y_max = y; end
    end
    x_final = x;
end