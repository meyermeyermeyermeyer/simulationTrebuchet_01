%% OPTIMISATION, ANIMATION ET ANALYSE PHYSIQUE COMPLÈTE
clc; clear; close all;

%% =====================================================================
%%  DONNÉES UTILISATEUR — SIMULATION
%% =====================================================================

% --- Cibles de trajectoire ---
cible_Hmax = 8;       % Hauteur maximale à atteindre / plafond (m)
cible_Dist = 25+2.1;      % Distance horizontale de la cible (m)

% --- Point de départ de l'optimisation ---
p0 = [15, 60];        % [v0_initiale (m/s), theta0_initiale (°)] — estimation initiale pour fminsearch

% --- Propriétés physiques du projectile ---
m      = 0.6;         % Masse du projectile (kg)
r_bal  = 0.12;        % Rayon de la sphère / projectile (m)
Cd_bal = 0.45;        % Coefficient de traînée du projectile sans parachute (-)

% --- Conditions initiales ---
y0_sim = 3.9;         % Hauteur initiale de lancement (m)

% --- Environnement ---
rho = 1.2041;         % Densité de l'air (kg/m³)
g   = 9.81;           % Accélération gravitationnelle (m/s²)

% --- Paramètres numériques ---
dt = 0.005;           % Pas de temps de la simulation (s)

%% =====================================================================
%%  DONNÉES UTILISATEUR — PARACHUTE
%% =====================================================================

% --- Masse ---
Pa_m = 1.5;           % Masse totale capsule + parachute (kg)

% --- Parachute déployé ---
Pa_Cd       = 1.38;    % Coefficient de traînée du parachute déployé (-)
Pa_A        = 0.5;    % Surface projetée du parachute déployé (m²)
Pa_CdAfull  = Pa_Cd * Pa_A;  % Produit Cd×A parachute déployé (m²)

% --- Capsule seule (avant déploiement complet) ---
Pa_Cd0  = 0.5;        % Coefficient de traînée de la capsule seule (-)
Pa_A0   = 0.01;       % Surface projetée de la capsule seule (m²)
Pa_CdA0 = Pa_Cd0 * Pa_A0;   % Produit Cd×A capsule seule (m²)

% --- Dynamique de déploiement ---
Pa_t_dep    = 1.4;    % Instant de début de déploiement du parachute (s) — correspond à t_para
Pa_tau      = 0.4;    % Constante de temps d'inflation du parachute (s)
Pa_tauAlign = 0.25;   % Constante de temps de réorientation horizontale (s)

% --- Conditions initiales du parachute (reprises de la simulation au moment du déploiement) ---
Pa_theta0 = -20;      % Angle initial de la vitesse au déploiement (°, 0=horizontal, négatif=vers le bas)
Pa_x0_pa  = 0;        % Position horizontale initiale pour l'analyse parachute (m)
Pa_y0_pa  = 5;        % Position verticale initiale pour l'analyse parachute (m)

% --- Intervalle de la simulation parachute ---
Pa_tspan = [0 5];     % Intervalle de temps de l'analyse parachute (s)

% --- Seuils de détection de stabilisation ---
Pa_eps_dvdt   = 0.05; % Seuil sur |dv/dt| pour considérer la vitesse stabilisée (m/s²)
Pa_t_min_stab = 1.5;  % Temps minimal avant recherche de stabilisation (s)

% --- Profondeurs de chute à analyser ---
Pa_chutes = [3 4 5];  % Profondeurs de chute à analyser (m)

%% =====================================================================
%%  SECTION 1 : OPTIMISATION DE LA TRAJECTOIRE
%% =====================================================================

options = optimset('Display', 'none', 'TolX', 1e-4);
fprintf('Optimisation des paramètres en cours...\n');

[p_opt, ~] = fminsearch(@(p) fonction_cout(p, cible_Hmax, cible_Dist, ...
    m, r_bal, Cd_bal, Pa_t_dep, Pa_tau, Pa_tauAlign, Pa_CdA0, Pa_CdAfull, Pa_m, ...
    y0_sim, rho, g, dt), p0, options);

v0_final     = p_opt(1);
theta0_final = p_opt(2);

%% =====================================================================
%%  SECTION 2 : SIMULATION BALISTIQUE COMPLÈTE
%% =====================================================================

[H, D, X, Y, T_sim, V_mag, A_mag, idx_para] = sim_physique_complete(v0_final, theta0_final, ...
    m, r_bal, Cd_bal, Pa_t_dep, Pa_tau, Pa_tauAlign, Pa_CdA0, Pa_CdAfull, Pa_m, ...
    y0_sim, rho, g, dt);

%% =====================================================================
%%  SECTION 3 : ANALYSE DYNAMIQUE DU PARACHUTE (ode45)
%% =====================================================================

% Vitesse au moment du déploiement (reprise depuis la simulation balistique)
Pa_V0  = V_mag(idx_para);   % Norme de la vitesse au déploiement (m/s)
Pa_vx0 = Pa_V0 * cosd(Pa_theta0);  % Composante horizontale initiale (m/s)
Pa_vy0 = Pa_V0 * sind(Pa_theta0);  % Composante verticale initiale (m/s)

Pa_x0vec = [Pa_x0_pa; Pa_y0_pa; Pa_vx0; Pa_vy0];  % État initial [x; y; vx; vy]

[Pa_t, Pa_X] = ode45(@(Pa_t_loc, Pa_x_loc) Pa_dyn_corrected( ...
    Pa_t_loc, Pa_x_loc, ...
    Pa_m, -g, rho, ...
    Pa_CdA0, Pa_CdAfull, ...
    0, Pa_tau, Pa_tauAlign), ...
    Pa_tspan, Pa_x0vec);

% Extraction des composantes d'état
Pa_x  = Pa_X(:,1);   % Position horizontale (m)
Pa_y  = Pa_X(:,2);   % Position verticale (m)
Pa_vx = Pa_X(:,3);   % Vitesse horizontale (m/s)
Pa_vy = Pa_X(:,4);   % Vitesse verticale (m/s)

% Calcul des accélérations
Pa_ay = zeros(size(Pa_t));
Pa_ax = zeros(size(Pa_t));
for Pa_k = 1:length(Pa_t)
    [Pa_ax(Pa_k), Pa_ay(Pa_k)] = Pa_accel_corrected( ...
        Pa_t(Pa_k), Pa_vx(Pa_k), Pa_vy(Pa_k), ...
        Pa_m, -g, rho, ...
        Pa_CdA0, Pa_CdAfull, ...
        0, Pa_tau, Pa_tauAlign);
end

% CdA effectif en fonction du temps
Pa_CdA_eff = Pa_CdA0 + (Pa_CdAfull - Pa_CdA0) .* (1 - exp(-Pa_t / Pa_tau));
Pa_CdA_eff(Pa_t < 0) = Pa_CdA0;

% Vitesse terminale théorique
Pa_vt_theo = -sqrt( (2 * Pa_m * g) / (rho * Pa_CdAfull) );

% Détection de la stabilisation
Pa_dvy   = diff(Pa_vy) ./ diff(Pa_t);
Pa_t_mid = Pa_t(1:end-1);
Pa_idx_stab = find(abs(Pa_dvy) < Pa_eps_dvdt & Pa_t_mid >= Pa_t_min_stab, 1, 'first');
if isempty(Pa_idx_stab)
    Pa_idx_term = length(Pa_t);
else
    Pa_idx_term = Pa_idx_stab + 1;
end
Pa_t_term  = Pa_t(Pa_idx_term);
Pa_vy_term = Pa_vy(Pa_idx_term);
Pa_x_term  = Pa_x(Pa_idx_term);
Pa_y_term  = Pa_y(Pa_idx_term);

% Décélération maximale
Pa_a_norm = sqrt(Pa_ax.^2 + Pa_ay.^2);
[Pa_a_max, Pa_idx_a_max] = max(Pa_a_norm);
Pa_t_a_max = Pa_t(Pa_idx_a_max);

% Temps et déplacement pour chutes de 3, 4, 5 m
Pa_t_chute = nan(size(Pa_chutes));
Pa_x_chute = nan(size(Pa_chutes));
for Pa_i = 1:length(Pa_chutes)
    Pa_d = Pa_chutes(Pa_i);
    Pa_idx_d = find(Pa_y <= -Pa_d, 1, 'first');
    if ~isempty(Pa_idx_d)
        Pa_t_chute(Pa_i) = Pa_t(Pa_idx_d);
        Pa_x_chute(Pa_i) = Pa_x(Pa_idx_d);
    end
end

%% =====================================================================
%%  AFFICHAGE DES VALEURS IMPORTANTES
%% =====================================================================

fprintf('\n========================================================\n');
fprintf('       RÉSULTATS — SIMULATION BALISTIQUE\n');
fprintf('========================================================\n');
fprintf('  Vitesse initiale optimisée       : %.2f m/s\n',  v0_final);
fprintf('  Angle initial optimisé           : %.2f °\n',    theta0_final);
fprintf('  Hauteur maximale atteinte        : %.2f m\n',    H);
fprintf('  Distance horizontale finale      : %.2f m\n',    D);
fprintf('  Vitesse au déploiement parachute : %.2f m/s\n',  V_mag(idx_para));
fprintf('  Temps de la trajectoire          : %.2f \n',  max(T_sim));

fprintf('\n========================================================\n');
fprintf('       RÉSULTATS — DYNAMIQUE DU PARACHUTE\n');
fprintf('========================================================\n');
fprintf('  Vitesse terminale théorique        : %.2f m/s\n',     Pa_vt_theo);
fprintf('  Vitesse verticale stabilisée       : %.2f m/s\n',     Pa_vy_term);
fprintf('  Temps de stabilisation             : %.2f s\n',       Pa_t_term);
fprintf('  Position verticale à stabilisation : %.2f m\n',       Pa_y_term);
fprintf('  Position horizontale à stabilisation : %.2f m\n',     Pa_x_term);
fprintf('  Décélération maximale              : %.2f m/s² à t = %.2f s\n', Pa_a_max, Pa_t_a_max);

fprintf('\n  -----------------------------------------------\n');
fprintf('     Chute (m) | Temps (s) | Déplacement X (m)\n');
fprintf('  -----------------------------------------------\n');
for Pa_i = 1:length(Pa_chutes)
    if ~isnan(Pa_t_chute(Pa_i))
        fprintf('      %1d      |  %6.3f  |      %8.3f\n', ...
            Pa_chutes(Pa_i), Pa_t_chute(Pa_i), Pa_x_chute(Pa_i));
    else
        fprintf('      %1d      |    N/A   |        N/A\n', Pa_chutes(Pa_i));
    end
end
fprintf('  -----------------------------------------------\n');

%% =====================================================================
%%  FIGURE 1 : TRAJECTOIRE + PROPRIÉTÉS DE TRAÎNÉE DU PARACHUTE
%% =====================================================================

fig1 = figure('Name', 'Trajectoire et Propriétés de Traînée', ...
    'Color', 'w', 'Position', [50, 100, 950, 750]);

% --- Subplot 1 : Trajectoire balistique animée ---
subplot(2,2,[1 2]);
hold on; grid on;
xlabel('Distance X (m)'); ylabel('Hauteur Y (m)');
title(sprintf('Trajectoire optimisée : V_0 = %.1f m/s, \\theta = %.1f°, Temps = %.3fs', v0_final, theta0_final,max(T_sim)));
axis([0 max(X)+0.5 0 max(Y)+1]);
yline(cible_Hmax, '--r', 'Cible H_{max}', 'LabelHorizontalAlignment', 'left');
xline(cible_Dist, '--r', 'Cible Dist');

hTraj      = plot(NaN, NaN, 'b', 'LineWidth', 2);
hPoint     = plot(NaN, NaN, 'ko', 'MarkerFaceColor', 'r', 'MarkerSize', 7);
hParaLabel = text(NaN, NaN, '');

for i = 1:8:length(X)
    set(hTraj,  'XData', X(1:i), 'YData', Y(1:i));
    set(hPoint, 'XData', X(i),   'YData', Y(i));
    if i >= idx_para && idx_para > 0
        set(hParaLabel, 'Position', [X(idx_para), Y(idx_para)+0.5], ...
            'String', '🪂 Déploiement', 'Color', [0.8 0.2 0], 'FontWeight', 'bold');
        plot(X(idx_para), Y(idx_para), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
    end
    pause(0.01); drawnow;
end

% --- Subplot 2 : CdA effectif du parachute ---
subplot(2,2,3);
plot(Pa_t, Pa_CdA_eff, 'Color', [0.2 0.4 0.8], 'LineWidth', 1.8);
hold on;
yline(Pa_CdAfull, '--k', sprintf('CdA_{déployé} = %.3f m²', Pa_CdAfull), ...
    'LabelHorizontalAlignment', 'left', 'FontSize', 8);
yline(Pa_CdA0,    ':',  sprintf('CdA_{capsule} = %.4f m²', Pa_CdA0), ...
    'LabelHorizontalAlignment', 'left', 'FontSize', 8);
grid on;
xlabel('Temps depuis déploiement (s)'); ylabel('C_d \times A (m²)');
title('Évolution du C_d \times A du parachute');
xlim([0 Pa_tspan(2)]);

% --- Subplot 3 : Trajectoire parachute ---
subplot(2,2,4);
plot(Pa_x, Pa_y, 'Color', [0.8 0.3 0], 'LineWidth', 1.8);
hold on;
plot(Pa_x(1),           Pa_y(1),           'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
plot(Pa_x(Pa_idx_term), Pa_y(Pa_idx_term), 'bs', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
legend('Trajectoire', 'Départ', 'Stabilisation', 'Location', 'best');
grid on; axis equal;
xlabel('Position horizontale (m)'); ylabel('Position verticale (m)');
title('Trajectoire de la capsule — Phase parachute');

%% =====================================================================
%%  FIGURE 2 : VITESSE ET ACCÉLÉRATION — PHASE PARACHUTE
%% =====================================================================

fig2 = figure('Name', 'Vitesse et Accélération — Phase Parachute', ...
    'Color', 'w', 'Position', [1020, 100, 850, 750]);

% --- Subplot 1 : Vitesses ---
subplot(3,1,1);
plot(Pa_t, Pa_vx, 'LineWidth', 1.5); hold on;
plot(Pa_t, Pa_vy, 'LineWidth', 1.5);
yline(Pa_vt_theo, '--k', sprintf('v_{term} = %.2f m/s', Pa_vt_theo), ...
    'LabelHorizontalAlignment', 'left', 'FontSize', 8);
xline(Pa_t_term,  '--b', sprintf('t_{stab} = %.2f s', Pa_t_term), 'FontSize', 8);
grid on;
legend('Vitesse horizontale v_x', 'Vitesse verticale v_y', 'Location', 'best');
title('Vitesses du système lors du déploiement parachute');
xlabel('Temps (s)'); ylabel('Vitesse (m/s)');

% --- Subplot 2 : Accélérations ---
subplot(3,1,2);
plot(Pa_t, Pa_ax, 'LineWidth', 1.5); hold on;
plot(Pa_t, Pa_ay, 'LineWidth', 1.5);
plot(Pa_t, Pa_a_norm, 'k--', 'LineWidth', 1.2);
xline(Pa_t_a_max, '--r', sprintf('a_{max} = %.2f m/s²', Pa_a_max), 'FontSize', 8);
grid on;
legend('Accélération a_x', 'Accélération a_y', 'Norme \|a\|', 'Location', 'best');
title('Accélérations du système');
xlabel('Temps (s)'); ylabel('Accélération (m/s²)');

% --- Subplot 3 : Norme de la vitesse ---
subplot(3,1,3);
Pa_V_norm = sqrt(Pa_vx.^2 + Pa_vy.^2);
plot(Pa_t, Pa_V_norm, 'Color', [0.5 0 0.5], 'LineWidth', 1.5);
hold on;
xline(Pa_t_term, '--b', sprintf('t_{stab} = %.2f s', Pa_t_term), 'FontSize', 8);
grid on;
title('Norme de la vitesse totale');
xlabel('Temps (s)'); ylabel('||v|| (m/s)');

%% =====================================================================
%%  FONCTIONS LOCALES
%% =====================================================================

%% --- Fonction coût pour l'optimisation ---
function score = fonction_cout(p, h_target, d_target, ...
    m, r_bal, Cd_bal, t_para, Pa_tau, Pa_tauAlign, Pa_CdA0, Pa_CdAfull, Pa_m, ...
    y0, rho, g, dt)
    [h_sim, d_sim] = sim_physique_complete(p(1), p(2), ...
        m, r_bal, Cd_bal, t_para, Pa_tau, Pa_tauAlign, Pa_CdA0, Pa_CdAfull, Pa_m, ...
        y0, rho, g, dt);
    score = (h_sim - h_target)^2 + (d_sim - d_target)^2;
end

%% --- Simulation balistique avec modèle de traînée parachute précis ---
function [y_max, x_final, X, Y, T, V_mag, A_mag, idx_para] = sim_physique_complete( ...
    v0, theta0_deg, m, r_bal, Cd_bal, t_para, Pa_tau, Pa_tauAlign, Pa_CdA0, Pa_CdAfull, Pa_m, ...
    y0, rho, g, dt)

    % Coefficient de traînée balistique (projectile seul, constant)
    k_bal = 0.5 * rho * Cd_bal * (pi * r_bal^2) / m;

    vx = v0 * cos(theta0_deg * pi/180);
    vy = v0 * sin(theta0_deg * pi/180);
    x = 0; y = y0; t = 0;

    X = x; Y = y; T = t; V_mag = sqrt(vx^2+vy^2); A_mag = g;
    y_max    = y;
    idx_para = 0;

    while y >= 0

        if t < t_para
            %-- Phase balistique : traînée isotrope du projectile seul --
            k   = k_bal;
            v   = sqrt(vx^2 + vy^2);
            ax  = -k * v * vx;
            ay  = -k * v * vy - g;

        else
            %-- Phase parachute : modèle précis (inflation + réorientation) --
            if idx_para == 0, idx_para = length(X); end

            % Temps écoulé depuis le début du déploiement
            t_dep_loc = t - t_para;

            % CdA effectif avec inflation exponentielle (identique à Pa_dyn_corrected)
            CdA_eff = Pa_CdAfull * (1 - exp(-t_dep_loc / Pa_tau));

            % Traînée verticale uniquement (modèle parachute)
            Fdy = -0.5 * rho * CdA_eff * abs(vy) * vy;   % N

            % Accélérations : réorientation horizontale + gravité/traînée verticale
            ax  = -vx / Pa_tauAlign;                       % m/s², réorientation horiz.
            ay  =  Fdy / Pa_m - g;                         % m/s², traînée vert. + gravité
        end

        vx = vx + ax * dt; vy = vy + ay * dt;
        x  = x  + vx * dt; y  = y  + vy * dt; t = t + dt;

        X(end+1)     = x;                  %#ok<AGROW>
        Y(end+1)     = y;                  %#ok<AGROW>
        T(end+1)     = t;                  %#ok<AGROW>
        V_mag(end+1) = sqrt(vx^2 + vy^2); %#ok<AGROW>
        A_mag(end+1) = sqrt(ax^2 + ay^2); %#ok<AGROW>

        if y > y_max, y_max = y; end
    end
    x_final = x;
end

%% --- Dérivée d'état pour ode45 (dynamique parachute) ---
function Pa_dxdt = Pa_dyn_corrected( ...
    Pa_t_loc, Pa_x_loc, ...
    Pa_m_loc, Pa_g_loc, Pa_rho_loc, ...
    Pa_CdA0_loc, Pa_CdAfull_loc, ...
    Pa_t_dep_loc, Pa_tau_loc, Pa_tauAlign_loc)

    Pa_vx_loc = Pa_x_loc(3);
    Pa_vy_loc = Pa_x_loc(4);

    if Pa_t_loc < Pa_t_dep_loc
        Pa_CdA_loc = Pa_CdA0_loc;
    else
        Pa_CdA_loc = Pa_CdAfull_loc * (1 - exp(-(Pa_t_loc - Pa_t_dep_loc)/Pa_tau_loc));
    end

    Pa_Fdy_loc = -0.5 * Pa_rho_loc * Pa_CdA_loc * abs(Pa_vy_loc) * Pa_vy_loc;  % N, traînée verticale
    Pa_ax_loc  = -Pa_vx_loc / Pa_tauAlign_loc;                                  % m/s², réorientation horiz.
    Pa_ay_loc  =  Pa_g_loc  + Pa_Fdy_loc / Pa_m_loc;                            % m/s², accél. verticale

    Pa_dxdt = [Pa_vx_loc; Pa_vy_loc; Pa_ax_loc; Pa_ay_loc];
end

%% --- Accélérations instantanées (pour post-traitement) ---
function [Pa_ax_out, Pa_ay_out] = Pa_accel_corrected( ...
    Pa_t_loc, Pa_vx_loc, Pa_vy_loc, ...
    Pa_m_loc, Pa_g_loc, Pa_rho_loc, ...
    Pa_CdA0_loc, Pa_CdAfull_loc, ...
    Pa_t_dep_loc, Pa_tau_loc, Pa_tauAlign_loc)

    if Pa_t_loc < Pa_t_dep_loc
        Pa_CdA_loc = Pa_CdA0_loc;
    else
        Pa_CdA_loc = Pa_CdAfull_loc * (1 - exp(-(Pa_t_loc - Pa_t_dep_loc)/Pa_tau_loc));
    end

    Pa_Fdy_loc  = -0.5 * Pa_rho_loc * Pa_CdA_loc * abs(Pa_vy_loc) * Pa_vy_loc;
    Pa_ax_out   = -Pa_vx_loc / Pa_tauAlign_loc;
    Pa_ay_out   =  Pa_g_loc  + Pa_Fdy_loc / Pa_m_loc;
end
