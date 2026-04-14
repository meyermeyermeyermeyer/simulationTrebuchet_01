 %% OPTIMISATION DU TEMPS DE DÉPLOIEMENT DU PARACHUTE — MODÈLE PRÉCIS
clc; clear; close all;

%% =====================================================================
%% DONNÉES UTILISATEUR
%% =====================================================================

% --- Conditions initiales (angle et vitesse connus) ---
v0_final     = 22.13;   % Vitesse initiale (m/s)
theta0_final = 20;   % Angle initial (°)

% --- Délai mécanique entre déclenchement et déploiement effectif ---
Pa_t_delay = 0.9;       % Délai avant que le parachute soit gonflé (s)

% --- Cible ---
cible_Dist = 28;        % Distance horizontale cible (m)

% --- Point de départ de l'optimisation ---
t_para_0 = 1.0;         % Estimation initiale du temps de déploiement (s)

% --- Propriétés physiques du projectile ---
m      = 1.4;           % Masse du projectile (kg)
r_bal  = 0.12;          % Rayon de la sphère (m)
Cd_bal = 0.25;          % Coefficient de traînée balistique (-)

% --- Condition initiale ---
y0_sim = 3.691;         % Hauteur initiale de lancement (m)

% --- Environnement ---
rho = 1.2041;           % Densité de l'air (kg/m³)
g   = 9.81;             % Accélération gravitationnelle (m/s²)

% --- Paramètres numériques ---
dt = 0.005;             % Pas de temps (s)

%% =====================================================================
%% PARACHUTE
%% =====================================================================

% --- Masse totale capsule + parachute ---
Pa_m = 1.6;             % Masse (kg)

% --- Parachute déployé ---
Pa_Cd      = 1.38;
Pa_A       = pi * 0.4^2;
Pa_CdAfull = Pa_Cd * Pa_A;  % CdA parachute déployé (m²)

% --- Capsule seule (traînée résiduelle avant gonflage complet) ---
Pa_Cd0  = 0.9;
Pa_A0   = 0.01;
Pa_CdA0 = Pa_Cd0 * Pa_A0;  % CdA capsule seule (m²)

% --- Dynamique de déploiement ---
Pa_tau      = 0.4;      % Constante de temps d'inflation (s)
Pa_tauAlign = 0.25;     % Constante de temps de réorientation horizontale (s)

% --- Paramètres de l'analyse parachute (ode45) ---
Pa_tspan       = [0 5];   % Intervalle de temps de l'analyse parachute (s)
Pa_eps_dvdt    = 0.05;    % Seuil sur |dv/dt| pour stabilisation (m/s²)
Pa_t_min_stab  = 1.5;    % Temps minimal avant recherche de stabilisation (s)
Pa_chutes      = [3 4 5]; % Profondeurs de chute à analyser (m)

%% =====================================================================
%% OPTIMISATION DU TEMPS DE DÉPLOIEMENT
%% =====================================================================

options = optimset('Display', 'none', 'TolX', 1e-6);

[t_para_opt, ~] = fminsearch(@(t_p) fonction_cout_tpara( ...
    t_p, v0_final, theta0_final, cible_Dist, ...
    m, r_bal, Cd_bal, Pa_tau, Pa_tauAlign, ...
    Pa_CdA0, Pa_CdAfull, Pa_m, ...
    y0_sim, rho, g, dt), t_para_0, options);

Pa_t_dep     = t_para_opt;
Pa_t_trigger = Pa_t_dep - Pa_t_delay;  % Instant de déclenchement (signal)

%% =====================================================================
%% SIMULATION BALISTIQUE FINALE
%% =====================================================================

[H, D, X, Y, T_sim, V_mag, A_mag, idx_para] = sim_physique_complete( ...
    v0_final, theta0_final, ...
    m, r_bal, Cd_bal, Pa_t_dep, ...
    Pa_tau, Pa_tauAlign, ...
    Pa_CdA0, Pa_CdAfull, Pa_m, ...
    y0_sim, rho, g, dt);

% Index de l'instant de déclenchement dans la trajectoire
idx_trigger = find(T_sim >= Pa_t_trigger, 1, 'first');

%% =====================================================================
%% ANALYSE DYNAMIQUE DU PARACHUTE (ode45) — MODÈLE PRÉCIS
%% =====================================================================

% Vitesse et angle au moment du déploiement (repris de la simulation)
Pa_V0     = V_mag(idx_para);
Pa_vx0_pa = V_mag(idx_para) * cos(atan2(0, 1)); % réorienté horizontalement
Pa_vy0_pa = -sqrt(max(0, Pa_V0^2 - Pa_vx0_pa^2)); % composante descendante estimée

% On extrait directement vx et vy depuis la simulation au moment idx_para
% Reconstruction depuis les arrays X, Y, T_sim
if idx_para > 1
    Pa_vx0_pa = (X(idx_para) - X(idx_para-1)) / dt;
    Pa_vy0_pa = (Y(idx_para) - Y(idx_para-1)) / dt;
else
    Pa_vx0_pa = v0_final * cosd(theta0_final);
    Pa_vy0_pa = v0_final * sind(theta0_final);
end

Pa_x0vec = [0; Y(idx_para); Pa_vx0_pa; Pa_vy0_pa]; % état initial [x; y; vx; vy]

[Pa_t, Pa_X] = ode45(@(Pa_t_loc, Pa_x_loc) Pa_dyn_precise( ...
    Pa_t_loc, Pa_x_loc, ...
    Pa_m, g, rho, ...
    Pa_CdA0, Pa_CdAfull, ...
    Pa_tau, Pa_tauAlign), ...
    Pa_tspan, Pa_x0vec);

Pa_x  = Pa_X(:,1);
Pa_y  = Pa_X(:,2);
Pa_vx = Pa_X(:,3);
Pa_vy = Pa_X(:,4);

% Calcul des accélérations (post-traitement)
Pa_ax = zeros(size(Pa_t));
Pa_ay = zeros(size(Pa_t));
for Pa_k = 1:length(Pa_t)
    [Pa_ax(Pa_k), Pa_ay(Pa_k)] = Pa_accel_precise( ...
        Pa_t(Pa_k), Pa_vx(Pa_k), Pa_vy(Pa_k), ...
        Pa_m, g, rho, ...
        Pa_CdA0, Pa_CdAfull, ...
        Pa_tau, Pa_tauAlign);
end

% CdA effectif — modèle précis : démarre à Pa_CdA0, monte vers Pa_CdAfull
Pa_CdA_eff = Pa_CdA0 + (Pa_CdAfull - Pa_CdA0) .* (1 - exp(-Pa_t / Pa_tau));

% Vitesse terminale théorique
Pa_vt_theo = -sqrt((2 * Pa_m * g) / (rho * Pa_CdAfull));

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
    Pa_d     = Pa_chutes(Pa_i);
    Pa_idx_d = find(Pa_y <= (Pa_y(1) - Pa_d), 1, 'first');
    if ~isempty(Pa_idx_d)
        Pa_t_chute(Pa_i) = Pa_t(Pa_idx_d);
        Pa_x_chute(Pa_i) = Pa_x(Pa_idx_d);
    end
end

%% =====================================================================
%% AFFICHAGE DES RÉSULTATS
%% =====================================================================

fprintf('\n========================================================\n');
fprintf('       RÉSULTATS — OPTIMISATION\n');
fprintf('========================================================\n');
fprintf('  Temps de déclenchement (signal) : %.4f s\n', Pa_t_trigger);
fprintf('  Délai parachute                 : %.4f s\n', Pa_t_delay);
fprintf('  Déploiement effectif            : %.4f s\n', Pa_t_dep);
fprintf('  Distance finale                 : %.4f m\n', D);
fprintf('  Erreur vs cible (%.1f m)        : %.4f m\n', cible_Dist, abs(D - cible_Dist));

fprintf('\n========================================================\n');
fprintf('       RÉSULTATS — DYNAMIQUE DU PARACHUTE (précis)\n');
fprintf('========================================================\n');
fprintf('  CdA capsule seule (Pa_CdA0)     : %.4f m²\n', Pa_CdA0);
fprintf('  CdA parachute déployé (full)    : %.4f m²\n', Pa_CdAfull);
fprintf('  Vitesse terminale théorique     : %.2f m/s\n', Pa_vt_theo);
fprintf('  Vitesse verticale stabilisée    : %.2f m/s\n', Pa_vy_term);
fprintf('  Temps de stabilisation          : %.2f s\n',   Pa_t_term);
fprintf('  Décélération maximale           : %.2f m/s² à t = %.2f s\n', Pa_a_max, Pa_t_a_max);

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
%% FIGURE 1 : TRAJECTOIRE ANIMÉE
%% =====================================================================

fig1 = figure('Name', 'Trajectoire Parachute — Modèle Précis', ...
    'NumberTitle', 'off', 'Color', 'w', 'Position', [100 100 900 550]);

hold on; grid on;
xlabel('Distance X (m)');
ylabel('Hauteur Y (m)');
title(sprintf('Trajectoire | Déclenchement : %.3f s | Déploiement : %.3f s | Portée : %.2f m', ...
    Pa_t_trigger, Pa_t_dep, D));
axis([0 max(X)+0.5 0 max(Y)+1]);
xline(cible_Dist, '--r', sprintf('Cible : %.1f m', cible_Dist), ...
    'LabelHorizontalAlignment', 'right');

hTraj  = plot(NaN, NaN, 'b',  'LineWidth', 2);
hPoint = plot(NaN, NaN, 'ko', 'MarkerFaceColor', 'r', 'MarkerSize', 7);

flag_trigger_drawn = false;
flag_para_drawn    = false;

for i = 1:8:length(X)
    set(hTraj,  'XData', X(1:i), 'YData', Y(1:i));
    set(hPoint, 'XData', X(i),   'YData', Y(i));

    % Marqueur de déclenchement (signal envoyé)
    if i >= idx_trigger && idx_trigger > 0 && ~flag_trigger_drawn
        plot(X(idx_trigger), Y(idx_trigger), 'ro', ...
            'MarkerSize', 9, 'LineWidth', 2);
        text(X(idx_trigger)+0.1, Y(idx_trigger)+0.3, ...
            sprintf('Déclenchement\nt = %.3f s', Pa_t_trigger), ...
            'Color', 'r', 'FontWeight', 'bold', 'FontSize', 8);
        flag_trigger_drawn = true;
    end

    % Marqueur de déploiement effectif du parachute
    if i >= idx_para && idx_para > 0 && ~flag_para_drawn
        plot(X(idx_para), Y(idx_para), 'rx', 'MarkerSize', 12, 'LineWidth', 2.5);
        text(X(idx_para)+0.1, Y(idx_para)-0.5, ...
            sprintf('🪂 Déploiement\nt = %.3f s', Pa_t_dep), ...
            'Color', [0.8 0.2 0], 'FontWeight', 'bold', 'FontSize', 8);
        flag_para_drawn = true;
    end

    pause(0.01);
    drawnow;
end

%% =====================================================================
%% FIGURE 2 : ANALYSE PARACHUTE — MODÈLE PRÉCIS
%% =====================================================================

fig2 = figure('Name', 'Analyse Parachute — Modèle Précis', ...
    'Color', 'w', 'Position', [1020 100 850 700]);

% --- Subplot 1 : CdA effectif ---
subplot(3,1,1);
plot(Pa_t, Pa_CdA_eff, 'Color', [0.2 0.4 0.8], 'LineWidth', 1.8);
hold on;
yline(Pa_CdAfull, '--k', sprintf('CdA_{déployé} = %.4f m²', Pa_CdAfull), ...
    'LabelHorizontalAlignment', 'left', 'FontSize', 8);
yline(Pa_CdA0,    ':r',  sprintf('CdA_{capsule} = %.4f m²', Pa_CdA0), ...
    'LabelHorizontalAlignment', 'left', 'FontSize', 8);
grid on;
xlabel('Temps depuis déploiement (s)');
ylabel('C_d \times A (m²)');
title('Évolution du C_d \times A — inflation de Pa\_CdA0 vers Pa\_CdAfull');
xlim([0 Pa_tspan(2)]);

% --- Subplot 2 : Vitesses ---
subplot(3,1,2);
plot(Pa_t, Pa_vx, 'LineWidth', 1.5); hold on;
plot(Pa_t, Pa_vy, 'LineWidth', 1.5);
yline(Pa_vt_theo, '--k', sprintf('v_{term} = %.2f m/s', Pa_vt_theo), ...
    'LabelHorizontalAlignment', 'left', 'FontSize', 8);
xline(Pa_t_term, '--b', sprintf('t_{stab} = %.2f s', Pa_t_term), 'FontSize', 8);
grid on;
legend('v_x (horizontal)', 'v_y (vertical)', 'Location', 'best');
xlabel('Temps (s)'); ylabel('Vitesse (m/s)');
title('Vitesses lors du déploiement du parachute');

% --- Subplot 3 : Accélérations ---
subplot(3,1,3);
plot(Pa_t, Pa_ax, 'LineWidth', 1.5); hold on;
plot(Pa_t, Pa_ay, 'LineWidth', 1.5);
plot(Pa_t, Pa_a_norm, 'k--', 'LineWidth', 1.2);
xline(Pa_t_a_max, '--r', sprintf('a_{max} = %.2f m/s²', Pa_a_max), 'FontSize', 8);
grid on;
legend('a_x', 'a_y', '||a||', 'Location', 'best');
xlabel('Temps (s)'); ylabel('Accélération (m/s²)');
title('Accélérations du système — Phase parachute');

%% =====================================================================
%% FONCTIONS LOCALES
%% =====================================================================

%% --- Fonction coût : minimise l'écart de portée ---
function score = fonction_cout_tpara(t_p, v0, theta0, d_target, ...
    m, r_bal, Cd_bal, Pa_tau, Pa_tauAlign, ...
    Pa_CdA0, Pa_CdAfull, Pa_m, ...
    y0, rho, g, dt)

    if t_p <= 0 || t_p > 10
        score = 1e6;
        return;
    end

    [~, d_sim] = sim_physique_complete(v0, theta0, ...
        m, r_bal, Cd_bal, t_p, ...
        Pa_tau, Pa_tauAlign, ...
        Pa_CdA0, Pa_CdAfull, Pa_m, ...
        y0, rho, g, dt);

    score = (d_sim - d_target)^2;
end

%% --- Simulation balistique — modèle de parachute précis ---
function [y_max, x_final, X, Y, T, V_mag, A_mag, idx_para] = sim_physique_complete( ...
    v0, theta0_deg, m, r_bal, Cd_bal, t_para, ...
    Pa_tau, Pa_tauAlign, Pa_CdA0, Pa_CdAfull, Pa_m, ...
    y0, rho, g, dt)

    % Coefficient de traînée balistique (projectile seul)
    k_bal = 0.5 * rho * Cd_bal * (pi * r_bal^2) / m;

    vx = v0 * cosd(theta0_deg);
    vy = v0 * sind(theta0_deg);

    x = 0; y = y0; t = 0;

    X = x; Y = y; T = t;
    V_mag = sqrt(vx^2 + vy^2);
    A_mag = g;

    y_max    = y;
    idx_para = 0;

    while y >= 0

        if t < t_para
            %-- Phase balistique : traînée isotrope du projectile seul --
            v  = sqrt(vx^2 + vy^2);
            ax = -k_bal * v * vx;
            ay = -k_bal * v * vy - g;

        else
            %-- Phase parachute : modèle précis --
            if idx_para == 0
                idx_para = length(X);
            end

            % Temps écoulé depuis début du déploiement
            t_dep_loc = t - t_para;

            % ✅ CdA effectif : part de Pa_CdA0 (capsule seule), monte vers Pa_CdAfull
            %    Modèle script 2 : Pa_CdA0 + (Pa_CdAfull - Pa_CdA0)*(1 - exp(-t/tau))
            %    vs script 1     :            Pa_CdAfull              *(1 - exp(-t/tau))
            CdA_eff = Pa_CdA0 + (Pa_CdAfull - Pa_CdA0) * (1 - exp(-t_dep_loc / Pa_tau));

            % Traînée verticale (parachute résiste à la chute)
            Fdy = -0.5 * rho * CdA_eff * abs(vy) * vy;

            % Accélérations
            ax = -vx / Pa_tauAlign;      % Réorientation horizontale exponentielle
            ay =  Fdy / Pa_m - g;        % Traînée verticale + gravité

        end

        vx = vx + ax * dt;
        vy = vy + ay * dt;
        x  = x  + vx * dt;
        y  = y  + vy * dt;
        t  = t  + dt;

        X(end+1)     = x;                  %#ok<AGROW>
        Y(end+1)     = y;                  %#ok<AGROW>
        T(end+1)     = t;                  %#ok<AGROW>
        V_mag(end+1) = sqrt(vx^2 + vy^2); %#ok<AGROW>
        A_mag(end+1) = sqrt(ax^2 + ay^2); %#ok<AGROW>

        if y > y_max
            y_max = y;
        end
    end

    x_final = x;
end

%% --- Dérivée d'état pour ode45 (analyse dynamique parachute) ---
function dxdt = Pa_dyn_precise( ...
    t_loc, x_loc, ...
    Pa_m_loc, g_loc, rho_loc, ...
    Pa_CdA0_loc, Pa_CdAfull_loc, ...
    Pa_tau_loc, Pa_tauAlign_loc)

    vx_loc = x_loc(3);
    vy_loc = x_loc(4);

    % ✅ Même modèle précis : CdA part de Pa_CdA0 et monte vers Pa_CdAfull
    CdA_loc = Pa_CdA0_loc + (Pa_CdAfull_loc - Pa_CdA0_loc) * (1 - exp(-t_loc / Pa_tau_loc));

    Fdy_loc = -0.5 * rho_loc * CdA_loc * abs(vy_loc) * vy_loc;  % Traînée verticale (N)
    ax_loc  = -vx_loc / Pa_tauAlign_loc;                          % Réorientation horiz. (m/s²)
    ay_loc  =  Fdy_loc / Pa_m_loc - g_loc;                        % Traînée vert. + gravité (m/s²)

    dxdt = [vx_loc; vy_loc; ax_loc; ay_loc];
end

%% --- Accélérations instantanées (post-traitement ode45) ---
function [ax_out, ay_out] = Pa_accel_precise( ...
    t_loc, vx_loc, vy_loc, ...
    Pa_m_loc, g_loc, rho_loc, ...
    Pa_CdA0_loc, Pa_CdAfull_loc, ...
    Pa_tau_loc, Pa_tauAlign_loc)

    CdA_loc = Pa_CdA0_loc + (Pa_CdAfull_loc - Pa_CdA0_loc) * (1 - exp(-t_loc / Pa_tau_loc));

    Fdy_loc = -0.5 * rho_loc * CdA_loc * abs(vy_loc) * vy_loc;
    ax_out  = -vx_loc / Pa_tauAlign_loc;
    ay_out  =  Fdy_loc / Pa_m_loc - g_loc;
end
