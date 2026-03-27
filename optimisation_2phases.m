% ========================================
% SCRIPT: optimisation_2phases.m
% ========================================
% Optimise la masse du contrepoids (M) et la longueur de fronde (l4)
% en utilisant la simulation complète à 2 phases :
%   Phase 1 — Projectile contraint sur rail horizontal (KKT)
%   Phase 2 — Trébuchet libre (dynamique non-contrainte)
% ========================================
clear; clc; close all;

% --- 1. CIBLES DÉSIRÉES ---
v_cible        = 18.1;
angle_cible_deg = 32.8;
angle_cible_rad = deg2rad(angle_cible_deg);

% --- 2. PARAMÈTRES FIXES ---
lb2kg = 0.454;

p.m      = 2.0;
p.mbeam  = 10.0;
p.l1     = 2.006;
p.l2     = 0.508;
p.l3     = 0.4572;
p.g      = 9.81;
p.h_pivot = 2.1018;
p.Ibeam  = 6.28;

% Conditions initiales (constantes durant l'optimisation)
y0 = [deg2rad(60); deg2rad(80); deg2rad(0); 0; 0; 0];

% --- 3. OPTIMISATION ---
% x = [Masse M (kg), Longueur l4 (m)]
x0 = [250*lb2kg, 0.998];

options = optimset('Display', 'iter', 'TolFun', 1e-6, 'TolX', 1e-6);

fonction_objectif = @(x) calculer_erreur_premier_pic(x, p, y0, v_cible, angle_cible_rad);

[x_opt, cout_min] = fminsearch(fonction_objectif, x0, options);

% --- 4. RÉSULTATS ET VÉRIFICATION FINALE ---
M_opt  = x_opt(1);
l4_opt = x_opt(2);

p.M  = M_opt;
p.l4 = l4_opt;

% Recalcul de la contrainte rail depuis les CI
p.C_rail = p.l1*sin(y0(1)) + p.l4*cos(y0(2));
p.Y_rail = p.h_pivot - p.C_rail;

% --- Phase 1 ---
opts_p1 = odeset('RelTol', 1e-7, 'AbsTol', 1e-9, ...
                 'Events', @(t,y) rail_liftoff_event(t, y, p));
[t1, y1, te1, ye1, ~] = ode45(@(t,y) trebuchet_phase1(t, y, p), [0, 1], y0, opts_p1);

% --- Phase 2 ---
opts_p2 = odeset('RelTol', 1e-6, 'AbsTol', 1e-8);
if ~isempty(te1)
    t_trans = te1(1);
    y_trans = ye1(1,:)';
    [t2, y2] = ode45(@(t,y) trebuchet_dynamics(t, y, p), [t_trans, 1], y_trans, opts_p2);
    t = [t1;      t2(2:end)   ];
    y = [y1;      y2(2:end,:) ];
else
    warning('Aucun décollage détecté lors de la vérification finale.');
    t_trans = inf;
    t = t1;  y = y1;
end

% --- Détection du premier pic de vitesse ---
th  = y(:,1);  ps  = y(:,2);
dth = y(:,4);  dps = y(:,5);

vx = p.l1*sin(th).*dth + p.l4*cos(ps).*dps;
vy = -p.l1*cos(th).*dth + p.l4*sin(ps).*dps;
V  = sqrt(vx.^2 + vy.^2);

indices_pics = find(diff(sign(diff(V))) < 0) + 1;
if isempty(indices_pics)
    idx = length(V);
else
    idx = indices_pics(1);
end

% --- Affichage des résultats ---
fprintf('\n╔══════════════════════════════════════════════╗\n');
fprintf('║       RÉSULTATS DE L''OPTIMISATION 2 PHASES   ║\n');
fprintf('╠══════════════════════════════════════════════╣\n');
fprintf('║  Masse du contrepoids (M) : %8.3f kg      ║\n', M_opt);
fprintf('║  Masse du contrepoids (M) : %8.3f lbs     ║\n', M_opt/lb2kg);
fprintf('║  Longueur de fronde  (l4) : %8.4f m       ║\n', l4_opt);
fprintf('╠══════════════════════════════════════════════╣\n');
fprintf('║  Vitesse au 1er pic       : %8.3f m/s     ║\n', V(idx));
fprintf('║  Angle  au 1er pic        : %8.2f °       ║\n', rad2deg(atan2(vy(idx), vx(idx))));
fprintf('║  Temps  du pic            : %8.4f s       ║\n', t(idx));
fprintf('╠══════════════════════════════════════════════╣\n');
if isfinite(t_trans)
    fprintf('║  Décollage du rail à      : %8.4f s       ║\n', t_trans);
    if t(idx) <= t_trans
        fprintf('║  Phase au lancement       : Phase 1 (rail) ║\n');
    else
        fprintf('║  Phase au lancement       : Phase 2 (libre) ║\n');
    end
else
    fprintf('║  Pas de décollage détecté                    ║\n');
end
fprintf('║  Coût final               : %8.4f         ║\n', cout_min);
fprintf('╚══════════════════════════════════════════════╝\n\n');

% --- Tracé de la vitesse ---
figure('Color','w','Name','Vitesse de sortie — Optimisation 2 phases');
plot(t, V, 'LineWidth', 2, 'Color', [0.2 0.2 0.2]); hold on;
plot(t(idx), V(idx), 'ro', 'MarkerSize', 12, 'LineWidth', 2, 'MarkerFaceColor','r');
if isfinite(t_trans)
    xline(t_trans, 'b--', 'LineWidth', 1.5, 'Label', 'Décollage rail');
end
xlabel('Temps [s]');
ylabel('Vitesse [m/s]');
title(sprintf('Vitesse de sortie : %.2f m/s à %.1f°', V(idx), ...
      rad2deg(atan2(vy(idx), vx(idx)))));
legend('Vitesse instantanée', 'Premier maximum', 'Location', 'best');
grid on;


%% ╔══════════════════════════════════════════════════════════════════╗
%% ║                       FONCTIONS LOCALES                         ║
%% ╚══════════════════════════════════════════════════════════════════╝

% ─────────────────────────────────────────────────────────────────────
%  FONCTION DE COÛT — Premier maximum de vitesse (simulation 2 phases)
% ─────────────────────────────────────────────────────────────────────
function erreur = calculer_erreur_premier_pic(x, p, y0, v_cible, angle_cible_rad)
    p.M  = x(1);
    p.l4 = x(2);

    % Gardes sur les bornes physiques
    if p.M < 10 || p.l4 < 0.1
        erreur = 1e12;
        return;
    end

    % Constante de contrainte rail recalculée à chaque itération
    % (l4 peut changer, donc C_rail aussi)
    p.C_rail = p.l1*sin(y0(1)) + p.l4*cos(y0(2));
    p.Y_rail = p.h_pivot - p.C_rail;

    % ── Phase 1 ────────────────────────────────────────────────────
    opts_p1 = odeset('RelTol', 1e-6, 'AbsTol', 1e-8, ...
                     'Events', @(t,y) rail_liftoff_event(t, y, p));
    try
        [t1, y1, te1, ye1, ~] = ode45( ...
            @(t,y) trebuchet_phase1(t, y, p), [0, 1], y0, opts_p1);
    catch
        erreur = 1e12;
        return;
    end

    % ── Phase 2 ────────────────────────────────────────────────────
    opts_p2 = odeset('RelTol', 1e-5, 'AbsTol', 1e-7);
    if ~isempty(te1)
        t_trans = te1(1);
        y_trans = ye1(1,:)';
        try
            [t2, y2] = ode45( ...
                @(t,y) trebuchet_dynamics(t, y, p), [t_trans, 1], y_trans, opts_p2);
        catch
            erreur = 1e12;
            return;
        end
        t_all = [t1;      t2(2:end)   ];
        y_all = [y1;      y2(2:end,:) ];
    else
        % Aucun décollage — on utilise seulement la Phase 1
        t_all = t1;
        y_all = y1;
    end

    % ── Calcul de la vitesse du projectile ─────────────────────────
    th  = y_all(:,1);  ps  = y_all(:,2);
    dth = y_all(:,4);  dps = y_all(:,5);

    vx = p.l1*sin(th).*dth + p.l4*cos(ps).*dps;
    vy = -p.l1*cos(th).*dth + p.l4*sin(ps).*dps;
    V  = sqrt(vx.^2 + vy.^2);

    % ── Détection du premier maximum local ─────────────────────────
    indices_pics = find(diff(sign(diff(V))) < 0) + 1;

    if isempty(indices_pics)
        % Pas encore de pic : pénalité + on évalue à la fin
        v_actuelle   = V(end);
        angle_actuel = atan2(vy(end), vx(end));
        penalite     = 500;
    else
        idx          = indices_pics(1);
        v_actuelle   = V(idx);
        angle_actuel = atan2(vy(idx), vx(idx));
        penalite     = 0;
    end

    % ── Fonction de coût ───────────────────────────────────────────
    err_v = ((v_actuelle - v_cible) / v_cible)^2 * 100;
    err_a = (angle_actuel - angle_cible_rad)^2 * 10;

    erreur = err_v + err_a + penalite;
end

% ─────────────────────────────────────────────────────────────────────
%  PHASE 2 : Dynamique Non-Contrainte
% ─────────────────────────────────────────────────────────────────────
function dydt = trebuchet_dynamics(~, y, p)
    th = y(1); ps = y(2); ph = y(3);
    dth = y(4); dps = y(5); dph = y(6);
    [M, F] = masse_et_force(th, ps, ph, dth, dps, dph, p);
    accel  = M \ F;
    dydt   = [y(4:6); accel];
end

% ─────────────────────────────────────────────────────────────────────
%  PHASE 1 : Dynamique avec Contrainte Rail (Système KKT)
% ─────────────────────────────────────────────────────────────────────
function dydt = trebuchet_phase1(~, y, p)
    th = y(1); ps = y(2); ph = y(3);
    dth = y(4); dps = y(5); dph = y(6);

    [Mmat, F] = masse_et_force(th, ps, ph, dth, dps, dph, p);

    % Jacobien de la contrainte : J = [l1·cos(θ), −l4·sin(ψ), 0]
    J     = [p.l1*cos(th),  -p.l4*sin(ps),  0];
    % Terme de Baumgarte (d²h/dt² = 0)
    gamma = p.l1*sin(th)*dth^2 + p.l4*cos(ps)*dps^2;

    % Système KKT 4×4
    A_aug = [Mmat,  -J';
             J,      0 ];
    b_aug = [F;   gamma];

    sol   = A_aug \ b_aug;
    accel = sol(1:3);

    dydt = [y(4:6); accel];
end

% ─────────────────────────────────────────────────────────────────────
%  ÉVÉNEMENT : Décollage du Rail (λ = 0)
% ─────────────────────────────────────────────────────────────────────
function [value, isterminal, direction] = rail_liftoff_event(~, y, p)
    [~, lambda] = compute_lambda(y, p);
    value       = lambda;
    isterminal  = 1;
    direction   = 1;   % λ : négatif → 0 (décollage)
end

% ─────────────────────────────────────────────────────────────────────
%  HELPER : Multiplicateur de Lagrange λ
% ─────────────────────────────────────────────────────────────────────
function [accel, lambda] = compute_lambda(y, p)
    th = y(1); ps = y(2); ph = y(3);
    dth = y(4); dps = y(5); dph = y(6);

    [Mmat, F] = masse_et_force(th, ps, ph, dth, dps, dph, p);

    J     = [p.l1*cos(th),  -p.l4*sin(ps),  0];
    gamma = p.l1*sin(th)*dth^2 + p.l4*cos(ps)*dps^2;

    A_aug = [Mmat, -J'; J, 0];
    b_aug = [F;  gamma];
    sol   = A_aug \ b_aug;

    accel  = sol(1:3);
    lambda = sol(4);
end

% ─────────────────────────────────────────────────────────────────────
%  HELPER : Matrice de Masse  et  Vecteur de Force Généralisée
% ─────────────────────────────────────────────────────────────────────
function [Mmat, F] = masse_et_force(th, ps, ph, dth, dps, dph, p)
    M11 = p.Ibeam + p.M*p.l2^2 + p.l1^2*p.m;
    M12 = -p.l1*p.l4*p.m*sin(ps - th);
    M13 =  p.M*p.l2*p.l3*sin(ph - th);
    M22 =  p.l4^2*p.m;
    M33 =  p.M*p.l3^2;

    Mmat = [M11, M12, M13;
            M12, M22,   0;
            M13,   0, M33];

    F1 = -p.M*p.g*p.l2*cos(th) ...
         - p.M*p.l2*p.l3*cos(ph - th)*dph^2 ...
         + p.g*p.l1*p.m*cos(th) ...
         + p.g*p.l1*p.mbeam*cos(th)/2 ...
         - p.g*p.l2*p.mbeam*cos(th)/2 ...
         + p.l1*p.l4*p.m*cos(ps - th)*dps^2;

    F2 = -p.l4*p.m*(p.g*sin(ps) + p.l1*cos(ps - th)*dth^2);

    F3 =  p.M*p.l3*(-p.g*sin(ph) + p.l2*cos(ph - th)*dth^2);

    F  = [F1; F2; F3];
end
