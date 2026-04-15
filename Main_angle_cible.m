clear; clc; close all;

% =========================================================
% SIMULATION TRÉBUCHET — PHASE 1 (Rail) + PHASE 2 (Libre)
% =========================================================
%
% PHASE 1 — Contrainte holonome : projectile sur rail horizontal
%   Contrainte: h(θ,ψ) = l1·sin(θ) + l4·cos(ψ) − C_rail = 0
%
%   Résolution par multiplicateur de Lagrange (système KKT 4×4) :
%
%     [ M    −J' ] [ q̈ ]   [ F     ]
%     [ J     0  ] [ λ ] = [ γ     ]
%
%   avec J = ∂h/∂q = [l1·cos(θ), −l4·sin(ψ), 0]
%        γ  = l1·sin(θ)·θ̇² + l4·cos(ψ)·ψ̇²  (contrainte niveau accél.)
%        λ  = −N_rail  (force normale du rail sur le projectile)
%
%   Transition Phase 1→2 : λ = 0  (décollage, N_rail → 0)
%
% PHASE 2 — Trébuchet libre (votre code original)
% =========================================================

%% ─── Paramètres Physiques ───────────────────────────────────────────────
lb2kg = 0.454;
nb_trou = 13;

p.m      = 1.4;              % Masse projectile          [kg]
p.M      = 104.54;      % Masse contrepoids         [kg]
p.mbeam  = 14.0;             % Masse du bras             [kg]
p.lp     = 0.991;            % Bras non ajustable côté projectile  [m]
p.la     = 0.0762 * nb_trou;              % Bras ajustable co^té projectile     [m]
p.l1     = p.lp + p.la;      % Bras total côté projectcile         [m]
p.l2     = 0.559;            % Bras côté contrepoids     [m]
p.l3     = 0.4572;           % Pendule contrepoids       [m]
p.l4     = 1.2;            % Longueur de corde
p.g      = 9.81;             % Gravité                   [m/s²]
p.h_pivot = 2.1018;           % Hauteur du pivot          [m]
p.h_rail = 0.20;           % % Hauteur du pivot          [m]
p.Ibeam = 12;   % Inertie du bras [kg·m²]

%% ─── *** ANGLE DE SORTIE CIBLE *** ──────────────────────────────────────
% Angle que doit faire la vitesse de la capsule avec l'horizontale au lâcher [°]
% Modifiez cette valeur pour cibler un angle de lancement différent.
angle_sortie_cible = 25;   % [degrés]

%% ─── Conditions Initiales ────────────────────────────────────────────────
th0  = deg2rad(60);   % Angle du bras          [rad]
Adj = (p.h_pivot)-(p.h_rail)-(p.l1*sin(th0));
Oppo = p.l4;
ps0 = acos(Adj/Oppo); % Angle de la fronde     [rad]
ph0  = deg2rad(0);    % Angle du contrepoids   [rad]
dth0 = 0;  dps0 = 0;  dph0 = 0;

y0 = [th0; ps0; ph0; dth0; dps0; dph0];

% Vérification de cohérence de la contrainte initiale
vel_constraint_check = p.l1*cos(th0)*dth0 - p.l4*sin(ps0)*dps0;
if abs(vel_constraint_check) > 1e-10
    warning(['Conditions initiales incohérentes avec la contrainte rail.\n' ...
             'Erreur de contrainte vitesse: %.2e'], vel_constraint_check);
end

%% ─── Paramètres Rail (Phase 1) ───────────────────────────────────────────
p.C_rail = p.l1*sin(th0) + p.l4*cos(ps0);
p.Y_rail =  p.h_rail - p.h_pivot;

x0_proj = -p.l1*cos(th0) + p.l4*sin(ps0);

fprintf('╔══════════════════════════════════════════╗\n');
fprintf('║      SIMULATION TRÉBUCHET 2 PHASES       ║\n');
fprintf('╠══════════════════════════════════════════╣\n');
fprintf('║  Rail (Phase 1) :                        ║\n');
fprintf('║    Hauteur absolue  : %.3f m            ║\n', p.Y_rail);
fprintf('║    X initial proj.  : %.3f m            ║\n', x0_proj);
fprintf('╚══════════════════════════════════════════╝\n\n');

%% ══════════════════════════════════════════════════════════════════════
%%  PHASE 1 — Projectile Contraint sur le Rail
%% ══════════════════════════════════════════════════════════════════════
fprintf('► Phase 1 : Résolution avec contrainte rail...\n');

[~, lambda_init] = compute_lambda(y0, p);
if lambda_init >= 0
    warning('Force normale initiale ≤ 0 (λ=%.4f) — Phase 1 ignorée.', lambda_init);
    t1 = 0; y1 = y0'; te1 = 0; ye1 = y0';
else
    opts_p1 = odeset('RelTol', 1e-7, 'AbsTol', 1e-9, ...
                     'Events', @(t,y) rail_liftoff_event(t, y, p));
    [t1, y1, te1, ye1, ~] = ode45( ...
        @(t,y) trebuchet_phase1(t, y, p), [0, 1], y0, opts_p1);
end

%% ══════════════════════════════════════════════════════════════════════
%%  PHASE 2 — Trébuchet Libre (Système Non-Contraint)
%% ══════════════════════════════════════════════════════════════════════
opts_p2 = odeset('RelTol', 1e-6, 'AbsTol', 1e-8);

if ~isempty(te1)
    p.t_transition   = te1(1);
    p.idx_transition = length(t1);
    y_trans = ye1(1,:)';

    fprintf('   ✓ Décollage du rail à t = %.4f s\n', p.t_transition);
    fprintf('   Position X proj. au décollage : %.3f m\n', ...
        -p.l1*cos(y_trans(1)) + p.l4*sin(y_trans(2)));
    fprintf('\n► Phase 2 : Résolution libre...\n');

    [t2, y2] = ode45( ...
        @(t,y) trebuchet_dynamics(t, y, p), ...
        [p.t_transition, 1], y_trans, opts_p2);

    t = [t1;       t2(2:end)    ];
    y = [y1;       y2(2:end,:)  ];

else
    warning('Aucun décollage détecté — simulation entière en Phase 1.');
    p.t_transition   = inf;
    p.idx_transition = length(t1);
    t = t1;   y = y1;
end

n_p1 = p.idx_transition;
n_p2 = length(t) - n_p1;
fprintf('\nSimulation terminée : %d points au total\n', length(t));
fprintf('  Phase 1 : %d points (0 → %.4f s)\n', n_p1, p.t_transition);
fprintf('  Phase 2 : %d points (%.4f → %.3f s)\n\n', n_p2, p.t_transition, t(end));

%% ─── Vérification de la Contrainte en Phase 1 ────────────────────────
constraint_violation = p.l1*sin(y1(:,1)) + p.l4*cos(y1(:,2)) - p.C_rail;
fprintf('Violation max de contrainte en Phase 1 : %.2e m\n\n', ...
    max(abs(constraint_violation)));

%% ─── Force Normale du Rail (Phase 1) ─────────────────────────────────
N_rail = zeros(n_p1, 1);
for i = 1:n_p1
    [~, lambda_i] = compute_lambda(y1(i,:)', p);
    N_rail(i) = -lambda_i;
end

figure('Color', 'w', 'Name', 'Force Normale du Rail (Phase 1)');
plot(t1, N_rail, 'LineWidth', 2, 'Color', [0.1 0.5 0.9]);
hold on;
yline(0, 'r--', 'LineWidth', 1.5, 'Label', 'Décollage (N=0)');
xlabel('Temps [s]');
ylabel('Force Normale N_{rail} [N]');
title('Force Normale du Rail sur le Projectile — Phase 1');
grid on;

%% ─── Animation ───────────────────────────────────────────────────────
fprintf('Lancement de l''animation...\n');
animation(t, y, p);

%% ─── Trajectoire & Vitesse de Sortie ────────────────────────────────
[dist, haut] = calcul_trajectoire(t, y, p, angle_sortie_cible);

th  = y(:,1);   ps  = y(:,2);
dth = y(:,4);   dps = y(:,5);

vx      =  p.l1*sin(th).*dth + p.l4*cos(ps).*dps;
vy      = -p.l1*cos(th).*dth + p.l4*sin(ps).*dps;
v_norme = sqrt(vx.^2 + vy.^2);

%% ─── *** DÉTECTION DE L'ANGLE DE SORTIE CIBLE *** ───────────────────
% Calcul de l'angle instantané de la vitesse [degrés]
angle_vitesse = rad2deg(atan2(vy, vx));

% On cherche le moment où l'angle de vitesse passe par angle_sortie_cible,
% en ne cherchant que dans la région où la vitesse est significative
% (évite les instants initiaux où vx≈vy≈0).
v_seuil   = 0.5;                         % vitesse minimale [m/s]
idx_valid = find(v_norme > v_seuil);     % indices à vitesse significative

if isempty(idx_valid)
    warning('Vitesse toujours inférieure au seuil — angle non détectable.');
    idx = length(t);
else
    % Différence signée entre l'angle courant et la cible
    diff_angle = angle_vitesse(idx_valid) - angle_sortie_cible;

    % Cherche le passage par zéro (changement de signe) le plus proche de 0
    changements = find(diff(sign(diff_angle)) ~= 0);

    if ~isempty(changements)
        % Prend le premier passage décroissant (angle qui descend vers la cible)
        % car la vitesse angle décroît généralement au cours du lancer
        [~, ic] = min(abs(diff_angle(changements)));
        idx = idx_valid(changements(ic));
    else
        % Pas de passage exact : prend le point le plus proche de la cible
        [~, ic] = min(abs(diff_angle));
        idx = idx_valid(ic);
        warning(['Aucun passage exact par %.1f° — ' ...
                 'point le plus proche utilisé (%.2f°).'], ...
                 angle_sortie_cible, angle_vitesse(idx));
    end
end

v_max          = v_norme(idx);
angle_sortie_rad = atan2(vy(idx), vx(idx));
angle_sortie_deg = rad2deg(angle_sortie_rad);
x_proj = -p.l1*cos(th(idx)) + p.l4*sin(ps(idx));
y_proj = -p.l1*sin(th(idx)) - p.l4*cos(ps(idx));

fprintf('\n════════════════════════════════════\n');
fprintf('   RÉSULTATS DU LANCEMENT           \n');
fprintf('════════════════════════════════════\n');
fprintf('Angle cible              : %.2f °\n',   angle_sortie_cible);
fprintf('Angle de sortie réel     : %.2f °\n',   angle_sortie_deg);
fprintf('Vitesse de sortie        : %.2f m/s\n', v_max);
fprintf('Temps du lâcher          : %.3f s\n',   t(idx));
if t(idx) <= p.t_transition
    fprintf('Phase au lancement       : Phase 1 (sur rail)\n');
else
    fprintf('Phase au lancement       : Phase 2 (vol libre)\n');
end
fprintf('────────────────────────────────────\n');
fprintf('Position au lancement:\n');
fprintf('  x = %.3f m\n', x_proj);
fprintf('  y = %.3f m (abs.)\n', y_proj + p.h_pivot);
fprintf('════════════════════════════════════\n\n');

%% ─── Figure Vitesse & Angle ──────────────────────────────────────────
figure('Color', 'w', 'Name', 'Vitesse de sortie');

% Sous-figure 1 : norme de la vitesse
subplot(2,1,1);
plot(t, v_norme, 'LineWidth', 2, 'Color', [0.2 0.2 0.2]); hold on;
plot(t(idx), v_max, 'ro', 'MarkerSize', 12, 'LineWidth', 2, 'MarkerFaceColor', 'r');
if isfinite(p.t_transition)
    xline(p.t_transition, 'b--', 'LineWidth', 1.5, 'Label', 'Décollage rail');
end
xlabel('Temps [s]');
ylabel('Vitesse [m/s]');
title(sprintf('Vitesse de sortie : %.2f m/s à %.1f° (cible : %.1f°)', ...
              v_max, angle_sortie_deg, angle_sortie_cible));
legend('Vitesse instantanée', 'Point de lâcher', 'Location', 'best');
grid on;

% Sous-figure 2 : angle de la vitesse
subplot(2,1,2);
plot(t, angle_vitesse, 'LineWidth', 2, 'Color', [0.2 0.6 0.3]); hold on;
plot(t(idx), angle_sortie_deg, 'ro', 'MarkerSize', 12, 'LineWidth', 2, 'MarkerFaceColor', 'r');
yline(angle_sortie_cible, 'k--', 'LineWidth', 1.5, ...
      'Label', sprintf('Cible %.1f°', angle_sortie_cible));
if isfinite(p.t_transition)
    xline(p.t_transition, 'b--', 'LineWidth', 1.5, 'Label', 'Décollage rail');
end
xlabel('Temps [s]');
ylabel('Angle [°]');
title('Angle de la vitesse de la capsule avec l''horizontale');
legend('Angle instantané', 'Point de lâcher', 'Location', 'best');
grid on;

%% ─── Tension dans la Corde ───────────────────────────────────────────
N_pts   = length(t);
Tension = zeros(N_pts, 1);

for i = 1:N_pts
    th_i  = y(i,1);  ps_i  = y(i,2);  ph_i  = y(i,3);
    dth_i = y(i,4);  dps_i = y(i,5);  dph_i = y(i,6);

    [Mmat, F] = masse_et_force(th_i, ps_i, ph_i, dth_i, dps_i, dph_i, p);

    if i <= n_p1
        J     = [p.l1*cos(th_i), -p.l4*sin(ps_i), 0];
        gamma = p.l1*sin(th_i)*dth_i^2 + p.l4*cos(ps_i)*dps_i^2;
        sol   = [Mmat, -J'; J, 0] \ [F; gamma];
        accel = sol(1:3);
    else
        accel = Mmat \ F;
    end

    ddth_i = accel(1);
    ddps_i = accel(2);

    ax = -p.l1*cos(th_i)*dth_i^2 - p.l1*sin(th_i)*ddth_i ...
         - p.l4*sin(ps_i)*dps_i^2 + p.l4*cos(ps_i)*ddps_i;
    ay = -p.l1*sin(th_i)*dth_i^2 + p.l1*cos(th_i)*ddth_i ...
         - p.l4*cos(ps_i)*dps_i^2 - p.l4*sin(ps_i)*ddps_i;

    erx =  sin(ps_i);
    ery = -cos(ps_i);

    a_radial   = ax*erx + ay*ery;
    Tension(i) = p.m * (a_radial + p.g*ery);
end

fprintf('Tension max : %.1f N\n', max(Tension));
fprintf('Tension min : %.1f N\n', min(Tension));

figure('Color', 'w', 'Name', 'Tension dans la corde');
plot(t, Tension, 'LineWidth', 2, 'Color', [0.8 0.3 0.1]); hold on;
plot(t(idx), Tension(idx), 'ro', 'MarkerSize', 12, 'LineWidth', 2, ...
     'MarkerFaceColor', 'r');
if isfinite(p.t_transition)
    xline(p.t_transition, 'b--', 'LineWidth', 1.5, 'Label', 'Décollage rail');
end
xlabel('Temps [s]');
ylabel('Tension [N]');
title(sprintf('Tension dans la Fronde  (T = %.1f N au lâcher, angle cible %.1f°)', ...
              Tension(idx), angle_sortie_cible));
legend('Tension instantanée', 'Point de lâcher', 'Location', 'best');
grid on;


%% ╔══════════════════════════════════════════════════════════════════╗
%% ║                      FONCTIONS LOCALES                          ║
%% ╚══════════════════════════════════════════════════════════════════╝

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

    J = [p.l1*cos(th),  -p.l4*sin(ps),  0];
    gamma = p.l1*sin(th)*dth^2 + p.l4*cos(ps)*dps^2;

    A_aug = [Mmat,  -J';
             J,      0 ];
    b_aug = [F;   gamma];

    sol   = A_aug \ b_aug;
    accel = sol(1:3);

    dydt = [y(4:6); accel];
end

% ─────────────────────────────────────────────────────────────────────
%  ÉVÉNEMENT : Décollage du Rail  (λ = 0)
% ─────────────────────────────────────────────────────────────────────
function [value, isterminal, direction] = rail_liftoff_event(~, y, p)
    [~, lambda] = compute_lambda(y, p);
    value       = lambda;
    isterminal  = 1;
    direction   = 1;
end

% ─────────────────────────────────────────────────────────────────────
%  HELPER : Calcul du Multiplicateur de Lagrange λ
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
%  HELPER : Matrice de Masse M  et  Vecteur de Force F
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

    F = [F1; F2; F3];
end

%% ─── Accélérations Tangentielle et Centripète de la Capsule ─────────
a_tang = zeros(N_pts, 1);
a_cent = zeros(N_pts, 1);
a_tot  = zeros(N_pts, 1);

for i = 1:N_pts
    th_i  = y(i,1);  ps_i  = y(i,2);  ph_i  = y(i,3);
    dth_i = y(i,4);  dps_i = y(i,5);  dph_i = y(i,6);

    % ── Accélérations généralisées (réutilise le même système que la tension) ──
    [Mmat, F] = masse_et_force(th_i, ps_i, ph_i, dth_i, dps_i, dph_i, p);

    if i <= n_p1          % Phase 1 : système KKT avec contrainte rail
        J     = [p.l1*cos(th_i), -p.l4*sin(ps_i), 0];
        gamma = p.l1*sin(th_i)*dth_i^2 + p.l4*cos(ps_i)*dps_i^2;
        sol   = [Mmat, -J'; J, 0] \ [F; gamma];
        accel = sol(1:3);
    else                  % Phase 2 : dynamique libre
        accel = Mmat \ F;
    end

    ddth_i = accel(1);
    ddps_i = accel(2);

    % ── Vitesse cartésienne de la capsule ──────────────────────────────
    vx_i =  p.l1*sin(th_i)*dth_i + p.l4*cos(ps_i)*dps_i;
    vy_i = -p.l1*cos(th_i)*dth_i + p.l4*sin(ps_i)*dps_i;
    v_i  = sqrt(vx_i^2 + vy_i^2);

    % ── Accélération cartésienne de la capsule  (d/dt des vitesses) ───
    %   d(vx)/dt = l1·cos(θ)·θ̇² + l1·sin(θ)·θ̈  − l4·sin(ψ)·ψ̇² + l4·cos(ψ)·ψ̈
    %   d(vy)/dt = l1·sin(θ)·θ̇² − l1·cos(θ)·θ̈  + l4·cos(ψ)·ψ̇² + l4·sin(ψ)·ψ̈
    ax_i =  p.l1*cos(th_i)*dth_i^2 + p.l1*sin(th_i)*ddth_i ...
           - p.l4*sin(ps_i)*dps_i^2 + p.l4*cos(ps_i)*ddps_i;
    ay_i =  p.l1*sin(th_i)*dth_i^2 - p.l1*cos(th_i)*ddth_i ...
           + p.l4*cos(ps_i)*dps_i^2 + p.l4*sin(ps_i)*ddps_i;

    a_tot(i) = sqrt(ax_i^2 + ay_i^2);

    if v_i > 1e-6
        % Tangentielle  : projection de a sur v̂  (signée : + si accélération, − si freinage)
        a_tang(i) = (ax_i*vx_i + ay_i*vy_i) / v_i;

        % Centripète    : norme de la composante perpendiculaire à v̂  (toujours ≥ 0)
        %                 = |a × v̂|  (produit vectoriel 2-D, scalaire)
        a_cent(i) = abs(ay_i*vx_i - ax_i*vy_i) / v_i;
    else
        a_tang(i) = 0;
        a_cent(i) = 0;
    end
end

% ── Affichage console ──────────────────────────────────────────────────
fprintf('\n════════════════════════════════════\n');
fprintf('   ACCÉLÉRATIONS DE LA CAPSULE      \n');
fprintf('════════════════════════════════════\n');
fprintf('Accél. tangentielle max  : %7.2f m/s²\n', max( a_tang));
fprintf('Accél. tangentielle min  : %7.2f m/s²\n', min( a_tang));
fprintf('Accél. centripète max    : %7.2f m/s²\n', max( a_cent));
fprintf('Accél. totale max        : %7.2f m/s²\n', max( a_tot ));
fprintf('────────────────────────────────────\n');
fprintf('Au point de lâcher :\n');
fprintf('  Tangentielle  = %7.2f m/s²\n', a_tang(idx));
fprintf('  Centripète    = %7.2f m/s²\n', a_cent(idx));
fprintf('  Totale        = %7.2f m/s²\n', a_tot( idx));
fprintf('════════════════════════════════════\n\n');

% ── Figure ────────────────────────────────────────────────────────────
figure('Color', 'w', 'Name', 'Accélérations de la Capsule');

subplot(3,1,1);
plot(t, a_tang, 'LineWidth', 2, 'Color', [0.2 0.5 0.8]); hold on;
plot(t(idx), a_tang(idx), 'ro', 'MarkerSize', 10, 'LineWidth', 2, 'MarkerFaceColor', 'r');
yline(0, 'k--', 'LineWidth', 1);
if isfinite(p.t_transition)
    xline(p.t_transition, 'b--', 'LineWidth', 1.5, 'Label', 'Décollage rail');
end
xlabel('Temps [s]');  ylabel('a_{tang} [m/s²]');
title(sprintf('Accélération tangentielle  (%.2f m/s² au lâcher)', a_tang(idx)));
legend('a_{tang}', 'Lâcher', 'Location', 'best');
grid on;

subplot(3,1,2);
plot(t, a_cent, 'LineWidth', 2, 'Color', [0.85 0.33 0.10]); hold on;
plot(t(idx), a_cent(idx), 'ro', 'MarkerSize', 10, 'LineWidth', 2, 'MarkerFaceColor', 'r');
if isfinite(p.t_transition)
    xline(p.t_transition, 'b--', 'LineWidth', 1.5, 'Label', 'Décollage rail');
end
xlabel('Temps [s]');  ylabel('a_{cent} [m/s²]');
title(sprintf('Accélération centripète  (%.2f m/s² au lâcher)', a_cent(idx)));
legend('a_{cent}', 'Lâcher', 'Location', 'best');
grid on;

subplot(3,1,3);
plot(t, a_tot, 'LineWidth', 2, 'Color', [0.49 0.18 0.56]); hold on;
plot(t(idx), a_tot(idx), 'ro', 'MarkerSize', 10, 'LineWidth', 2, 'MarkerFaceColor', 'r');
if isfinite(p.t_transition)
    xline(p.t_transition, 'b--', 'LineWidth', 1.5, 'Label', 'Décollage rail');
end
xlabel('Temps [s]');  ylabel('a_{tot} [m/s²]');
title(sprintf('Accélération totale  ||a||  (%.2f m/s² au lâcher)', a_tot(idx)));
legend('a_{tot}', 'Lâcher', 'Location', 'best');
grid on;

sgtitle('Accélérations de la Capsule au cours du Lancement', 'FontSize', 13, 'FontWeight', 'bold');