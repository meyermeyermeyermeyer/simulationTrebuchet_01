function animation_avec_liberation(t, y, p, alpha_target_deg)
% ========================================
% ANIMATION COMPLÈTE AVEC LIBÉRATION
% ========================================
% Animation du trébuchet en 2 phases sur la MÊME figure:
%   - Phase 1: Projectile attaché
%   - Phase 2: Vol balistique avec TRAÎNÉE AÉRODYNAMIQUE
%
% Inputs:
%   t : vecteur temps de la simulation
%   y : matrice états [theta, psi, phi, dtheta, dpsi, dphi]
%   p : structure des paramètres
%   alpha_target_deg : angle de libération souhaité [degrés] (ex: 45)
%
% Paramètres de traînée utilisés (ajustez dans p ou directement ici) :
%   p.m      : masse du projectile [kg]
%   p.r_bal  : rayon du projectile [m]       (défaut : 0.12 m)
%   p.Cd_bal : coefficient de traînée [-]    (défaut : 0.45)
%   p.rho    : densité de l'air [kg/m³]      (défaut : 1.2041)

    if nargin < 4
        alpha_target_deg = 20;
    end

    % ── Paramètres de traînée ──────────────────────────────────────
    rho    = 1.2041;
    r_bal  = 0.12;
    Cd_bal = 0.45;
    dt_bal = 0.005;   % pas d'intégration balistique [s]

    % Surcharge depuis p si disponibles
    if isfield(p, 'rho'),    rho    = p.rho;    end
    if isfield(p, 'r_bal'),  r_bal  = p.r_bal;  end
    if isfield(p, 'Cd_bal'), Cd_bal = p.Cd_bal; end

    k_bal = 0.5 * rho * Cd_bal * (pi * r_bal^2) / p.m;

    h_pivot_sol = p.h_pivot;
    g = p.g;

    % ========================================
    % 1. DÉTECTION DU MOMENT DE LIBÉRATION
    % ========================================
    idx_lib = 0;

    for k = 2:length(t)
        vx = p.l1*sin(y(k,1))*y(k,4) + p.l4*cos(y(k,2))*y(k,5);
        vy = -p.l1*cos(y(k,1))*y(k,4) + p.l4*sin(y(k,2))*y(k,5);
        angle_actuel = atan2d(vy, vx);

        if vy > 0 && angle_actuel <= alpha_target_deg
            idx_lib = k;
            break;
        end
    end

    if idx_lib == 0
        warning('Angle de libération %.1f° non atteint. Animation sans libération.', alpha_target_deg);
        animation_simple(t, y, p);
        return;
    end

    fprintf('✅ Libération détectée à t = %.3f s (angle = %.1f°)\n', t(idx_lib), alpha_target_deg);

    % ========================================
    % 2. TRAJECTOIRE BALISTIQUE AVEC TRAÎNÉE
    % ========================================

    % Position et vitesse à la libération
    th_lib = y(idx_lib,1);
    ps_lib = y(idx_lib,2);
    x0 = -p.l1*cos(th_lib) + p.l4*sin(ps_lib);
    y0 = -p.l1*sin(th_lib) - p.l4*cos(ps_lib);

    vx0 =  p.l1*sin(th_lib)*y(idx_lib,4) + p.l4*cos(ps_lib)*y(idx_lib,5);
    vy0 = -p.l1*cos(th_lib)*y(idx_lib,4) + p.l4*sin(ps_lib)*y(idx_lib,5);

    % Hauteur absolue au lâcher
    h_relache = y0 + h_pivot_sol;

    % ── Intégration numérique Euler avec traînée ──────────────────
    xb = x0;   yb = y0;
    vxb = vx0; vyb = vy0;

    X_bal = xb;
    Y_bal = yb;

    while (yb + h_pivot_sol) >= 0
        v   = sqrt(vxb^2 + vyb^2);
        axb = -k_bal * v * vxb;
        ayb = -k_bal * v * vyb - g;

        vxb = vxb + axb * dt_bal;
        vyb = vyb + ayb * dt_bal;
        xb  = xb  + vxb * dt_bal;
        yb  = yb  + vyb * dt_bal;

        X_bal(end+1) = xb;  %#ok<AGROW>
        Y_bal(end+1) = yb;  %#ok<AGROW>
    end

    dist_finale = xb;
    h_max_abs   = h_pivot_sol + max(Y_bal);

    % ── Référence sans traînée (pour comparaison) ──────────────────
    t_vol_ideal   = (vy0 + sqrt(vy0^2 + 2*g*h_relache)) / g;
    dist_sans_drag = x0 + vx0 * t_vol_ideal;

    fprintf('📊 RÉSULTATS:\n');
    fprintf('   Position libération      : (%.2f, %.2f) m\n', x0, y0);
    fprintf('   Vitesse libération       : %.2f m/s à %.1f°\n', sqrt(vx0^2+vy0^2), atan2d(vy0,vx0));
    fprintf('   Hauteur max absolue      : %.2f m\n', h_max_abs);
    fprintf('   Distance AVEC traînée    : %.2f m\n', dist_finale);
    fprintf('   Distance SANS traînée    : %.2f m  (référence idéale)\n', dist_sans_drag);
    fprintf('   Perte due à la traînée   : %.2f m\n\n', dist_sans_drag - dist_finale);

    % ========================================
    % 3. ANIMATION
    % ========================================

    fig = figure('Color', 'w', 'Name', 'Trébuchet avec Libération', 'Position', [100 100 1200 700]);
    hold on; axis equal; grid on;

    % Sol
    sol_width = max(dist_sans_drag + 3, p.l1 + p.l4 + 5);
    line([-2, sol_width], [-h_pivot_sol, -h_pivot_sol], 'Color', 'k', 'LineWidth', 3);

    % Pivot
    plot(0, 0, 'ks', 'MarkerFaceColor', 'k', 'MarkerSize', 12);

    % Objets graphiques
    h_beam  = plot(0, 0, 'k',  'LineWidth', 4);
    h_sling = plot(0, 0, 'k',  'LineWidth', 2);
    h_cw    = plot(0, 0, 'k',  'LineWidth', 2);
    h_proj  = plot(0, 0, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 12);
    h_mass  = plot(0, 0, 'bs', 'MarkerFaceColor', 'b', 'MarkerSize', 20);
    h_traj  = plot(0, 0, 'r:', 'LineWidth', 1.5);
    h_balist = plot(NaN, NaN, 'r-', 'LineWidth', 2.5);

    traj_x = []; traj_y = [];

    % Axes
    x_max_plot = max([dist_sans_drag + 3, p.l1 + p.l4 + 3]);
    y_max_plot = max([p.l1, p.l2, h_max_abs - h_pivot_sol]) + 1;
    y_min_plot = -max([p.l1 + p.l4, p.l2 + p.l3]) - 0.5;

    xlim([-2, x_max_plot]);
    ylim([y_min_plot, y_max_plot]);
    xlabel('Distance (m)', 'FontSize', 12);
    ylabel('Hauteur (m)',  'FontSize', 12);

    % ── PHASE 1 : Projectile attaché ──────────────────────────────
    for k = 1:idx_lib
        if ~ishandle(fig), return; end

        th = real(y(k,1));
        ps = real(y(k,2));
        ph = real(y(k,3));

        beam_lx =  p.l2 * cos(th);   beam_ly =  p.l2 * sin(th);
        beam_rx = -p.l1 * cos(th);   beam_ry = -p.l1 * sin(th);
        proj_x  = beam_rx + p.l4 * sin(ps);
        proj_y  = beam_ry - p.l4 * cos(ps);
        cw_x    = beam_lx + p.l3 * sin(ph);
        cw_y    = beam_ly - p.l3 * cos(ph);

        set(h_beam,  'XData', [beam_rx, beam_lx], 'YData', [beam_ry, beam_ly]);
        set(h_sling, 'XData', [beam_rx, proj_x],  'YData', [beam_ry, proj_y]);
        set(h_cw,    'XData', [beam_lx, cw_x],    'YData', [beam_ly, cw_y]);
        set(h_proj,  'XData', proj_x, 'YData', proj_y);
        set(h_mass,  'XData', cw_x,   'YData', cw_y);

        traj_x(end+1) = proj_x;  %#ok<AGROW>
        traj_y(end+1) = proj_y;  %#ok<AGROW>
        set(h_traj, 'XData', traj_x, 'YData', traj_y);

        title(sprintf('PHASE 1 — ATTACHÉ | t = %.2f s | \\theta = %.1f°', ...
            t(k), rad2deg(th)), 'FontSize', 14, 'FontWeight', 'bold');
        drawnow;

        if k < idx_lib
            pause(min(t(k+1)-t(k), 0.02));
        end
    end

    % ── PHASE 2 : Vol balistique avec traînée ─────────────────────
    set(h_sling, 'Visible', 'off');

    n_frames  = min(80, length(X_bal));
    idx_frames = round(linspace(1, length(X_bal), n_frames));

    for i = 1:length(idx_frames)
        if ~ishandle(fig), return; end

        k = idx_frames(i);
        set(h_proj,   'XData', X_bal(k), 'YData', Y_bal(k));
        set(h_balist, 'XData', X_bal(1:k), 'YData', Y_bal(1:k));

        title(sprintf('PHASE 2 — VOL LIBRE | H_{max} = %.2f m', ...
            h_max_abs), 'FontSize', 14, 'FontWeight', 'bold', 'Color', 'r');
        drawnow;

        if i < length(idx_frames), pause(0.03); end
    end

    % ── Affichage final ───────────────────────────────────────────
    title(sprintf('TERMINÉ | Dist. avec traînée : %.2f m |  Angle lib : %.1f°', ...
        dist_finale, alpha_target_deg), ...
        'FontSize', 12, 'FontWeight', 'bold', 'Color', [0 0.55 0]);

    legend([h_proj, h_mass, h_traj, h_balist], ...
        {'Projectile', 'Contrepoids', 'Phase attachée', 'Vol balistique (avec traînée)'}, ...
        'Location', 'northeast', 'FontSize', 11);

    fprintf('✅ Animation terminée!\n');
end

% ========================================
% ANIMATION SIMPLE (sans libération)
% ========================================
function animation_simple(t, y, p)
    fig = figure('Color', 'w', 'Name', 'Animation Trébuchet');
    hold on; axis equal; grid on;

    sol_width = p.l1 + p.l4 + p.l2 + p.l3 + 2;
    line([-sol_width, sol_width], [-p.h_pivot, -p.h_pivot], 'Color', 'k', 'LineWidth', 3);
    plot(0, 0, 'ks', 'MarkerFaceColor', 'k', 'MarkerSize', 10);

    h_beam  = plot(0,0,'k','LineWidth',4);
    h_sling = plot(0,0,'k','LineWidth',2);
    h_cw    = plot(0,0,'k','LineWidth',2);
    h_proj  = plot(0,0,'ro','MarkerFaceColor','r','MarkerSize',12);
    h_mass  = plot(0,0,'bs','MarkerFaceColor','b','MarkerSize',20);
    h_traj  = plot(0,0,'r:','LineWidth',1.5);

    traj_x = []; traj_y = [];

    xlim([-2, p.l1+p.l4+5]);
    ylim([-max([p.l1+p.l4, p.l2+p.l3])-1, max([p.l1,p.l2])+2]);

    for k = 1:length(t)
        if ~ishandle(fig), return; end

        th = real(y(k,1)); ps = real(y(k,2)); ph = real(y(k,3));

        b_lx = -p.l1*cos(th); b_ly = -p.l1*sin(th);
        b_rx =  p.l2*cos(th); b_ry =  p.l2*sin(th);
        px = b_lx + p.l4*sin(ps); py = b_ly - p.l4*cos(ps);
        mx = b_rx + p.l3*sin(ph); my = b_ry - p.l3*cos(ph);

        set(h_beam,  'XData', [b_lx, b_rx], 'YData', [b_ly, b_ry]);
        set(h_sling, 'XData', [b_lx, px],   'YData', [b_ly, py]);
        set(h_cw,    'XData', [b_rx, mx],   'YData', [b_ry, my]);
        set(h_proj,  'XData', px, 'YData', py);
        set(h_mass,  'XData', mx, 'YData', my);

        traj_x(end+1) = px; traj_y(end+1) = py; %#ok<AGROW>
        set(h_traj, 'XData', traj_x, 'YData', traj_y);

        title(sprintf('Temps: %.2f s | \\theta=%.1f°', t(k), rad2deg(th)));
        drawnow;

        if k < length(t), pause(min(t(k+1)-t(k), 0.02)); end
    end
end
