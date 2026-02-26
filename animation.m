function animation(t, y, p)
% ========================================================
% ANIMATION TRÉBUCHET - Version fluide (frames précalculées)
% ========================================================

    fig = figure('Color', [0.96 0.96 0.93], 'Name', 'Animation Trébuchet', ...
                 'Position', [50 50 1100 700]);
    ax = axes('Parent', fig);
    hold(ax, 'on');
    axis(ax, 'equal');
    grid(ax, 'on');
    ax.GridColor = [0.8 0.8 0.8];
    ax.GridAlpha = 0.5;
    ax.Color = [0.96 0.96 0.93];

    % -------------------------------------------------------
    % DIMENSIONS DE LA STRUCTURE
    % -------------------------------------------------------
    h_pivot = 1.8;
    base_w  = 1.2;
    pied_w  = 0.15;
    pied_h  = 0.12;
    sol_y   = -h_pivot;

    % -------------------------------------------------------
    % DESSIN STATIQUE : SOL + STRUCTURE EN A
    % -------------------------------------------------------
    sol_ext = p.l1 + p.l4 + 2;
    fill(ax, [-sol_ext sol_ext sol_ext -sol_ext], ...
         [sol_y sol_y sol_y-0.3 sol_y-0.3], ...
         [0.55 0.45 0.35], 'EdgeColor', 'none');
    line(ax, [-sol_ext, sol_ext], [sol_y, sol_y], ...
         'Color', [0.3 0.2 0.1], 'LineWidth', 2.5);

    draw_structure_A(ax, base_w, h_pivot, pied_w, pied_h, sol_y);

    % Pivot
    theta_circ = linspace(0, 2*pi, 40);
    r_piv = 0.07;
    fill(ax, r_piv*cos(theta_circ), r_piv*sin(theta_circ), ...
         [0.3 0.3 0.3], 'EdgeColor', 'k', 'LineWidth', 1.5);

    % -------------------------------------------------------
    % OBJETS DYNAMIQUES
    % -------------------------------------------------------
    h_beam_L1 = patch('XData', [0 0 0 0], 'YData', [0 0 0 0], ...
                      'FaceColor', [0.45 0.28 0.10], 'EdgeColor', [0.2 0.1 0.0], ...
                      'LineWidth', 1, 'Parent', ax);
    h_beam_L2 = patch('XData', [0 0 0 0], 'YData', [0 0 0 0], ...
                      'FaceColor', [0.45 0.28 0.10], 'EdgeColor', [0.2 0.1 0.0], ...
                      'LineWidth', 1, 'Parent', ax);

    h_sling   = line(ax, [0 0], [0 0], 'Color', [0.2 0.2 0.2], 'LineWidth', 1.5);
    h_cw_rope = line(ax, [0 0], [0 0], 'Color', [0.2 0.2 0.2], 'LineWidth', 1.5);

    % Contrepoids (cercle)
    h_mass = fill(ax, zeros(1,40), zeros(1,40), [0.25 0.25 0.30], ...
                  'EdgeColor', [0.1 0.1 0.15], 'LineWidth', 2);

    % Capsule (trapèze)
    h_proj_fill = patch('XData', zeros(1,4), 'YData', zeros(1,4), ...
                        'FaceColor', [0.75 0.75 0.80], 'EdgeColor', [0.3 0.3 0.35], ...
                        'LineWidth', 2, 'Parent', ax);

    % Trajectoire
    h_traj = line(ax, NaN, NaN, 'Color', [0.8 0.2 0.1], ...
                  'LineWidth', 1.2, 'LineStyle', ':', 'Marker', 'none');

    % Vecteur vitesse
    h_vel = quiver(ax, 0, 0, 0, 0, 0, ...
                   'Color', [0.1 0.6 0.1], 'LineWidth', 2.5, ...
                   'MaxHeadSize', 0.5, 'AutoScale', 'off');

    % -------------------------------------------------------
    % LÉGENDE ET TITRE
    % -------------------------------------------------------
    h_title = title(ax, '', 'FontSize', 13, 'FontWeight', 'bold');
    xlabel(ax, 'Position horizontale [m]', 'FontSize', 11);
    ylabel(ax, 'Hauteur [m]',              'FontSize', 11);

    lp1 = patch('XData', NaN, 'YData', NaN, 'FaceColor', [0.75 0.75 0.80], ...
                'EdgeColor', [0.3 0.3 0.35], 'Parent', ax);
    lp2 = fill(ax, NaN, NaN, [0.25 0.25 0.30], 'EdgeColor', [0.1 0.1 0.15]);
    lp3 = line(ax, NaN, NaN, 'Color', [0.8 0.2 0.1], 'LineStyle', ':', 'LineWidth', 1.5);
    lp4 = quiver(ax, NaN, NaN, NaN, NaN, 'Color', [0.1 0.6 0.1], 'LineWidth', 2);
    legend(ax, [lp1 lp2 lp3 lp4], ...
           {'Capsule', 'Contrepoids', 'Trajectoire', 'Vitesse'}, ...
           'Location', 'northeast', 'FontSize', 10, 'Color', [1 1 1 0.85]);

    % Axes
    marge = 0.5;
    xlim(ax, [-(p.l1 + p.l4 + marge),  p.l1 + p.l4 + marge]);
    ylim(ax, [sol_y - 0.4,  p.l1 + p.l4 + 1]);

    % -------------------------------------------------------
    % PRÉ-CALCUL DE TOUTES LES FRAMES
    % -------------------------------------------------------
    dt      = 0.016;
    t_anim  = t(1);
    t_final = t(end);
    frames  = struct();
    k       = 0;

    fprintf('Précalcul des frames...\n');

    while t_anim <= t_final
        idx = find(t <= t_anim, 1, 'last');
        if isempty(idx) || idx >= length(t); break; end

        t1 = t(idx); t2 = t(idx+1);
        a  = (t_anim - t1) / (t2 - t1);

        k = k + 1;
        frames(k).th  = (1-a)*real(y(idx,1)) + a*real(y(idx+1,1));
        frames(k).ps  = (1-a)*real(y(idx,2)) + a*real(y(idx+1,2));
        frames(k).ph  = (1-a)*real(y(idx,3)) + a*real(y(idx+1,3));
        frames(k).dth = (1-a)*real(y(idx,4)) + a*real(y(idx+1,4));
        frames(k).dps = (1-a)*real(y(idx,5)) + a*real(y(idx+1,5));
        frames(k).t   = t_anim;

        t_anim = t_anim + dt;
    end

    fprintf('✅ %d frames précalculées\n', k);

    % Dimensions capsule
    wb = 0.12;   % demi-largeur base
    wt = 0.04;   % demi-largeur sommet
    hc = 0.22;   % hauteur capsule
    r_cw = 0.20; % rayon contrepoids
    bw   = 0.04; % épaisseur bras

    % -------------------------------------------------------
    % BOUCLE D'ANIMATION (dessin uniquement)
    % -------------------------------------------------------
    traj_x = [];
    traj_y = [];

    for i = 1:length(frames)
        if ~ishandle(fig); break; end

        th  = frames(i).th;
        ps  = frames(i).ps;
        ph  = frames(i).ph;
        dth = frames(i).dth;
        dps = frames(i).dps;

        % --- Positions clés ---
        b_lx = -p.l1 * cos(th);
        b_ly = -p.l1 * sin(th);
        b_rx =  p.l2 * cos(th);
        b_ry =  p.l2 * sin(th);
        px = b_lx + p.l4 * sin(ps);
        py = b_ly - p.l4 * cos(ps);
        mx = b_rx + p.l3 * sin(ph);
        my = b_ry - p.l3 * cos(ph);

        % --- Bras ---
        nx = -sin(th); ny = cos(th);
        set(h_beam_L1, ...
            'XData', [0+bw*nx, b_lx+bw*nx, b_lx-bw*nx, 0-bw*nx], ...
            'YData', [0+bw*ny, b_ly+bw*ny, b_ly-bw*ny, 0-bw*ny]);
        set(h_beam_L2, ...
            'XData', [0+bw*nx, b_rx+bw*nx, b_rx-bw*nx, 0-bw*nx], ...
            'YData', [0+bw*ny, b_ry+bw*ny, b_ry-bw*ny, 0-bw*ny]);

        % --- Fronde et corde ---
        set(h_sling,   'XData', [b_lx, px], 'YData', [b_ly, py]);
        set(h_cw_rope, 'XData', [b_rx, mx], 'YData', [b_ry, my]);

        % --- Contrepoids (cercle) ---
        ang_c = linspace(0, 2*pi, 40);
        set(h_mass, 'XData', mx + r_cw*cos(ang_c), ...
                    'YData', my + r_cw*sin(ang_c));

        % --- Capsule (trapèze orienté) ---
        vx_p   = p.l1*sin(th)*dth + p.l4*cos(ps)*dps;
        vy_p   = -p.l1*cos(th)*dth + p.l4*sin(ps)*dps;
        v_norm = sqrt(vx_p^2 + vy_p^2);

        if v_norm > 0.1
            ang_vol = atan2(vy_p, vx_p) + pi;
        else
            ang_vol = ps + pi/2;
        end

        R = [cos(ang_vol), -sin(ang_vol);
             sin(ang_vol),  cos(ang_vol)];

        trap_x = [-wb,  wb,  wt, -wt];
        trap_y = [  0,   0,  hc,  hc];
        pts = R * [trap_x; trap_y];
        set(h_proj_fill, 'XData', px + pts(1,:), ...
                         'YData', py + pts(2,:));

        % --- Trajectoire ---
        traj_x(end+1) = px; %#ok<AGROW>
        traj_y(end+1) = py; %#ok<AGROW>
        set(h_traj, 'XData', traj_x, 'YData', traj_y);

        % --- Vecteur vitesse ---
        if v_norm > 1
            set(h_vel, 'XData', px, 'YData', py, ...
                       'UData', 0.05*vx_p, 'VData', 0.05*vy_p, ...
                       'Visible', 'on');
        else
            set(h_vel, 'Visible', 'off');
        end

        % --- Titre ---
        set(h_title, 'String', ...
            sprintf('t = %.3f s  |  \\theta = %.1f°  |  v = %.1f m/s', ...
                    frames(i).t, rad2deg(th), v_norm));

        drawnow;
        pause(dt);
    end
end


% ============================================================
% Sous-fonction : structure en A
% ============================================================
function draw_structure_A(ax, base_w, h_piv, pied_w, pied_h, sol_y)

    col_struct  = [0.25 0.25 0.25];
    col_soudure = [0.15 0.15 0.15];
    lw = 3.5;

    pivot  = [0, 0];
    pied_L = [-base_w, sol_y];
    pied_R = [ base_w, sol_y];

    line(ax, [pivot(1), pied_L(1)], [pivot(2), pied_L(2)], ...
         'Color', col_struct, 'LineWidth', lw);
    line(ax, [pivot(1), pied_R(1)], [pivot(2), pied_R(2)], ...
         'Color', col_struct, 'LineWidth', lw);

    mid_y  = sol_y + h_piv * 0.35;
    mid_Lx = -base_w * (1 - 0.35);
    mid_Rx =  base_w * (1 - 0.35);
    line(ax, [mid_Lx, mid_Rx], [mid_y, mid_y], ...
         'Color', col_struct, 'LineWidth', lw*0.7);

    off = 0.06;
    line(ax, [pivot(1)+off, pied_L(1)+off], [pivot(2), pied_L(2)], ...
         'Color', col_soudure, 'LineWidth', lw*0.5);
    line(ax, [pivot(1)-off, pied_R(1)-off], [pivot(2), pied_R(2)], ...
         'Color', col_soudure, 'LineWidth', lw*0.5);

    sabot_col = [0.35 0.35 0.35];
    fill(ax, [pied_L(1)-pied_w*2, pied_L(1)+pied_w*2, ...
              pied_L(1)+pied_w*2, pied_L(1)-pied_w*2], ...
             [sol_y, sol_y, sol_y-pied_h, sol_y-pied_h], ...
             sabot_col, 'EdgeColor', 'k', 'LineWidth', 1);
    fill(ax, [pied_R(1)-pied_w*2, pied_R(1)+pied_w*2, ...
              pied_R(1)+pied_w*2, pied_R(1)-pied_w*2], ...
             [sol_y, sol_y, sol_y-pied_h, sol_y-pied_h], ...
             sabot_col, 'EdgeColor', 'k', 'LineWidth', 1);
end
