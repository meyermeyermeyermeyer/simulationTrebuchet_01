function animation_avec_liberation(t, y, p, alpha_target_deg)
% ========================================
% ANIMATION COMPLÈTE AVEC LIBÉRATION
% ========================================
% Animation du trébuchet en 2 phases sur la MÊME figure:
%   - Phase 1: Projectile attaché
%   - Phase 2: Vol balistique (courbe tracée en direct)
%
% Inputs:
%   t : vecteur temps de la simulation
%   y : matrice états [theta, psi, phi, dtheta, dpsi, dphi]
%   p : structure des paramètres
%   alpha_target_deg : angle de libération souhaité [degrés] (ex: 45)

    if nargin < 4
        alpha_target_deg = 45; % Valeur par défaut
    end
    
    h_pivot_sol = 2; % Hauteur du pivot par rapport au sol [m]
    g = p.g;
    
    % ========================================
    % 1. DÉTECTION DU MOMENT DE LIBÉRATION
    % ========================================
    idx_lib = 0;
    
    for k = 2:length(t)
        % Vitesse du projectile
        vx = p.l1*sin(y(k,1))*y(k,4) + p.l4*cos(y(k,2))*y(k,5);
        vy = -p.l1*cos(y(k,1))*y(k,4) + p.l4*sin(y(k,2))*y(k,5);
        angle_actuel = atan2d(vy, vx);
        
        % Condition de libération: vy > 0 ET angle <= alpha_target
        if vy > 0 && angle_actuel <= alpha_target_deg
            idx_lib = k;
            break;
        end
    end
    
    if idx_lib == 0
        warning('Angle de libération %.1f° non atteint. Animation sans libération.', alpha_target_deg);
        % Animation simple sans libération
        animation_simple(t, y, p);
        return;
    end
    
    fprintf('✅ Libération détectée à t = %.3f s (angle = %.1f°)\n', t(idx_lib), alpha_target_deg);
    
    % ========================================
    % 2. CALCUL DE LA TRAJECTOIRE BALISTIQUE
    % ========================================
    
    % Position à la libération
    th_lib = y(idx_lib,1);
    ps_lib = y(idx_lib,2);
    x0 = -p.l1*cos(th_lib) + p.l4*sin(ps_lib);
    y0 = -p.l1*sin(th_lib) - p.l4*cos(ps_lib);
    
    % Vitesse à la libération
    vx0 = p.l1*sin(th_lib)*y(idx_lib,4) + p.l4*cos(ps_lib)*y(idx_lib,5);
    vy0 = -p.l1*cos(th_lib)*y(idx_lib,4) + p.l4*sin(ps_lib)*y(idx_lib,5);
    
    % Hauteur par rapport au sol
    h_relache = y0 + h_pivot_sol;
    
    % Temps de vol jusqu'au sol
    t_vol = (vy0 + sqrt(vy0^2 + 2*g*h_relache)) / g;
    
    % Distance finale
    dist_finale = x0 + vx0 * t_vol;
    
    % Hauteur maximale
    h_max = h_relache + (vy0^2) / (2*g);
    
    % Trajectoire balistique (points pour tracer)
    n_points_balist = 50;
    t_balist = linspace(0, t_vol, n_points_balist);
    x_balist = x0 + vx0 * t_balist;
    y_balist = y0 + vy0 * t_balist - 0.5 * g * t_balist.^2;
    
    fprintf('📊 RÉSULTATS:\n');
    fprintf('   Position libération: (%.2f, %.2f) m\n', x0, y0);
    fprintf('   Vitesse libération: %.2f m/s à %.1f°\n', sqrt(vx0^2+vy0^2), atan2d(vy0,vx0));
    fprintf('   Hauteur max: %.2f m\n', h_max);
    fprintf('   Distance finale: %.2f m\n', dist_finale);
    fprintf('   Temps vol: %.3f s\n\n', t_vol);
    
    % ========================================
    % 3. ANIMATION
    % ========================================
    
    fig = figure('Color', 'w', 'Name', 'Trébuchet avec Libération', 'Position', [100 100 1200 700]);
    hold on; axis equal; grid on;
    
    % --- SOL ---
    sol_width = max(dist_finale + 2, p.l1 + p.l4 + 5);
    line([-2, sol_width], [-h_pivot_sol, -h_pivot_sol], 'Color', 'k', 'LineWidth', 3);
    
    % Pivot
    plot(0, 0, 'ks', 'MarkerFaceColor', 'k', 'MarkerSize', 12);
    
    % --- OBJETS GRAPHIQUES ---
    h_beam  = plot(0,0, 'k', 'LineWidth', 4);
    h_sling = plot(0,0, 'k', 'LineWidth', 2);
    h_cw    = plot(0,0, 'k', 'LineWidth', 2);
    h_proj  = plot(0,0, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 12);
    h_mass  = plot(0,0, 'bs', 'MarkerFaceColor', 'b', 'MarkerSize', 20);
    h_traj  = plot(0,0, 'r:', 'LineWidth', 1.5);
    
    % Trajectoire balistique (sera remplie progressivement)
    h_balist = plot(NaN, NaN, 'r--', 'LineWidth', 2.5);
    
    traj_x = []; traj_y = [];
    
    % --- LIMITES DES AXES ---
    x_max_plot = max([dist_finale + 2, p.l1 + p.l4 + 3]);
    y_max = max([p.l1, p.l2, h_max - h_pivot_sol]) + 1;
    y_min = -max([p.l1 + p.l4, p.l2 + p.l3]) - 0.5;
    
    xlim([-2, x_max_plot]);
    ylim([y_min, y_max]);
    xlabel('Distance (m)', 'FontSize', 12);
    ylabel('Hauteur (m)', 'FontSize', 12);
    
    % ========================================
    % PHASE 1: PROJECTILE ATTACHÉ
    % ========================================
    
    for k = 1:idx_lib
        if ~ishandle(fig), return; end
        
        th = real(y(k,1));
        ps = real(y(k,2));
        ph = real(y(k,3));
        
        % Positions
        beam_left_x = -p.l1 * cos(th);
        beam_left_y = -p.l1 * sin(th);
        beam_right_x = p.l2 * cos(th);
        beam_right_y = p.l2 * sin(th);
        
        proj_x = beam_left_x + p.l4 * sin(ps);
        proj_y = beam_left_y - p.l4 * cos(ps);
        
        cw_x = beam_right_x + p.l3 * sin(ph);
        cw_y = beam_right_y - p.l3 * cos(ph);
        
        % Mise à jour
        set(h_beam,  'XData', [beam_left_x, beam_right_x], 'YData', [beam_left_y, beam_right_y]);
        set(h_sling, 'XData', [beam_left_x, proj_x], 'YData', [beam_left_y, proj_y]);
        set(h_cw,    'XData', [beam_right_x, cw_x], 'YData', [beam_right_y, cw_y]);
        set(h_proj,  'XData', proj_x, 'YData', proj_y);
        set(h_mass,  'XData', cw_x, 'YData', cw_y);
        
        % Trajectoire phase 1
        traj_x(end+1) = proj_x;
        traj_y(end+1) = proj_y;
        set(h_traj, 'XData', traj_x, 'YData', traj_y);
        
        title(sprintf('PHASE 1 - ATTACHÉ | Temps: %.2f s | \\theta=%.1f°', ...
                      t(k), rad2deg(th)), 'FontSize', 14, 'FontWeight', 'bold');
        drawnow;
        
        if k < idx_lib
            pause(min(t(k+1)-t(k), 0.02));
        end
    end
    
    % ========================================
    % PHASE 2: VOL BALISTIQUE
    % ========================================
    
    % Masquer la fronde (projectile libéré)
    set(h_sling, 'Visible', 'off');
    
    % Animation du vol balistique
    n_frames_balist = min(50, length(t_balist));
    idx_balist = round(linspace(1, length(t_balist), n_frames_balist));
    
    for i = 1:length(idx_balist)
        if ~ishandle(fig), return; end
        
        k = idx_balist(i);
        
        % Position du projectile
        proj_x = x_balist(k);
        proj_y = y_balist(k);
        
        % Mise à jour projectile
        set(h_proj, 'XData', proj_x, 'YData', proj_y);
        
        % Trajectoire balistique (tracer progressivement)
        set(h_balist, 'XData', x_balist(1:k), 'YData', y_balist(1:k));
        
        % Titre
        t_actuel = t(idx_lib) + t_balist(k);
        title(sprintf('PHASE 2 - VOL LIBRE | Temps: %.2f s | H_{max}=%.2f m', ...
                      t_actuel, h_max), 'FontSize', 14, 'FontWeight', 'bold', 'Color', 'r');
        drawnow;
        
        if i < length(idx_balist)
            pause(0.03);
        end
    end
    
    % ========================================
    % AFFICHAGE FINAL
    % ========================================
    
    title(sprintf('TERMINÉ | Distance: %.2f m | H_{max}: %.2f m | Angle lib: %.1f°', ...
                  dist_finale, h_max, alpha_target_deg), ...
          'FontSize', 14, 'FontWeight', 'bold', 'Color', 'g');
    
    % Légende
    legend([h_proj, h_mass, h_traj, h_balist], ...
           {'Projectile', 'Contrepoids', 'Phase attachée', 'Vol balistique'}, ...
           'Location', 'northeast', 'FontSize', 11);
    
    fprintf('✅ Animation terminée!\n');
end

% ========================================
% ANIMATION SIMPLE (sans libération)
% ========================================
function animation_simple(t, y, p)
    % Animation basique si pas de libération
    
    fig = figure('Color', 'w', 'Name', 'Animation Trébuchet');
    hold on; axis equal; grid on;
    
    sol_width = p.l1 + p.l4 + p.l2 + p.l3 + 2;
    line([-sol_width, sol_width], [-2, -2], 'Color', 'k', 'LineWidth', 3);
    plot(0, 0, 'ks', 'MarkerFaceColor', 'k', 'MarkerSize', 10);
    
    h_beam  = plot(0,0, 'k', 'LineWidth', 4);
    h_sling = plot(0,0, 'k', 'LineWidth', 2);
    h_cw    = plot(0,0, 'k', 'LineWidth', 2);
    h_proj  = plot(0,0, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 12);
    h_mass  = plot(0,0, 'bs', 'MarkerFaceColor', 'b', 'MarkerSize', 20);
    h_traj  = plot(0,0, 'r:', 'LineWidth', 1.5);
    
    traj_x = []; traj_y = [];
    
    x_max = p.l1 + p.l4 + 1;
    y_max = max([p.l1, p.l2]) + 2;
    y_min = -max([p.l1 + p.l4, p.l2 + p.l3]) - 1;
    xlim([-2, x_max+5]); ylim([y_min, y_max]);
    
    for k = 1:length(t)
        if ~ishandle(fig), return; end
        
        th = real(y(k,1)); ps = real(y(k,2)); ph = real(y(k,3));
        
        b_lx = -p.l1 * cos(th); b_ly = -p.l1 * sin(th);
        b_rx =  p.l2 * cos(th); b_ry =  p.l2 * sin(th);
        px = b_lx + p.l4 * sin(ps); py = b_ly - p.l4 * cos(ps);
        mx = b_rx + p.l3 * sin(ph); my = b_ry - p.l3 * cos(ph);
        
        set(h_beam,  'XData', [b_lx, b_rx], 'YData', [b_ly, b_ry]);
        set(h_sling, 'XData', [b_lx, px],   'YData', [b_ly, py]);
        set(h_cw,    'XData', [b_rx, mx],   'YData', [b_ry, my]);
        set(h_proj,  'XData', px, 'YData', py);
        set(h_mass,  'XData', mx, 'YData', my);
        
        traj_x(end+1) = px; traj_y(end+1) = py;
        set(h_traj, 'XData', traj_x, 'YData', traj_y);
        
        title(sprintf('Temps: %.2f s | \\theta=%.1f°', t(k), rad2deg(th)));
        drawnow;
        
        if k < length(t), pause(min(t(k+1)-t(k), 0.02)); end
    end
end
