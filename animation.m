function animation(t, y, p)
% ========================================
% ANIMATION SIMPLE DU TRÉBUCHET
% ========================================
% Version corrigée de votre fonction animation originale
% Compatible avec votre code existant

    fig = figure('Color', 'w', 'Name', 'Animation Trébuchet');
    hold on; axis equal; grid on;

    % --- DESSIN DU SOL ---
    sol_width = p.l1 + p.l4 + p.l2 + p.l3 + 2;
    line([-sol_width, sol_width], [-2, -2], 'Color', 'k', 'LineWidth', 2);
    plot(0, 0, 'ks', 'MarkerFaceColor', 'k', 'MarkerSize', 10);

    % --- INITIALISATION DES OBJETS ---
    h_beam  = plot(0,0, 'k', 'LineWidth', 2);
    h_sling = plot(0,0, 'k', 'LineWidth', 1);
    h_cw    = plot(0,0, 'k', 'LineWidth', 2);
    h_proj  = plot(0,0, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 4);
    h_mass  = plot(0,0, 'bs', 'MarkerFaceColor', 'b', 'MarkerSize', 8);
    h_traj  = plot(0,0, 'r:', 'LineWidth', 1.5);

    % AJOUT DE LA LÉGENDE ICI (Avant la boucle pour éviter l'erreur)
    legend([h_proj, h_mass], {'Projectile', 'Contrepoids'}, 'Location', 'northeast');

    % --- LIMITES DES AXES (CORRIGÉ: max avec vecteur) ---
    x_max = p.l1 + p.l4 + 1;
    y_max = max([p.l1, p.l2]) + 2;  % CORRECTION ICI
    y_min = -max([p.l1 + p.l4, p.l2 + p.l3]) - 1;  % CORRECTION ICI
    
    xlim([-5, x_max+5]); 
    ylim([y_min, y_max]);
    
    traj_x = []; 
    traj_y = [];

    for k = 1:length(t)
        if ~ishandle(fig), break; end % Arrêt si on ferme la fenêtre

        th = real(y(k,1)); 
        ps = real(y(k,2)); 
        ph = real(y(k,3));

        % Positions
        b_lx = -p.l1 * cos(th); 
        b_ly = -p.l1 * sin(th);
        b_rx =  p.l2 * cos(th); 
        b_ry =  p.l2 * sin(th);
        px = b_lx + p.l4 * sin(ps); 
        py = b_ly - p.l4 * cos(ps);
        mx = b_rx + p.l3 * sin(ph); 
        my = b_ry - p.l3 * cos(ph);

        % Mise à jour
        set(h_beam,  'XData', [b_lx, b_rx], 'YData', [b_ly, b_ry]);
        set(h_sling, 'XData', [b_lx, px],   'YData', [b_ly, py]);
        set(h_cw,    'XData', [b_rx, mx],   'YData', [b_ry, my]);
        set(h_proj,  'XData', px, 'YData', py);
        set(h_mass,  'XData', mx, 'YData', my);

        traj_x(end+1) = px; 
        traj_y(end+1) = py;
        set(h_traj, 'XData', traj_x, 'YData', traj_y);

        title(sprintf('Temps: %.2f s | \\theta=%.1f°', t(k), rad2deg(th)));
        drawnow;

        if k < length(t), pause(t(k+1)-t(k)); end
    end
end
