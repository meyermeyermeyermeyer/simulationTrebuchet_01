function [dist_x, h_max] = calcul_trajectoire(t, y, p, alpha_target_deg)
    % alpha_target_deg : Angle du VECTEUR VITESSE au moment du lâcher (ex: 45)
    
    idx = 0;
    g = p.g;
    h_pivot_sol = 2; % Pivot est à 2m du sol dans ton code

    % 1. Trouver l'instant du lâcher
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
        error('L''angle de tir cible n''est pas atteint dans cette simulation.');
    end

    % 2. Conditions initiales du vol
    x0 = -p.l1*cos(y(idx,1)) + p.l4*sin(y(idx,2));
    y0 = -p.l1*sin(y(idx,1)) - p.l4*cos(y(idx,2));
    vx0 = p.l1*sin(y(idx,1))*y(idx,4) + p.l4*cos(y(idx,2))*y(idx,5);
    vy0 = -p.l1*cos(y(idx,1))*y(idx,4) + p.l4*sin(y(idx,2))*y(idx,5);
    
    % Hauteur réelle par rapport au sol
    h_relache = y0 + h_pivot_sol;
    
    % 3. Calculs Balistiques
    % Temps de vol : -0.5*g*t^2 + vy0*t + h_relache = 0
    t_vol = (vy0 + sqrt(vy0^2 + 2*g*h_relache)) / g;
    dist_x = x0 + vx0 * t_vol;
    h_max = h_relache + (vy0^2) / (2*g);

    % 4. Graphique de la parabole
    t_p = linspace(0, t_vol, 50);
    xp = x0 + vx0 * t_p;
    yp = y0 + vy0 * t_p - 0.5 * g * t_p.^2;

    figure('Color', 'w'); hold on; grid on;
    plot(xp, yp, 'r--', 'LineWidth', 2);
    plot(x0, y0, 'ko', 'MarkerFaceColor', 'k');
    line([-5, dist_x+5], [-h_pivot_sol, -h_pivot_sol], 'Color', 'k', 'LineWidth', 2);
    title(sprintf('Trajectoire : Portée = %.2f m | H_{max} = %.2f m', dist_x, h_max));
    xlabel('Distance (m)'); ylabel('Hauteur (m)');
    axis equal;
end
