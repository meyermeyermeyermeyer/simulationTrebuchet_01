% ========================================
% SCRIPT: optimisation_masse_Lcorde.m
% ========================================
clear; clc; close all;

% --- 1. CIBLES DÉSIRÉES ---
v_cible = 15.0;               
angle_cible_deg = 31.0;       
angle_cible_rad = deg2rad(angle_cible_deg);

% --- 2. PARAMÈTRES FIXES ---
p.m = 2.0;         
p.mbeam = 10.0;    
p.l1 = 0.915;      
p.l2 = 0.48;       
p.l3 = 0.435;      
p.g = 9.81;        
p.Ibeam = (1/3)*p.mbeam*(p.l1^2 + p.l2^2);
y0 = [deg2rad(60); deg2rad(80); 0; 0; 0; 0];

% --- 3. OPTIMISATION ---
% x = [Masse M, Longueur l4]
x0 = [250.0, 0.85]; 

options = optimset('Display', 'iter', 'TolFun', 1e-6, 'TolX', 1e-6);
fonction_objectif = @(x) calculer_erreur_premier_pic(x, p, y0, v_cible, angle_cible_rad);

[x_opt, cout_min] = fminsearch(fonction_objectif, x0, options);

% --- 4. RÉSULTATS ET VÉRIFICATION ---
M_opt = x_opt(1);
l4_opt = x_opt(2);

p.M = M_opt; p.l4 = l4_opt;
[t, y] = ode45(@(t,y) trebuchet_dynamics(t, y, p), [0 0.8], y0);

% Calcul du premier pic pour l'affichage final
th = y(:,1); ps = y(:,2); dth = y(:,4); dps = y(:,5);
vx = p.l1*sin(th).*dth + p.l4*cos(ps).*dps;
vy = -p.l1*cos(th).*dth + p.l4*sin(ps).*dps;
V = sqrt(vx.^2 + vy.^2);

% Detection du premier pic
indices_pics = find(diff(sign(diff(V))) < 0) + 1;
if isempty(indices_pics)
    idx = length(V); % Si pas de pic, on prend la fin
else
    idx = indices_pics(1); % On prend strictement le PREMIER maximum local
end

fprintf('\n=== RÉSULTATS AU PREMIER MAXIMUM LOCAL ===\n');
fprintf('Masse du contrepoids (M) : %.2f kg\n', M_opt);
fprintf('Longueur de corde (l4)   : %.3f m\n', l4_opt);
fprintf('Vitesse au 1er pic       : %.2f m/s\n', V(idx));
fprintf('Angle au 1er pic         : %.2f°\n', rad2deg(atan2(vy(idx), vx(idx))));
fprintf('Temps du pic             : %.3f s\n', t(idx));
fprintf('==========================================\n');

% ========================================
% FONCTION DE COÛT : Cible le 1er Maximum
% ========================================
function erreur = calculer_erreur_premier_pic(x, p, y0, v_cible, angle_cible_rad)
    p.M = x(1);
    p.l4 = x(2);
    
    if p.M < 10 || p.l4 < 0.1, erreur = 1e12; return; end
    
    [~, y] = ode45(@(t,y) trebuchet_dynamics(t, y, p), [0 0.8], y0, odeset('RelTol',1e-5));
    
    vx = p.l1*sin(y(:,1)).*y(:,4) + p.l4*cos(y(:,2)).*y(:,5);
    vy = -p.l1*cos(y(:,1)).*y(:,4) + p.l4*sin(y(:,2)).*y(:,5);
    V = sqrt(vx.^2 + vy.^2);
    
    % --- Logique de détection du premier maximum local ---
    % On cherche l'endroit où la pente de la vitesse devient négative
    indices_pics = find(diff(sign(diff(V))) < 0) + 1;
    
    if isempty(indices_pics)
        % Si aucun pic n'est trouvé, le tir n'a pas encore "fouetté"
        v_actuelle = V(end);
        angle_actuel = atan2(vy(end), vx(end));
        penalite = 500; % On pénalise car on veut un vrai sommet
    else
        idx = indices_pics(1); % On s'arrête au PREMIER pic
        v_actuelle = V(idx);
        angle_actuel = atan2(vy(idx), vx(idx));
        penalite = 0;
    end
    
    % Calcul de l'erreur
    err_v = ((v_actuelle - v_cible)/v_cible)^2 * 100;
    err_a = (angle_actuel - angle_cible_rad)^2 * 10;
    
    erreur = err_v + err_a + penalite;
end

function dydt = trebuchet_dynamics(~, y, p)
    % Extraction des variables d'état
    th = y(1);   % Angle du bras
    ps = y(2);   % Angle de la fronde
    ph = y(3);   % Angle du contrepoids
    dth = y(4);  % Vitesse angulaire du bras
    dps = y(5);  % Vitesse angulaire de la fronde
    dph = y(6);  % Vitesse angulaire du contrepoids

    % --- MATRICE DE MASSE M (3x3) ---
    M11 = p.Ibeam + p.M*p.l2^2 + p.l1^2*p.m;
    M12 = -p.l1*p.l4*p.m*sin(ps - th);
    M13 = p.M*p.l2*p.l3*sin(ph - th);
    
    M21 = M12;  % Symétrique
    M22 = p.l4^2*p.m;
    M23 = 0;
    
    M31 = M13;  % Symétrique
    M32 = 0;
    M33 = p.M*p.l3^2;
    
    M = [M11, M12, M13;
         M21, M22, M23;
         M31, M32, M33];

    % --- VECTEUR FORCE F (3x1) ---
    % Composante pour theta
    F1 = -p.M*p.g*p.l2*cos(th) ...
         - p.M*p.l2*p.l3*cos(ph - th)*dph^2 ...
         + p.g*p.l1*p.m*cos(th) ...
         + p.g*p.l1*p.mbeam*cos(th)/2 ...
         - p.g*p.l2*p.mbeam*cos(th)/2 ...
         + p.l1*p.l4*p.m*cos(ps - th)*dps^2;
    
    % Composante pour psi
    F2 = -p.l4*p.m*(p.g*sin(ps) + p.l1*cos(ps - th)*dth^2);
    
    % Composante pour phi
    F3 = p.M*p.l3*(-p.g*sin(ph) + p.l2*cos(ph - th)*dth^2);
    
    F = [F1; F2; F3];

    % --- RÉSOLUTION: M * [ddth; ddps; ddph] = F ---
    accel = M \ F;
    
    % Vecteur dérivée d'état
    dydt = [y(4:6); accel];
end
