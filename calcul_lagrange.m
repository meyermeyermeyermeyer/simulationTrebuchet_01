clear; clc;

% --- Variables Symboliques ---
syms m M mbeam l1 l2 l3 l4 g Ibeam t real
syms th(t) ps(t) ph(t) 
syms v_th v_ps v_ph real % Substitution pour vitesses
syms a_th a_ps a_ph real % Substitution pour accélérations

% Dérivées
dth = diff(th,t); dps = diff(ps,t); dph = diff(ph,t);
ddth = diff(th,t,t); ddps = diff(ps,t,t); ddph = diff(ph,t,t);

% --- Positions CORRIGÉES (selon le schéma) ---
% Convention: θ depuis l'horizontale (x-axis), positif antihoraire
%             ψ depuis la verticale vers le bas, positif vers la droite
%             φ depuis la verticale, positif vers la droite

% Extrémité DROITE du bras (où pend le contrepoids)
x_beam_right = l2*cos(th);
y_beam_right = l2*sin(th);

% Extrémité GAUCHE du bras (où s'attache la fronde)
x_beam_left = -l1*cos(th);
y_beam_left = -l1*sin(th);

% Position du CONTREPOIDS M (pend depuis l'extrémité droite)
% φ mesuré depuis la verticale, positif vers la droite
xM = x_beam_right + l3*sin(ph);
yM = y_beam_right - l3*cos(ph);

% Position du PROJECTILE m (fronde depuis l'extrémité gauche)
% ψ mesuré depuis la verticale vers le bas, positif vers la droite
xm = x_beam_left + l4*sin(ps);
ym = y_beam_left - l4*cos(ps);

% Centre de masse du bras (au milieu entre les deux extrémités)
% Position du centre de masse: à distance (l2-l1)/2 du pivot
xb = ((l2-l1)/2)*cos(th);
yb = ((l2-l1)/2)*sin(th);

% --- Énergies ---
% Vitesses au carré
vM_sq = diff(xM,t)^2 + diff(yM,t)^2;
vm_sq = diff(xm,t)^2 + diff(ym,t)^2;

% Énergie cinétique totale
T = 0.5*M*vM_sq + 0.5*m*vm_sq + 0.5*Ibeam*dth^2;

% Énergie potentielle totale (avec g positif vers le haut)
V = M*g*yM + m*g*ym + mbeam*g*yb;

% Lagrangien
L = simplify(T - V);

% --- Équations d'Euler-Lagrange ---
L_sub = subs(L, [dth, dps, dph], [v_th, v_ps, v_ph]);
vars_q = {th, ps, ph}; 
vars_v = {v_th, v_ps, v_ph}; 
real_v = {dth, dps, dph};
Eq = sym(zeros(3,1));

for k = 1:3
    dL_dv = diff(L_sub, vars_v{k});
    dL_dv = subs(dL_dv, vars_v, real_v);
    Eq(k) = simplify(diff(dL_dv, t) - diff(L, vars_q{k}));
end

% --- Extraction Matrice de Masse M et Vecteur Force F ---
Eq_final = subs(Eq, [ddth, ddps, ddph], [a_th, a_ps, a_ph]);
[M_mat, F_vec_raw] = equationsToMatrix(Eq_final, [a_th, a_ps, a_ph]);
F_vec = simplify(-F_vec_raw);

% --- Affichage pour copie manuelle ---
disp('========================================');
disp('--- Matrice de Masse M ---');
disp('========================================');
disp(M_mat);
disp(' ');
disp('========================================');
disp('--- Vecteur Force F ---');
disp('========================================');
disp(F_vec);
disp(' ');

% --- Affichage des expressions individuelles pour copie dans main.m ---
disp('========================================');
disp('--- Éléments de M pour copier dans main.m ---');
disp('========================================');
fprintf('M11 = '); disp(M_mat(1,1));
fprintf('M12 = '); disp(M_mat(1,2));
fprintf('M13 = '); disp(M_mat(1,3));
fprintf('M22 = '); disp(M_mat(2,2));
fprintf('M23 = '); disp(M_mat(2,3));
fprintf('M33 = '); disp(M_mat(3,3));
disp(' ');

disp('========================================');
disp('--- Éléments de F pour copier dans main.m ---');
disp('========================================');
fprintf('F1 = '); disp(F_vec(1));
fprintf('F2 = '); disp(F_vec(2));
fprintf('F3 = '); disp(F_vec(3));
