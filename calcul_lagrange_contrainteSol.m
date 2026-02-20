clear; clc;

%% ============================================================
%  PHASE 1 - Fronde contrainte au sol
%
%  Contrainte holonome: ym_abs = 0  (sol réel à y = 0)
%  Le pivot est à hauteur h_pivot au-dessus du sol.
%  En coordonnées relatives au pivot:
%    ym_rel = -l1*sin(θ) - l4*cos(ψ) = -h_pivot
%  => l1*sin(θ) + l4*cos(ψ) = h_pivot
%  => ψ(θ) = acos((h_pivot - l1*sin(θ)) / l4)
%  => dψ/dt = l1*cos(θ)*dθ / (l4*sin(ψ))
%
%  DOF réduit à 2: q = {θ, φ}
%  Lagrangien substitué avec ψ(θ) et dψ(θ,dθ)
%% ============================================================

% --- Variables Symboliques ---
syms m M mbeam l1 l2 l3 l4 h_pivot g Ibeam t real
syms th(t) ph(t)
syms v_th v_ph real
syms a_th a_ph real

% Dérivées libres
dth  = diff(th,t);
dph  = diff(ph,t);
ddth = diff(th,t,t);
ddph = diff(ph,t,t);

%% --- Contrainte: ψ et dψ exprimés en fonction de θ et dθ ---
% Contrainte géométrique: ym_rel = -l1*sin(θ) - l4*cos(ψ) = -h_pivot
%   => cos(ψ) = (h_pivot - l1*sin(θ)) / l4
%   => ψ(θ)   = acos((h_pivot - l1*sin(θ)) / l4)
%   => dψ/dt  = l1*cos(θ)*dθ / (l4*sin(ψ))    [signe + car d(cos(ψ))/dθ = -l1*cos(θ)/l4]
ps_expr  = acos((h_pivot - l1*sin(th)) / l4);

dps_expr = l1*cos(th)*dth / (l4*sin(ps_expr));

%% --- Positions (en coordonnées relatives au pivot) ---
% Extrémité droite du bras (côté contrepoids)
x_beam_right = l2*cos(th);
y_beam_right = l2*sin(th);

% Extrémité gauche du bras (côté fronde)
x_beam_left  = -l1*cos(th);
y_beam_left  = -l1*sin(th);

% Contrepoids M (pendule depuis l'extrémité droite)
xM = x_beam_right + l3*sin(ph);
yM = y_beam_right - l3*cos(ph);

% Projectile m — ψ substitué, ym_rel = -h_pivot par contrainte
xm = x_beam_left + l4*sin(ps_expr);
ym_rel = y_beam_left - l4*cos(ps_expr);   % = -h_pivot  (constante)

% Centre de masse du bras
xb = ((l2-l1)/2)*cos(th);
yb = ((l2-l1)/2)*sin(th);

%% --- Vitesses avec ψ substitué ---
vxm = diff(xm, t);     % = f(θ, dθ) uniquement
vym = sym(0);          % contrainte impose ym_rel = cste => dym_rel/dt = 0

vxM = diff(xM, t);
vyM = diff(yM, t);

%% --- Energies ---
% Vitesse au carré du projectile (vym = 0 par contrainte)
vM_sq = vxM^2 + vyM^2;
vm_sq = vxm^2;

% Energie cinétique
T = 0.5*M*vM_sq + 0.5*m*vm_sq + 0.5*Ibeam*dth^2;
T = simplify(T);

% Energie potentielle
% ym_abs_m = h_pivot + ym_rel = h_pivot - h_pivot = 0 (constante) => m*g*ym_abs_m = cste
% On peut inclure le terme constant ou l'omettre (ne change pas les EL).
% On l'omet: V = M*g*yM + mbeam*g*yb  (comme avant, relatif au pivot)
V = M*g*yM + mbeam*g*yb;
V = simplify(V);

% Lagrangien réduit (2 DOF: θ, φ)
L = simplify(T - V);

%% --- Equations d'Euler-Lagrange (2 DOF) ---
L_sub = subs(L, [dth, dph], [v_th, v_ph]);

vars_q = {th,   ph  };
vars_v = {v_th, v_ph};
real_v = {dth,  dph };

Eq = sym(zeros(2,1));
for k = 1:2
    dL_dv = diff(L_sub, vars_v{k});
    dL_dv = subs(dL_dv, vars_v, real_v);
    Eq(k) = simplify(diff(dL_dv, t) - diff(L, vars_q{k}));
end

%% --- Extraction Matrice de Masse M1 et Vecteur Force F1 ---
Eq_final = subs(Eq, [ddth, ddph], [a_th, a_ph]);
[M_mat1, F_vec1_raw] = equationsToMatrix(Eq_final, [a_th, a_ph]);
F_vec1 = simplify(-F_vec1_raw);

%% --- Force normale (multiplicateur de Lagrange) ---
% N est la force de réaction verticale du sol sur le projectile.
% Via Newton sur m dans la direction verticale (coordonnées absolues):
%   N - m*g = m * aym_abs
%   aym_abs = d²(ym_rel)/dt²   (car h_pivot = cste)
%           = d²/dt²[-l1*sin(θ) - l4*cos(ψ)]
%           = l1*sin(θ)*dθ² - l1*cos(θ)*ddθ
%           + l4*cos(ψ)*dψ² + l4*sin(ψ)*ddψ
%
% Avec la contrainte ym_rel = -h_pivot = cste, aym_abs = 0 en Phase 1
% (cohérent: le sol ne bouge pas). La force normale N = m*(g + aym_abs)
% sert à détecter le décollage (N → 0).

aym_expr = simplify(diff(-l1*sin(th) - l4*cos(ps_expr), t, 2));

% Force normale: N = m*(g + aym)
N_expr = simplify(m*(g + aym_expr));

%% --- Affichage ---
disp('====================================================');
disp('  PHASE 1 — Système contraint (fronde au sol)');
disp('  Contrainte: l1*sin(θ) + l4*cos(ψ) = h_pivot');
disp('  ψ(θ) = acos((h_pivot - l1*sin(θ)) / l4)');
disp('  dψ/dt = +l1*cos(θ)*dθ / (l4*sin(ψ))');
disp('  DOF: {θ, φ}');
disp('====================================================');
disp(' ');

disp('--- Contrainte et ψ contraint ---');
fprintf('ps(th)      = acos((h_pivot - l1*sin(th)) / l4)\n');
fprintf('dps(th,dth) = +l1*cos(th)*dth / (l4*sin(ps))\n\n');

disp('--- Matrice de Masse M1 (2x2) ---');
disp(M_mat1);

disp('--- Vecteur Force F1 (2x1) ---');
disp(F_vec1);

disp('--- Force Normale N ---');
disp(N_expr);

disp('====================================================');
disp('--- Éléments pour main.m (Phase 1) ---');
disp('====================================================');
fprintf('M1_11 = '); disp(M_mat1(1,1));
fprintf('M1_12 = '); disp(M_mat1(1,2));
fprintf('M1_22 = '); disp(M_mat1(2,2));
fprintf('\n');
fprintf('F1_1  = '); disp(F_vec1(1));
fprintf('F1_2  = '); disp(F_vec1(2));
fprintf('\nN     = '); disp(N_expr);
