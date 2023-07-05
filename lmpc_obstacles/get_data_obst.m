function data = get_data_obst()

interp = @griddedInterpolant;

% ========== Data ==========

n_save = 1; % Number for save file name

% ===== Simulation =====

anim_plot = false; % Animated plot
print_figures = false; % Export figures

% ===== Model =====

T = 0.1; % Discrete time interval [s]

% ===== Track =====

% Track 1
track.type = ['s','l','s','l','s'];
track.radius = [4, 4, 8, 4, 4];
track.curve = [0, pi, 0, pi, 0];

d_lim = 2;

Q_s = 15; Q_d = 1e05; Q_z = 3e05; Qd_v = 3e04; Qd_vz = 3e04;
d_r_coef = 0.5/d_lim;
v_lim = 8.5;

% Track length
L_track = sum(track.radius(track.type == 's')) + ...
	sum(track.curve(track.type ~= 's').*track.radius(track.type ~= 's'));

% ===== Curvature =====

% Relaxed
c_rel = 0.1;
[s_interp, K_interp] = get_curv(track,c_rel);
K_fun = interp(s_interp,K_interp,'pchip');

% Real
[s_interp, K_interp] = get_curv(track,1e-06);
K_fun_real = interp(s_interp,K_interp,'pchip');

% ===== Obstacles (horizontal) =====

% Inner border
obst_d_i.s = [0, 17, 19, 33, 35];
obst_d_i.d = [2, -0.5, 2, 1, 2];
% Outer border
obst_d_o.s = [0, 17, 19, 33, 35];
obst_d_o.d = [-2, -1.5, -2, -1, -2];

% Relaxed
c_rel = 0.999;

[s_interp, v_interp] = get_obst(obst_d_i,L_track,c_rel,'d','i');
obst_d_i_fun = interp(s_interp,v_interp,'pchip');

[s_interp, v_interp] = get_obst(obst_d_o,L_track,c_rel,'d','o');
obst_d_o_fun = interp(s_interp,v_interp,'pchip');

% Real
[s_interp, v_interp] = get_obst(obst_d_i,L_track,1e-06,'d','i');
obst_d_i_fun_real = interp(s_interp,v_interp,'pchip');

[s_interp, v_interp] = get_obst(obst_d_o,L_track,1e-06,'d','o');
obst_d_o_fun_real = interp(s_interp,v_interp,'pchip');

% ===== Obstacles (vertical) =====

% Lower border
obst_z_l.s = [0, 11, 13, 33, 35];
obst_z_l.z = [1, 2.5, 1, 3.5, 1];
% Upper border
obst_z_u.s = [0, 11, 13, 33, 35];
obst_z_u.z = [5, 3.5, 5, 4.5, 5];

% Relaxed
c_rel = 0.999;

[s_interp, v_interp] = get_obst(obst_z_l,L_track,c_rel,'z','l');
obst_z_l_fun = interp(s_interp,v_interp,'pchip');

[s_interp, v_interp] = get_obst(obst_z_u,L_track,c_rel,'z','u');
obst_z_u_fun = interp(s_interp,v_interp,'pchip');

% Real
[s_interp, v_interp] = get_obst(obst_z_l,L_track,1e-06,'z','l');
obst_z_l_fun_real = interp(s_interp,v_interp,'pchip');

[s_interp, v_interp] = get_obst(obst_z_u,L_track,1e-06,'z','u');
obst_z_u_fun_real = interp(s_interp,v_interp,'pchip');

% ===== Control =====

c_o = 0.5; n_o = 4; % First traj. oscillations
z_r = 3; % Reference altitude [m]

% LMPC
N_lmpc = 10; % LMPC prediction horizon
n_iter = 50; % Number of LMPC iterations

% ========== Output data structure ==========

data.T = T;

data.track = track;
data.d_lim = d_lim;
data.L_track = L_track;
data.Q_s = Q_s;
data.Q_d = Q_d;
data.Q_z = Q_z;
data.Qd_v = Qd_v;
data.Qd_vz = Qd_vz;
data.d_r_coef = d_r_coef;
data.v_lim = v_lim;

data.K_fun = K_fun;
data.K_fun_real = K_fun_real;

data.obst_d_i_fun = obst_d_i_fun;
data.obst_d_o_fun = obst_d_o_fun;
data.obst_z_l_fun = obst_z_l_fun;
data.obst_z_u_fun = obst_z_u_fun;

data.obst_d_i_fun_real = obst_d_i_fun_real;
data.obst_d_o_fun_real = obst_d_o_fun_real;
data.obst_z_l_fun_real = obst_z_l_fun_real;
data.obst_z_u_fun_real = obst_z_u_fun_real;

data.obst_d_i = obst_d_i;
data.obst_z_l = obst_z_l;

data.c_o = c_o;
data.n_o = n_o;
data.z_r = z_r;
data.N_lmpc = N_lmpc;
data.n_iter = n_iter;

data.anim_plot = anim_plot;
data.n_save = n_save;
data.print_figures = print_figures;

end









