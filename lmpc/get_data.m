function data = get_data()

interp = @griddedInterpolant;

% ========== Data ==========

n_save = 1; % Number for save file name

% ===== Simulation =====

anim_plot = false; % Animated plot
print_figures = false; % Export figures

% ===== Model =====

T = 0.1; % Discrete time interval [s]

% ===== Track (select only one) =====

% Track 1
track.type = ['s','l','s','l','s'];
track.radius = [4, 4, 8, 4, 4];
track.curve = [0, pi, 0, pi, 0];

d_lim = 2;

Q_s = 20; Q_d = 1e05; Q_z = 3e05; Qd_v = 3e04; Qd_vz = 1e05;
d_r_coef = 0.5/d_lim;
v_lim = 9;

% % Track 2
% track.type = ['s','l','r','l','s','l','s'];
% track.radius = [4, 4, 4, 4, 8, 4, 4];
% track.curve = [0, pi, pi/2, pi, 0, pi/2, 0];
% 
% d_lim = 2;
% 
% Q_s = 25; Q_d = 1e05; Q_z = 3e05; Qd_v = 2e04; Qd_vz = 1e05;
% d_r_coef = 0.9/d_lim;
% v_lim = 12;

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

data.c_o = c_o;
data.n_o = n_o;
data.z_r = z_r;
data.N_lmpc = N_lmpc;
data.n_iter = n_iter;

data.anim_plot = anim_plot;
data.n_save = n_save;
data.print_figures = print_figures;

end









