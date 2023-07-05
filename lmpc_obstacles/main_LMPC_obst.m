clc
clear variables
close all
rng('default');

%% ========== Get data ==========

data = get_data_obst();

%% ========== First trajectory (SS init.) - MPC ==========

% x = [z  phi  theta  psi  vx  vy  vz  v_phi  v_theta  v_psi  s  d  t]
% u = [u1  u2  u3  u4]

x1 = [0, 0, 0, deg2rad(45), 0, 0, 0, 0, 0, 0, 0, 0, 0]';

[x_mpc, u_mpc, avg_exec_time_mpc] = MPC_obst(x1);

%% ========== LMPC ==========

x1 = x_mpc(:,end);
x1(11) = x1(11) - data.L_track;
x1(13) = x1(13) - 2*pi;

[x_safe, u_safe, Q_safe, avg_exec_time_lmpc] = LMPC_obst(x1, x_mpc, u_mpc);

fprintf('\nAverage exec. time (MPC) = %.3f ms\n',avg_exec_time_mpc*1e03)
fprintf('Average exec. time (LMPC) = %.3f ms',avg_exec_time_lmpc*1e03)

%% ========== Save data externally ==========

data_to_save = sprintf('lmpc_obst_data_%d.mat',data.n_save);
save(data_to_save,'x_safe','u_safe','Q_safe','data','avg_exec_time_mpc','avg_exec_time_lmpc');

%% ========== Plots ==========
close all
%pause
LMPC_obst_plot_results_fun(data.n_save,data.print_figures)


























