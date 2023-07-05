clc
clear variables
close all
load('my_colormap.mat');

% ========== Settings ==========
n_save = 1;
print_figures = true;

% ========== Load data ==========

data_to_load = sprintf('lmpc_data_%d.mat',n_save);
load(data_to_load,'x_safe','Q_safe','data','avg_exec_time_mpc','avg_exec_time_lmpc');

N_traj = length(x_safe);

for i=1:1:N_traj
	Q_traj(i) = Q_safe{i}(1);
end

N_traj_vec = 1:1:N_traj;
min_cost_traj = N_traj_vec(Q_traj == min(Q_traj));
min_cost_traj = min_cost_traj(end);

%% ========== Trajectories on track with colored velocity profile ==========

f1 = figure(1); hold on
f1.Position = [100   100   560   650];

[x_c,y_c] = plot_track(data.track,0,30);
[x_in,y_in] = plot_track(data.track,data.d_lim,30);
[x_out,y_out] = plot_track(data.track,-data.d_lim,30);

% Track

plot(x_c(1),y_c(1),'k.','markersize',15)
plot(x_c,y_c,'k-','linewidth',0.5)
plot(x_in,y_in,'k-','linewidth',1)
plot(x_out,y_out,'k-','linewidth',1)
x_axis_u = max(x_out);
x_axis_l = min(x_out);
y_axis_u = max(y_out);
y_axis_l = min(y_out);

colormap(my_colormap)

% Trajectories

for i = 1:1:N_traj
	x = x_safe{i};
	v = sqrt(x(5,:).^2+x(6,:).^2);
	v_max(i) = max(v);
	
	[x_traj, y_traj] = plot_on_track(data.track,x(11,:),x(12,:));
	color_line(x_traj,y_traj,v, 'linewidth', 1.5);
end

x = x_safe{min_cost_traj};
[x_traj, y_traj] = plot_on_track(data.track,x(11,:),x(12,:));
h1 = plot(x_traj,y_traj,'k:','linewidth',1);

hold off, grid on, axis equal

color_bar = colorbar;
color_bar.Label.String = '\textbf{Planar velocity} $ v = \sqrt{v_x^2 + v_y^2} \; $ [$\mathrm{m \, s^{-1}}$]';
color_bar.Label.Interpreter = 'latex';

caxis([0, max(v_max)])

xlim([x_axis_l-0.1, x_axis_u+0.1]);
ylim([y_axis_l-0.1, y_axis_u+0.1]);

xlabel('$x$ [m]','interpreter','latex')
ylabel('$y$ [m]','interpreter','latex')

title('\textbf{Track}','interpreter','latex')

legend(h1,{'Best'},...
	'NumColumns',1,'location','southoutside','interpreter','latex')

set(gca,'fontsize',16)

%% Export images

if print_figures == true
	name_fig_1 = sprintf('lmpc_col_vel_%d.jpg',n_save');

	exportgraphics(f1,name_fig_1,'Resolution',600);
end



























