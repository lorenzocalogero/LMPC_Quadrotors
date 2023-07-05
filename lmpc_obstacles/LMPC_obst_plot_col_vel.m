clc
clear variables
close all
load('my_colormap.mat');

% ========== Settings ==========
n_save = 1;
print_figures = true;

% ========== Load data ==========

data_to_load = sprintf('lmpc_obst_data_%d.mat',n_save);
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

% Vertical obstacles

for i = 2:1:length(data.obst_z_l.s)
	[x_obst, y_obst] = plot_on_track(data.track,...
		[data.obst_z_l.s(i), data.obst_z_l.s(i)],[-data.d_lim, data.d_lim]);
	v_obst = plot(x_obst,y_obst,'m-','linewidth',1);
end

if length(data.obst_z_l.s) == 1
	v_obst = [];
end

% Track
plot(x_c(1),y_c(1),'k.','markersize',15)
plot(x_c,y_c,'k-','linewidth',0.5)
plot(x_in,y_in,'-','color',[0.7 0.7 0.7],'linewidth',1)
plot(x_out,y_out,'-','color',[0.7 0.7 0.7],'linewidth',1)
x_axis_u = max(x_out);
x_axis_l = min(x_out);
y_axis_u = max(y_out);
y_axis_l = min(y_out);

% Horizontal obstacles
s_plot = linspace(0,data.L_track,5000);

d_plot = data.obst_d_i_fun_real(s_plot);
[x_obst, y_obst] = plot_on_track(data.track,s_plot,d_plot);
h_obst = plot(x_obst,y_obst,'k-','linewidth',1);
d_plot = data.obst_d_o_fun_real(s_plot);
[x_obst, y_obst] = plot_on_track(data.track,s_plot,d_plot);
plot(x_obst,y_obst,'k-','linewidth',1)

if length(data.obst_d_i.s) == 1
	h_obst = [];
end

title('\textbf{Track}','interpreter','latex')

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

if isempty(v_obst)
	if isempty(h_obst)
	else
		legend([h_obst, h1],...
			{'Track/Horiz. obst.','Best'},...
			'NumColumns',3,'location','southoutside','interpreter','latex')
	end
else
	if isempty(h_obst)
		legend([v_obst, h1],...
			{'Vert. obst.','Best'},...
			'NumColumns',3,'location','southoutside','interpreter','latex')
	else
		legend([h_obst, v_obst, h1],...
			{'Track/Horiz. obst.','Vert. obst.','Best'},...
			'NumColumns',3,'location','southoutside','interpreter','latex')
	end
end

set(gca,'fontsize',16)

%% Export images

if print_figures == true
	name_fig_1 = sprintf('lmpc_obst_col_vel_%d.jpg',n_save');

	exportgraphics(f1,name_fig_1,'Resolution',600);
end



























