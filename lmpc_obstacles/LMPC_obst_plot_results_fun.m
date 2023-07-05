function LMPC_obst_plot_results_fun(n_save, print_figures)

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

% ========== (1) Trajectories on track ==========

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

% Trajectories

x = x_safe{1};
[x_traj, y_traj] = plot_on_track(data.track,x(11,:),x(12,:));
traj_p(1) = plot(x_traj,y_traj,'g-','linewidth',1);

for i = 2:1:N_traj
	x = x_safe{i};
	[x_traj, y_traj] = plot_on_track(data.track,x(11,:),x(12,:));
	traj_p(2) = plot(x_traj,y_traj,'b-','linewidth',1);
end

x = x_safe{min_cost_traj};
[x_traj, y_traj] = plot_on_track(data.track,x(11,:),x(12,:));
traj_p(3) = plot(x_traj,y_traj,'r-','linewidth',2); hold off, grid on, axis equal

if isempty(v_obst)
	if isempty(h_obst)
		legend(traj_p,'MPC','LMPC','Best',...
			'NumColumns',3,'location','southoutside','interpreter','latex')
	else
		legend([traj_p, h_obst],'MPC','LMPC','Best',...
			'Track/Horiz. obst.',...
			'NumColumns',3,'location','southoutside','interpreter','latex')
	end
else
	if isempty(h_obst)
		legend([traj_p, v_obst],'MPC','LMPC','Best',...
			'Vert. obst.',...
			'NumColumns',3,'location','southoutside','interpreter','latex')
	else
		legend([traj_p, h_obst, v_obst],'MPC','LMPC','Best',...
			'Track/Horiz. obst.','Vert. obst.',...
			'NumColumns',3,'location','southoutside','interpreter','latex')
	end
end

xlabel('$x$ [m]','interpreter','latex')
ylabel('$y$ [m]','interpreter','latex')

xlim([x_axis_l-0.1, x_axis_u+0.1]);
ylim([y_axis_l-0.1, y_axis_u+0.1]);

title('\textbf{Track}','interpreter','latex')

set(gca,'fontsize',16)

% ========== (2) Altitude ==========

f2 = figure(2); hold on
f2.Position = [200   100   560   420];

plot([0,data.L_track],[1, 1],'-','color',[0.7 0.7 0.7],'linewidth',1)
plot([0,data.L_track],[5, 5],'-','color',[0.7 0.7 0.7],'linewidth',1)

% Vertical ostacles

v_plot = data.obst_z_u_fun_real(s_plot);
v_obst = plot(s_plot,v_plot,'k-', 'linewidth', 1);
v_plot = data.obst_z_l_fun_real(s_plot);
plot(s_plot,v_plot,'k-', 'linewidth', 1)

if length(data.obst_z_l.s) == 1
	v_obst = [];
end

% Trajectories

x = x_safe{1};
traj_p(1) = plot(x(11,:), x(1,:), 'g-', 'linewidth', 1);

for i = 2:1:N_traj
	x = x_safe{i};
	traj_p(2) = plot(x(11,:), x(1,:), 'b-', 'linewidth', 1);
end
	
x = x_safe{min_cost_traj};
traj_p(3) = plot(x(11,:), x(1,:), 'r-', 'linewidth', 2); hold off, grid on

if isempty(v_obst)
	legend(traj_p,'MPC','LMPC','Best',...
		'NumColumns',3,'location','southoutside','interpreter','latex')
else
	legend([traj_p, v_obst],'MPC','LMPC','Best',...
		'Vert. obst.',...
		'NumColumns',3,'location','southoutside','interpreter','latex')
end

xlim([0, data.L_track])

xlabel('$s$ [m]','interpreter','latex')

title('\textbf{Altitude} $z$ [m]','interpreter','latex')

set(gca,'XMinorGrid','on')
set(gca,'fontsize',16)

% ========== (3) Velocity profiles ==========

f3 = figure(3); hold on
f3.Position = [300   100   560   420];

h1 = plot(x_safe{1}(11,:), sqrt(x_safe{1}(5,:).^2 + x_safe{1}(6,:).^2), 'g-');

for i=2:1:N_traj
	h2 = plot(x_safe{i}(11,:), sqrt(x_safe{i}(5,:).^2 + x_safe{i}(6,:).^2), 'b-');
end

h3 = plot(x_safe{min_cost_traj}(11,:), ...
		sqrt(x_safe{min_cost_traj}(5,:).^2 + x_safe{min_cost_traj}(6,:).^2), ...
		'r-', 'linewidth', 2);

hold off, grid on

xlim([0, data.L_track])

xlabel('$s$ [m]','interpreter','latex')

title('\textbf{Planar velocity} $ v = \sqrt{v_x^2 + v_y^2} \; $ [$\mathrm{m \, s^{-1}}$]',...
	'interpreter','latex')

legend([h1 h2 h3],{'MPC','LMPC','Best'},...
	'NumColumns',3,'location','southoutside','interpreter','latex')

set(gca,'XMinorGrid','on')
set(gca,'YMinorGrid','on')
set(gca,'fontsize',16)

% ========== (4) Iteration cost ==========

f4 = figure(4); hold on
f4.Position = [400   100   560   420];
tl = tiledlayout(2,1,'TileSpacing','Compact');

nexttile

plot(0:1:N_traj-1, Q_traj, 'b.-','markersize',10)
% for i = 1:1:N_traj
% 	
% 	data_val = sprintf('%1.6f',Q_traj(i)/10^4);
% 	
% 	if i == 1
% 		horiz_align = 'left';
% 		vert_align = 'top';
% 	else
% 		if i == N_traj
% 			horiz_align = 'right';
% 			vert_align = 'top';
% 		else
% 			horiz_align = 'center';
% 			vert_align = 'bottom';
% 		end
% 	end
% 	
% 	text(i-1,Q_traj(i),data_val,...
% 		'VerticalAlignment',vert_align,'HorizontalAlignment',horiz_align,...
% 		'FontSize',9);
% end

hold off

xticks(0:5:N_traj-1)
xticklabels([])

%xlabel('Iteration','interpreter','latex')

title('\textbf{Iteration cost}','interpreter','latex')

grid on
set(gca,'XMinorGrid','on')
set(gca,'YMinorGrid','on')
set(gca,'fontsize',16)

% ========== (5) Lap time ==========

% f5 = figure(5); hold on
% f5.Position = [500   100   560   420];

nexttile

for i=1:1:N_traj
	lap_time(i) = length(x_safe{i,1})*data.T;
end

plot(0:1:N_traj-1, lap_time, 'b.-','markersize',10)
% for i = 1:1:N_traj
% 	
% 	data_val = sprintf('%1.2f',lap_time(i));
% 	
% 	if i == 1
% 		horiz_align = 'left';
% 		vert_align = 'top';
% 	else
% 		if i == N_traj
% 			horiz_align = 'right';
% 			vert_align = 'bottom';
% 		else
% 			horiz_align = 'center';
% 			vert_align = 'bottom';
% 		end
% 	end
% 	
% 	text(i-1,lap_time(i),data_val,...
% 		'VerticalAlignment',vert_align,'HorizontalAlignment',horiz_align,...
% 		'FontSize',9);
% end

hold off

xticks(0:5:N_traj-1)

%xlabel('Iteration','interpreter','latex')

title('\textbf{Lap time} [s]','interpreter','latex')

grid on

set(gca,'XMinorGrid','on')
set(gca,'YMinorGrid','on')
set(gca,'fontsize',16)

xlabel(tl,'Iteration','interpreter','latex','fontsize',17)

% ========== Print iteration cost and lap time on terminal ==========

clc

fprintf('Average exec. time (MPC) = %.3f ms\n',avg_exec_time_mpc*1e03)
fprintf('Average exec. time (LMPC) = %.3f ms\n',avg_exec_time_lmpc*1e03)

fprintf('Iteration cost:\n');
for j=1:1:N_traj
	fprintf('%d\t%1.10e\n', j-1, Q_safe{j,1}(1));
end

fprintf('Lap time:\n');
for j=1:1:N_traj
	fprintf('%d\t%1.2f s\n', j-1, length(x_safe{j,1})*data.T);
end

% ========== Export images ==========

if print_figures == true
	name_fig_1 = sprintf('lmpc_obst_track_%d.jpg',n_save');
	name_fig_2 = sprintf('lmpc_obst_altitude_%d.jpg',n_save');
	name_fig_3 = sprintf('lmpc_obst_velocity_%d.jpg',n_save');
	name_fig_4 = sprintf('lmpc_obst_iter_cost_steps_%d.jpg',n_save');
	%name_fig_5 = sprintf('lmpc_obst_steps_%d.jpg',n_save');

	exportgraphics(f1,name_fig_1,'Resolution',600);
	exportgraphics(f2,name_fig_2,'Resolution',600);
	exportgraphics(f3,name_fig_3,'Resolution',600);
	exportgraphics(f4,name_fig_4,'Resolution',600);
	%exportgraphics(f5,name_fig_5,'Resolution',600);
end

end


























