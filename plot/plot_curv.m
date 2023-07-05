clc
clear variables
close all
interp = @griddedInterpolant;

%% ========== Settings ==========
print_figures = true;

%% ========== Track (select only one) ==========

% Track 1
% track.type = ['s','l','s','l','s'];
% track.radius = [4, 4, 8, 4, 4];
% track.curve = [0, pi, 0, pi, 0];
% d_lim = 2;

% Track 2
track.type = ['s','l','r','l','s','l','s'];
track.radius = [4, 4, 4, 4, 8, 4, 4];
track.curve = [0, pi, pi/2, pi, 0, pi/2, 0];
d_lim = 2;

% Track length
L_track = sum(track.radius(track.type == 's')) + ...
	sum(track.curve(track.type ~= 's').*track.radius(track.type ~= 's'));

%% ========== Curvature plot ==========

c_rel = 0.3;
[s_interp, K_interp] = get_curv(track,c_rel);
K_fun = interp(s_interp,K_interp,'pchip');

[s_interp, K_interp] = get_curv(track,1e-06);
K_fun_real = interp(s_interp,K_interp,'pchip');

f1 = figure(1);

s_plot = linspace(0,L_track,5000);

K_plot_real = K_fun_real(s_plot);
plot(s_plot,K_plot_real,'r--', 'linewidth', 1), hold on

K_plot = K_fun(s_plot);
plot(s_plot,K_plot,'r-', 'linewidth', 1), hold off, grid on

title('\textbf{Track curvature} $K, \; \tilde{K}$ [$\mathrm{m^{-1}}$]', 'interpreter', 'latex')
xlabel('$s$ [m]', 'interpreter', 'latex')
legend('Real curvature','Relaxed curvature',...
	'NumColumns',2,'location','southoutside','interpreter','latex')
ylim([min(K_plot_real)-0.02, max(K_plot_real)+0.02])
xlim([0, L_track])

set(gca,'fontsize',16)

%% ========== Export images ==========
if print_figures == true
	exportgraphics(f1,'curv_track_2_rel.jpg','Resolution',600);
end















