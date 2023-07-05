function [x_safe, u_safe, Q_safe, avg_exec_time] = LMPC_obst(x1, x_first, u_first)

yalmip('clear'); yalmip('clearsolution'); yalmip('clearsos');

% ========== Get data ==========

data = get_data_obst();

% ========== Model ==========

nx = 13;
nu = 4;

% ========== LMPC data ==========

N = data.N_lmpc; % Prediction horizon

% ========== Yalmip declarations ==========

% ===== Optimization variables =====
% Matrices
A_op = sdpvar(nx,nx,'full');
B_op = sdpvar(nx,nu,'full');
c_op = sdpvar(nx,1,'full');

% States
x = sdpvar(nx*ones(1,N+1),ones(1,N+1));
x1_in = sdpvar(nx,1);

% Inputs
u = sdpvar(nu*ones(1,N),ones(1,N));

% Slack variables
e1 = sdpvar(1,1); % Horizontal bounds/obstacles
e2 = sdpvar(1,1); % Vertical bounds/obstacles
e3 = sdpvar(1,1); % Terminal constraint

% Auxiliary variables
q = sdpvar(1,1); % Terminal cost
d_r_op = sdpvar(1,1); % Lateral distance reference
z_r_op = sdpvar(1,1); % Altitude reference
d_o_op = sdpvar(1,1); % Horizontal outer bound
d_i_op = sdpvar(1,1); % Horizontal inner bound
z_l_op = sdpvar(1,1); % Vertical lower bound
z_u_op = sdpvar(1,1); % Vertical upper bound

% ===== Cost function =====

cost = 0;

% x = [z  phi  theta  psi  vx  vy  vz  v_phi  v_theta  v_psi  s  d  t]
% u = [u1  u2  u3  u4]

x_r = [z_r_op, 0, 0, pi/4, 0, 0, 0, 0, 0, 0, data.L_track*1.2, d_r_op, 0]';

Q = diag([data.Q_z, 0, 0, 1, 0, 0, 0, 0, 0, 0, data.Q_s, data.Q_d, 0]);

R = diag([0.01, 0.01, 0.01, 0.01]);

Qd = (0.1^2/0.2^2)*(1/data.T^2)*...
	diag([10, 0.01, 0.01, 10, data.Qd_v, data.Qd_v, data.Qd_vz, 0.01, 0.01, 1e03, 10, 0.01, 0]);
Rd = (0.1^2/0.2^2)*(1/data.T^2)*diag([0.01, 0.01, 0.01, 0.01]);

for k = 1:1:N
	cost = cost + (x{k}-x_r)'*Q*(x{k}-x_r) + u{k}'*R*u{k};
end

for k = 2:1:N
	cost = cost + (x{k}-x{k-1})'*Qd*(x{k}-x{k-1}) + (u{k}-u{k-1})'*Rd*(u{k}-u{k-1});
end

cost = cost + 1e10*e1^2 + 1e10*e2^2 + 1e03*e3^2;

cost = cost + q; % Terminal cost

% ===== Constraints =====
constr = [];

for k = 1:1:N
	constr = [constr;
		x{k+1} == A_op*x{k} + B_op*u{k} + c_op];
	
	constr = [constr;
		d_o_op - e1 <= x{k}(12) <= d_i_op + e1];
	
	constr = [constr;
		z_l_op - e2 <= x{k}(1) <= z_u_op + e2];
	
	constr = [constr;
		0 <= abs(x{k}(5)+x{k}(6)) <= data.v_lim];
end

constr = [constr;
	x{1} == x1_in];

constr = [constr;
	e1 >= 0;
	e2 >= 0];

% ===== Optimizer object (1) =====
sol_out = u{1};
options = sdpsettings('verbose',0,'solver','quadprog');

% ========== Opt. prob. resol. and control ==========

% Store first trajectory

x_safe{1} = x_first;
u_safe{1} = u_first;
Q_safe{1} = compute_cost(x_first,u_first,data);

% Initialize SS

SS = x_safe{1}([11,12],:);
Q_SS = Q_safe{1};

% SS and Q_SS relaxation

[A_ch, b_ch] = points_to_ch_lcon(SS);

% Additional opt. variables
A_ch_op = sdpvar(size(A_ch,1),size(A_ch,2),'full');
b_ch_op = sdpvar(size(b_ch,1),size(b_ch,2),'full');

k_cf = 10;
[a_cf, b_cf, ~] = convex_piecewise_fit(SS,Q_SS,k_cf,k_cf*5);
while length(b_cf) < 3
	k_cf = k_cf + 1;
	[a_cf, b_cf, ~] = convex_piecewise_fit(SS,Q_SS,k_cf,k_cf*5);
end

% Auxiliary constraints for terminal cost
for i = 1:1:length(b_cf)
	constr = [constr;
		a_cf(:,i)'*x{N+1}([11,12]) + b_cf(i) <= q];
end

constr_1 = constr;

% Terminal constraint
constr = [constr;
	A_ch_op*x{N+1}([11,12]) <= b_ch_op + e3];

% ===== Optimizer object (2) =====
params_in = {x1_in, A_op, B_op, c_op, A_ch_op, b_ch_op, ...
	d_r_op, z_r_op, d_o_op, d_i_op, z_l_op, z_u_op};
lmpc = optimizer(constr,cost,options,params_in,sol_out);

% x = [z  phi  theta  psi  vx  vy  vz  v_phi  v_theta  v_psi  s  d  t]
% u = [u1  u2  u3  u4]

exec_time = [];

for j = 1:1:data.n_iter
	
	x_lmpc = x1;
	u_lmpc = [];

	u1 = [0; 0; 0; 0];
	
	% ===== BEGIN Animated plot (1) =====
	figure(1)
	
	% Track
	subplot(1,2,1)
	
	[q_loc_x, q_loc_y] = plot_on_track(data.track,x1(11),x1(12));
	q_loc = plot(q_loc_x,q_loc_y,'r.','markersize',15);
	
	% Altitude
	subplot(1,2,2)
	
	q_alt = plot(x1(11),x1(1),'r.','markersize',15);
	
	step_disp = 1;
	% ===== END Animated plot (1) =====
	
	while x1(11) < data.L_track
		
		K1 = data.K_fun(x1(11)); % Relaxed curvature
	
		% Update ATV model
		A = quadrot_A(x1(1),x1(2),x1(3),x1(4),x1(5),x1(6),...
			x1(7),x1(8),x1(9),x1(10),x1(11),x1(12),x1(13),...
			u1(1),u1(2),u1(3),u1(4),K1,data.T);
	
		B = quadrot_B(x1(1),x1(2),x1(3),x1(4),x1(5),x1(6),...
			x1(7),x1(8),x1(9),x1(10),x1(11),x1(12),x1(13),...
			u1(1),u1(2),u1(3),u1(4),K1,data.T);

		c = quadrot_nl(x1(1),x1(2),x1(3),x1(4),x1(5),x1(6),...
			x1(7),x1(8),x1(9),x1(10),x1(11),x1(12),x1(13),...
			u1(1),u1(2),u1(3),u1(4),K1,data.T) - (A*x1 + B*u1);
		
		% Obstacles/bounds
		z_l = data.obst_z_l_fun(x1(11));
		z_u = data.obst_z_u_fun(x1(11));
		d_i = data.obst_d_i_fun(x1(11));
		d_o = data.obst_d_o_fun(x1(11));

		% Lateral distance reference
		K_sign = sign(data.K_fun_real(x1(11)));

		if K_sign == 0 || K_sign == 1
			d_r = d_i * data.d_r_coef;
		else
			d_r = d_o * data.d_r_coef;
		end

		% Altitude reference
		z_r = (z_l+z_u)/2;
		
		% ===== BEGIN LMPC =====
		% Get optimal control input from LMPC algorithm
		tic
		u1 = lmpc({x1, A, B, c, A_ch, b_ch, d_r, z_r, d_o, d_i, z_l, z_u});
		curr_exec_time = toc;
		
		exec_time = [exec_time, curr_exec_time];

		% Apply optimal control input and get next states
		x2 = quadrot_nl(x1(1),x1(2),x1(3),x1(4),x1(5),x1(6),...
			x1(7),x1(8),x1(9),x1(10),x1(11),x1(12),x1(13),...
			u1(1),u1(2),u1(3),u1(4),data.K_fun_real(x1(11)),data.T);

		% Store optimal states and inputs
		x_lmpc = [x_lmpc, x2];
		u_lmpc = [u_lmpc, u1];
		% ===== END LMPC =====
		
		% ===== BEGIN Animated plot (2) =====
		subplot(1,2,1)
	
		delete(q_loc);

		[x_traj_1, y_traj_1] = plot_on_track(data.track,x1(11),x1(12));
		[x_traj_2, y_traj_2] = plot_on_track(data.track,x2(11),x2(12));
		plot([x_traj_1, x_traj_2],[y_traj_1, y_traj_2],'b-','linewidth',1)

		[q_loc_x, q_loc_y] = plot_on_track(data.track,x2(11),x2(12));
		q_loc = plot(q_loc_x,q_loc_y,'r.','markersize',15);

		if data.anim_plot == true
			drawnow
		end

		subplot(1,2,2)

		delete(q_alt);

		plot([x1(11), x2(11)], [x1(1), x2(1)], 'b-', 'linewidth', 1)

		q_alt = plot(x2(11),x2(1),'r.','markersize',15);

		if data.anim_plot == true
			drawnow
		end
		
		clc
		fprintf('Iteration %d\n', j);
		fprintf('Time = %1.2f s\n',step_disp*data.T);
		
		fprintf('Previous iteration (%d):\nLap time = %1.2f s\nCost = %1.10e\n', ...
			j-1, length(x_safe{j,1})*data.T, Q_safe{j,1}(1))
		
		step_disp = step_disp + 1;
		% ===== END Animated plot (2) =====

		% Update initial states
		x1 = x2;
		
	end
	
	x_safe{end+1,1} = x_lmpc;
	u_safe{end+1,1} = u_lmpc;
	Q_safe{end+1,1} = compute_cost(x_lmpc,u_lmpc,data);
	
	% Update SS
	SS = [SS, x_safe{end,1}([11,12],:)];
	Q_SS = [Q_SS, Q_safe{end,1}];
	
	% SS relaxation
	[A_ch, b_ch] = points_to_ch_lcon(SS);
	
	% Update opt. variables size
	A_ch_op = sdpvar(size(A_ch,1),size(A_ch,2),'full');
	b_ch_op = sdpvar(size(b_ch,1),size(b_ch,2),'full');
	
	constr = constr_1;
	
	% Update terminal constraint
	constr = [constr;
		A_ch_op*x{N+1}([11,12]) <= b_ch_op + e3];
	
	% Update optimizer object
	params_in = {x1_in, A_op, B_op, c_op, A_ch_op, b_ch_op, ...
		d_r_op, z_r_op, d_o_op, d_i_op, z_l_op, z_u_op};
	lmpc = optimizer(constr,cost,options,params_in,sol_out);
	
	% Update initial state for next iteration
	
	x1 = x_lmpc(:,end);
	x1(11) = x1(11) - data.L_track;
	x1(13) = x1(13) - 2*pi;
	
	delete(q_loc);
	delete(q_alt);
	
end

clc

fprintf('Iteration cost:\n');
for j=1:1:data.n_iter+1
	fprintf('%d\t%1.10e\n', j-1, Q_safe{j,1}(1));
end

fprintf('Lap time:\n');
for j=1:1:data.n_iter+1
	fprintf('%d\t%1.2f s\n', j-1, length(x_safe{j,1})*data.T);
end

avg_exec_time = mean(exec_time);

end


















