function Q = compute_cost(x,u,data)

% ===== Cost function =====

% x = [z  phi  theta  psi  vx  vy  vz  v_phi  v_theta  v_psi  s  d  t]
% u = [u1  u2  u3  u4]

x_r = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, data.L_track*1.2, 0, 0]';

Q = diag([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, data.Q_s, 0, 0]);

cost = @(x,u) (x-x_r)'*Q*(x-x_r);

% ===== Compute cost-to-go =====

Q = zeros(1,size(x,2));
Q(size(x,2)) = 0;

for i = size(x,2)-1:-1:1
	Q(i) = Q(i+1) + cost(x(:,i), u(:,i));
end

end

