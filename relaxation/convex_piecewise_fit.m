function [a,b,N] = convex_piecewise_fit(x,y,k,N_max)

n = size(x,1); % Data points size
m = size(x,2); % Number of data points

% ===== First data partition =====

P = cell(k,1);

for i=1:1:k
	P{i} = [];
end

% i_rnd = randperm(m,k);
% 
% x0 = x(:,i_rnd);

mean_x = zeros(n,1);
cov_x = zeros(n);

for i = 1:1:m
	mean_x = mean_x + (1/m)*x(:,i);
	cov_x = cov_x + (1/m)*x(:,i)*x(:,i)';
end

x0 = mvnrnd(mean_x,cov_x,k)';

dist = zeros(k,1);
part_sel = 1:1:k;

for i = 1:1:m
	
	for j = 1:1:k
		dist(j) = norm(x0(:,j)-x(:,i));
	end
	
	min_dist = min(dist);
	i_min_dist = (dist == min_dist);
	i_part = part_sel(i_min_dist);
	i_part = i_part(1);
	
	P{i_part} = [P{i_part}, i];
	
end

% Remove empty partitions
P = P(~cellfun(@isempty, P));
k = size(P,1);

N = 1;
exit_flag = 0;
P_change = 1;

while exit_flag == 0
	
	% ===== Least squares fit on partitions =====
	
	a = zeros(n,k); b = zeros(1,k);

	for i = 1:1:k % For each partition
		
		A11 = 0;
		A12 = 0;

		B1 = 0;
		B2 = 0;

		for j = 1:1:length(P{i})
			A11 = A11 + x(:,P{i}(j))*x(:,P{i}(j))';
		end

		for j = 1:1:length(P{i})
			A12 = A12 + x(:,P{i}(j));
		end

		A21 = A12';

		A22 = length(P{i});

		A = [A11 A12; A21 A22];

		for j = 1:1:length(P{i})
			B1 = B1 + y(P{i}(j))*x(:,P{i}(j));
		end

		for j = 1:1:length(P{i})
			B2 = B2 + y(P{i}(j));
		end

		B = [B1; B2];

		s = pinv(A)*B;

		a(:,i) = s(1:n);
		b(:,i) = s(end);

	end

	% ===== Update partitions =====

	P_prev = P;
	P = cell(k,1);

	for i=1:1:m % For each data point

		val_max = a(:,1)'*x(:,i) + b(1);
		index_max = 1;

		for j = 2:1:k

			val = a(:,j)'*x(:,i) + b(j);
			if val >= val_max
				index_max = j;
				val_max = val;
			end

		end

		P{index_max} = [P{index_max}, i];

	end

	% Remove empty partitions
	P = P(~cellfun(@isempty, P));
	k = size(P,1);
	
	% Check if partitions have changed
	if size(P,1) == size(P_prev,1)
		if cellfun(@isequal, P, P_prev)
			P_change = 0;
		end
	end
	
	if (N == N_max) || (P_change == 0)
		exit_flag = 1;
	else
		N = N+1;
	end
	
end

end



















