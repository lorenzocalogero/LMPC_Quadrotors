function [x_d, y_d] = plot_on_track(track,s,d)

n = length(track.type);

L_track = sum(track.radius(track.type == 's')) + ...
	sum(track.curve(track.type ~= 's').*track.radius(track.type ~= 's'));

alpha = 0; alpha_v = alpha;
x0 = 0; y0 = 0;
s_track = 0;

% Compute s_track and initial points (x0, y0) of each track segment
for i = 1:1:n
	
	type = track.type(i);
	r = track.radius(i);
	c = track.curve(i);
	
	switch type
		
		case 's'
			l = r;
			x0 = [x0, x0(end) + l*cos(alpha)];
			y0 = [y0, y0(end) + l*sin(alpha)];
			alpha = alpha + 0;
			alpha_v = [alpha_v, alpha];
			
		case 'l'
			t = c; l = c*r;
			x0 = [x0, x0(end) + r*cos(alpha+pi/2) + r*cos(t - pi/2 + alpha)];
			y0 = [y0, y0(end) + r*sin(alpha+pi/2) + r*sin(t - pi/2 + alpha)];
			alpha = alpha + c;
			alpha_v = [alpha_v, alpha];
			
		case 'r'
			t = c; l = c*r;
			x0 = [x0, x0(end) + r*cos(alpha-pi/2) + r*cos(-t + pi/2 + alpha)];
			y0 = [y0, y0(end) + r*sin(alpha-pi/2) + r*sin(-t + pi/2 + alpha)];
			alpha = alpha - c;
			alpha_v = [alpha_v, alpha];
			
	end
	
	s_track = [s_track, s_track(end) + l];
	
end

% Plot (s,d) on track
N = length(s);

x = 0; y = 0;
x_d = []; y_d = [];

for i = 1:1:N
	
	if s(i) < 0
		s(i) = L_track + s(i);
	end
	
	s_logic = max(min(sum(((s_track - s(i)) <= 0)),n),1);
	si = max(s(i) - s_track(s_logic));
	
	type = track.type(s_logic);
	r = track.radius(s_logic);
	alpha = alpha_v(s_logic);
	
	switch type
		
		case 's'
			l = si;
			x1 = x0(s_logic) + l*cos(alpha);
			y1 = y0(s_logic) + l*sin(alpha);
			x_d1 = x1 + d(i)*cos(alpha+pi/2);
			y_d1 = y1 + d(i)*sin(alpha+pi/2);
			
		case 'l'
			t = si/r;
			x1 = x0(s_logic) + r*cos(alpha+pi/2) + r*cos(t - pi/2 + alpha);
			y1 = y0(s_logic) + r*sin(alpha+pi/2) + r*sin(t - pi/2 + alpha);
			x_d1 = x1 + d(i)*cos(alpha+t+pi/2);
			y_d1 = y1 + d(i)*sin(alpha+t+pi/2);
			
		case 'r'
			t = si/r;
			x1 = x0(s_logic) + r*cos(alpha-pi/2) + r*cos(-t + pi/2 + alpha);
			y1 = y0(s_logic) + r*sin(alpha-pi/2) + r*sin(-t + pi/2 + alpha);
			x_d1 = x1 + d(i)*cos(alpha-t+pi/2);
			y_d1 = y1 + d(i)*sin(alpha-t+pi/2);
			
	end
	
	x = [x, x1];
	y = [y, y1];
	x_d = [x_d, x_d1];
	y_d = [y_d, y_d1];
	
end

end












