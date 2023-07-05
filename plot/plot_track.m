function [x_d, y_d] = plot_track(track,d,N)

n = length(track.type);

alpha = 0;
x = 0; y = 0;
x_d = []; y_d = [];

for i = 1:1:n
	
	type = track.type(i);
	r = track.radius(i);
	c = track.curve(i);
	
	switch type
		
		case 's'
			l = linspace(0,r,N);
			x1 = x(end) + l*cos(alpha);
			y1 = y(end) + l*sin(alpha);
			x_d1 = x1 + d*cos(alpha+pi/2);
			y_d1 = y1 + d*sin(alpha+pi/2);
			alpha = alpha + 0;
			
		case 'l'
			t = linspace(0,c,N);
			x1 = x(end) + r*cos(alpha+pi/2) + r*cos(t - pi/2 + alpha);
			y1 = y(end) + r*sin(alpha+pi/2) + r*sin(t - pi/2 + alpha);
			x_d1 = x1 + d*cos(alpha+t+pi/2);
			y_d1 = y1 + d*sin(alpha+t+pi/2);
			alpha = alpha + c;
			
		case 'r'
			t = linspace(0,c,N);
			x1 = x(end) + r*cos(alpha-pi/2) + r*cos(-t + pi/2 + alpha);
			y1 = y(end) + r*sin(alpha-pi/2) + r*sin(-t + pi/2 + alpha);
			x_d1 = x1 + d*cos(alpha-t+pi/2);
			y_d1 = y1 + d*sin(alpha-t+pi/2);
			alpha = alpha - c;
			
	end
	
	x = [x, x1];
	y = [y, y1];
	x_d = [x_d, x_d1];
	y_d = [y_d, y_d1];
	
end

end











