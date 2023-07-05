function [s, K] = get_curv(track,c_rel)

n = length(track.type);

s = 0;
s_track = 0;
ds = [];
K = [];

for i = 1:1:n
	
	type = track.type(i);
	r = track.radius(i);
	c = track.curve(i);
	
	switch type
		
		case 's'
			l = r;
			
		case 'l'
			l = c*r;
			
		case 'r'
			l = c*r;
			
	end
	
	s_track = [s_track, s_track(end) + l];
	ds = [ds, l];
	
end

ds = [ds, ds(end)]*c_rel;

for i = 1:1:n
	
	type = track.type(i);
	r = track.radius(i);
	c = track.curve(i);
	
	switch type
		
		case 's'
			l = r;
			if isempty(K)
				K = [0, 0];
			else
				K = [K, 0, 0];
			end
			
		case 'l'
			l = c*r;
			if isempty(K)
				K = [1/r, 1/r];
			else
				K = [K, 1/r, 1/r];
			end
			
		case 'r'
			l = c*r;
			if isempty(K)
				K = [-1/r, -1/r];
			else
				K = [K, -1/r, -1/r];
			end
			
	end
	
	s = [s, s_track(i) + l - ds(i), s_track(i) + l];
	
end

K = [K, K(end)];

end













