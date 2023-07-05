function [s_i, v_i] = get_obst(obst,L_track,c_rel,obst_type,side)

if obst_type == 'z'
	v = obst.z;
	
	if side == 'u'
	v_sign = -1;
	else
		if side == 'l'
			v_sign = 1;
		end
	end
else
	if obst_type == 'd'
		v = obst.d;
		
		if side == 'i'
			v_sign = -1;
		else
			if side == 'o'
				v_sign = 1;
			end
		end
	end
end

s = obst.s;

n = length(v);

s = [s, L_track];
v = [v, v(end)];
ds = diff(s);

s_i = s(1);
v_i = v(1);

for i=2:1:n
	
	if v_sign*(v(i) - v(i-1)) > 0
		s_i = [s_i, s(i) - c_rel*ds(i-1), s(i)];
		v_i = [v_i, v(i-1), v(i)];
	else
		s_i = [s_i, s(i), s(i) + 1e-06*ds(i)];
		v_i = [v_i, v(i-1), v(i)];
	end
	
end

s_i = [s_i, L_track];
v_i = [v_i, v(end)];

end













