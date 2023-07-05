clc
clear variables
close all

syms z phi theta psi % States (positions)
syms vx vy vz v_phi v_theta v_psi % States (velocities)
syms s d t % States (Frenet)
syms u1 u2 u3 u4 % Inputs
syms K % Curvature
syms T % Discrete time step

m = 0.5;
Ix = 6.3e-03;
Iy = 6.3e-03;
Iz = 1.2e-02;
g = 9.81;
beta = 0.25;

f_sym(z, phi, theta, psi, ...
	vx, vy, vz, v_phi, v_theta, v_psi, ...
	s, d, t, ...
	u1, u2, u3, u4, K, T) = ...
	[z + T*vz;
	phi + T*v_phi;
	theta + T*v_theta;
	psi + T*v_psi;
	vx + T * ( (1/m)*(cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi))*u1 - (beta/m)*vx);
	vy + T * ( (1/m)*(cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi))*u1 - (beta/m)*vy);
	vz + T * ( (1/m)*(cos(phi)*cos(theta))*u1 - g - (beta/m)*vz);
	v_phi + T * ( (Iy-Iz)/Ix * v_theta*v_psi + u2/Ix );
	v_theta + T * ( (Iz-Ix)/Iy * v_phi*v_psi + u3/Iy );
	v_psi + T * ( (Ix-Iy)/Iz * v_theta*v_phi + u4/Iz );
	s + T * ( 1/(1-K*d) * (vx*cos(t) + vy*sin(t)) );
	d + T * ( -vx*sin(t) + vy*cos(t) );
	t + T * ( K/(1-K*d) * (vx*cos(t) + vy*sin(t)) )];

% Linearization (ATV model)

A_sym(z, phi, theta, psi, ...
	vx, vy, vz, v_phi, v_theta, v_psi, ...
	s, d, t, ...
	u1, u2, u3, u4, K, T) = jacobian(f_sym,[z, phi, theta, psi, ...
	vx, vy, vz, v_phi, v_theta, v_psi, s, d, t]);

B_sym(z, phi, theta, psi, ...
	vx, vy, vz, v_phi, v_theta, v_psi, ...
	s, d, t, ...
	u1, u2, u3, u4, K, T) = jacobian(f_sym,[u1, u2, u3, u4]);

f_sym = vpa(f_sym,4);
A_sym = vpa(A_sym,4);
B_sym = vpa(B_sym,4);

matlabFunction(f_sym,'File','quadrot_nl');
matlabFunction(A_sym,'File','quadrot_A');
matlabFunction(B_sym,'File','quadrot_B');









