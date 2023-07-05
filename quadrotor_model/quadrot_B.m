function B_sym = quadrot_B(z,phi,theta,psi,vx,vy,vz,v_phi,v_theta,v_psi,s,d,t,u1,u2,u3,u4,K,T)
%QUADROT_B
%    B_SYM = QUADROT_B(Z,PHI,THETA,PSI,VX,VY,VZ,V_PHI,V_THETA,V_PSI,S,D,T,U1,U2,U3,U4,K,T)

%    This function was generated by the Symbolic Math Toolbox version 8.7.
%    29-Jul-2022 16:22:02

t2 = cos(phi);
t3 = cos(psi);
t4 = sin(phi);
t5 = sin(psi);
t6 = sin(theta);
t7 = T.*1.587301587299444e+2;
B_sym = reshape([0.0,0.0,0.0,0.0,T.*(t4.*t5.*2.0+t2.*t3.*t6.*2.0),T.*(t3.*t4.*2.0-t2.*t5.*t6.*2.0).*-1.0,T.*t2.*cos(theta).*2.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t7,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t7,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,T.*8.333333333325572e+1,0.0,0.0,0.0],[13,4]);
