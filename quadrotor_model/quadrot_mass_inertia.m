clc, clear variables, close all

Ma = 0.04/4;
Mb = 0.34;
mr = 0.12/4;

b = 0.06;
l = 0.07;

format short

m = Mb + 4*Ma + 4*mr

L = b+2*l

format short e

Ix = (1/6)*Mb*b^2 + (2/3)*Ma*l^2 + (1/2)*Ma*b^2 + 2*mr*(b/2+l)
Iy = (1/6)*Mb*b^2 + (2/3)*Ma*l^2 + (1/2)*Ma*b^2 + 2*mr*(b/2+l)
Iz = (1/6)*Mb*b^2 + (4/3)*Ma*l^2 + Ma*b^2 + 4*mr*(b/2+l)
