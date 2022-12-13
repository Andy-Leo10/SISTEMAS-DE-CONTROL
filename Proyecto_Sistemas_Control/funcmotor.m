function [A,B,C,D,K,x0] = funcmotor(par,Tm)
A = [0 1 0 0;0 par(1) par(2) par(3);0 0 0 1;0 par(4) par(5) par(6)]; 
B = [0;par(7);0;par(8)];

%forma 1: solo para el angulo de inclinacion del robot
% C = [0 0 1 0]; %SALIDA SOLO DE INCLINACION Y SOBRE ELLO EL CONTROL
% D = zeros(1,1);
% K = zeros(4,1);

%forma 1: solo para el angulo de inclinacion del robot
% C = [1 0 0 0]; %SALIDA SOLO DE POSICION LINEAL Y SOBRE ELLO EL CONTROL
% D = zeros(1,1);
% K = zeros(4,1);

% forma 2: utilizando 2 salidas 
C=[0 0 1 0;1 0 0 0];
D = zeros(2,1);
K = zeros(4,2);

x0 = [0;0;0;0];

