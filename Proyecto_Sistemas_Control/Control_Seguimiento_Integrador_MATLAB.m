%Programa  de un sistema con realimentacion de estados con Referencia
%con accion integrativa
%-------------------------------------------------------------------
clc; clear all; close all;

A=[0,1,0,0;0,-0.221772837618740,0.450086090591265,-0.661704776911353;0,0,0,1;0,-0.0116217981367384,0.0235863438656785,-0.506192658126714];
B=[0;1.02819574469726;0;2.93672730148517];
% A = [0 1 0 0;0 par(1) par(2) par(3);0 0 0 1;0 par(4) par(5) par(6)]; 
% B = [0;par(7);0;par(8)];
C=[0 0 1 0];
D=[0];


%Formo las nuevas matrices del sistema aumentado 
Aa=[A zeros(size(A,1),1); -C 0];
Ba=[B;-D]
%Ca=[C-D*K1 D*K1];
Da=D;

%verificacion de la controlabilidad del sistema
con=ctrb(Aa,Ba);
vector=rank(con);
fprintf('Num.de vectores LI de la M. de Controlabilidad:\n'); 
disp(vector);

%Ubicacion de los polos en el plano s
s1=-4;
s2=-20;
s3=-20;
s4=-20;
s5=-20;

raices2=[s1 s2 s3 s4 s5];
%break
%********** REALIMENTACION DE ESTADOS *****************
%Calculo de la matriz K 
K1=acker(Aa,Ba,raices2)   %usando Acker

%K2=place(Aa,Ba,raices2)   %usando Place

