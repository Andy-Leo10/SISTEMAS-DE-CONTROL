% para lo del paper

clear all;
clc;
A=[0 1 0 0;0 -126.3 -12.8 5.1;0 0 0 1;0 1446.9 213.6 -57.9];
B=[0;10.7;0;-122.6];
C=[1 0 0 0;0 1 0 0;0 0 1 0; 0 0 0 1];
D=[0;0;0;0];
K=[-44.67, -29.26, -16.56, -1.57];
k1=-44.67;
k2=-29.26;
k3=-16.56;
k4=-1.57;
g1=-44.67;
%%
clear all;
clc;
A=[0 1 0 0;0 -126.3 -12.8 5.1;0 0 0 1;0 1446.9 213.6 -57.9];
B=[0;10.7;0;-122.6];
C=[1 0 0 0;0 1 0 0;0 0 1 0; 0 0 0 1];
D=[0;0;0;0];

con=ctrb(A,B);
vector=rank(con);
fprintf('Num.de vectores LI de la M. de Controlabilidad:\n'); 
disp(vector);

s1=-4;
s2=-20;
s3=-20;
s4=-20;
raices=[s1 s2 s3 s4];

%Calculo de la matriz K 
K=acker(A,B,raices)   %usando Acker
k1=K(1);
k2=K(2);
k3=K(3);
k4=K(4);
g1=K(1);

%% para la planta obtenida con la caja gris

% clear all;
% clc;

A=[0,1,0,0;0,-18.1491205715699,-0.241940259070541,-0.00161788352516104;0,0,0,1;0,-1042.17088344464,-16.2106294996100,-2.12241475559971]
B=[0;3.24704785577389;0;142.313463961677]
% A=m_est.A;
% B=m_est.B;
C=[1 0 0 0;0 1 0 0;0 0 1 0; 0 0 0 1];
D=[0;0;0;0];

con=ctrb(A,B);
vector=rank(con);
fprintf('Num.de vectores LI de la M. de Controlabilidad:\n'); 
disp(vector);

% Tess=2;
% % para 20% de sobreimpulso
% epsi=0.3744;
% wn=4/(epsi*Tess);
% s1=-epsi*wn+wn*sqrt(1-epsi^2)*j;
% s2=-epsi*wn-wn*sqrt(1-epsi^2)*j;
% s3=-5*epsi*wn+wn*sqrt(1-epsi^2)*j;
% s4=-5*epsi*wn-wn*sqrt(1-epsi^2)*j;
% raices=[s1 s2 s3 s4];
raices=[-4 -20 -20 -20];

%Calculo de la matriz K 
K=acker(A,B,raices)   %usando Acker

k1=K(1);
k2=K(2);
k3=K(3);
k4=K(4);
g1=K(1);

ts=1/1000;
%%