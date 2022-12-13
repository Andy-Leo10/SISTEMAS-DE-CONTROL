clc; 
clear all; 
close all;
%De continua a discreta
A=[0 0 0 1;0 1446.9 213.6 -57.9;0 1 0 0;0 -126.3 -12.8 5.1];
%A=[0 1 0 0;0 -126.3 -12.8 5.1;0 0 0 1;0 1446.9 213.6 -57.9];
B=[0;-122.6;0;10.7];
%B=[0;10.7;0;-122.6];
C=[1 0 0 0];
D=[0];
T=0.01; %CAMBIAR SIEMPRE
%Se discretiza
[Ad Bd Cd Dd]=c2dm(A,B,C,D,T,'zoh')
%Se hallan los polos ideales a lazo cerrado
%E= 0.6901;
%Wn= 2.89813;
%s1=complex(-E*Wn,Wn*(1-E^2)^(1/2))
%s2=complex(-E*Wn,-Wn*(1-E^2)^(1/2))
%s3=complex(5*(-E*Wn),0)
s1=-4
s2=-20
s3=-20
s4=-20
%Se discretizan
z1=exp(T*s1)
z2=exp(T*s2)
z3=exp(T*s3)
z4=exp(T*s4)
%Análisis de controlabilidad
S=[Bd Ad*Bd Ad*Ad*Bd Ad*Ad*Ad*Bd]
%SISO:det(s)=/=0
detS=det(S)
ranS=rank(S)
%Se hallan los coeficientes deseados a lazo cerrado:
ai0=(z1*z2*z3*z4)
ai1=(-z1*z2*z4)-(z2*z3*z4)-(z1*z3*z4)-(z1*z2*z3)
ai2=(z1*z4)+(z2*z4)+(z3*z4)+(z1*z2)+(z2*z3)+(z1*z3)
ai3=(-z1-z2-z3-z4)
%Se hallan los coeficientes de la planta real:
polinomio=poly(Ad)
a0 = polinomio(5) 
a1 = polinomio(4) 
a2 = polinomio(3)
a3 = polinomio(2)
%Se obtiene Kn
kn=[ai0-a0 ai1-a1 ai2-a2 ai3-a3]
%Se halla P=S*M
M=[a1 a2 a3 1; a2 a3 1 0; a3 1 0 0;1 0 0 0]
%Entonces:
P=S*M
P2=inv(P)
K=kn*P2
%Por ackerman
Pd=[z1 z2 z3 z4];
Ka=acker(Ad,Bd,Pd)
k1=Ka(1)
k2=Ka(2)
k3=Ka(3)
k4=Ka(4)
%g=1/(C*inv(-A+B*K).*B)