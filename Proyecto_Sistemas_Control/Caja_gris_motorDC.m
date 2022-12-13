%Ejemplo de identificación con el modelo de caja Gris
%Explicación en:
%https://www.mathworks.com/help/ident/ug/estimating-linear-grey-box-models.html

% close all
% clear all

par = [0; 0; 0; 0; 0; 0; 0; 0]
aux = {};
T = 0;
m = idgrey('funcmotor',par,'c',aux,T)

identificacion_ang;
data = iddata(Datos(:,2:3),Datos(:,1),Tm);

m_est = greyest(data,m)
