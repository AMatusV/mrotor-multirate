% Archivo para graficar las variables de estado del cuadrotor
clc;     % limpiamos pantalla y memoria
load Datos.dat % cargamos el archivo de datos
%
% Graficamos
plot(Datos(:,1), Datos(:,2), Datos(:,1),Datos(:,3),...
     Datos(:,1), Datos(:,4))
     

