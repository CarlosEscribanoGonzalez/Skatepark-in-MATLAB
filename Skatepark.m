close all
clear
clc

%EJERCICIO 1
% Definir el dominio
a = -2;
b = 4;
c = -2;
d = 2;

% Generar las coordenadas X, Y
I=[a:0.01:b];
J=[c:0.01:d];
[x, y] = meshgrid(I,J);

% Condición 1: inicialmente Z (los valores de la función) 
% es la matriz nula

Z = zeros(length(J),length(I));

%elipsoide:
cond2 = (x >= -2) & (x <= 2) &  (y>-1) & (y < 1);
a_elipsoide = 0.5;
b_elipsoide = 0.5;
Z(cond2)= -sqrt(max(0, 1 - (x(cond2).^2 / a_elipsoide^2) - (y(cond2).^2 / b_elipsoide^2)));


%Rampa:
cond3 = (x <= 4) & (x > 3.5) & (y >=-0.5) & (y<=1);
Z(cond3) = sin(y(cond3))/2 + 0.2 ;

%Esfera:
cond4 =  (y >=1) & (y<=2);
r_esfera = 0.5;
X_esfera1 = 3
Y_esfera1 = 1.5
Z(cond4) = sqrt(max(r_esfera^2 -(x(cond4)-X_esfera1 + 2).^2 - (y(cond4)-Y_esfera1).^2,0))

%Otra rampa
cond5 = (x>=1) & (x<=3) & (y<-1)
Z(cond5) = (max(0,-2 + 2.^(-1*y(cond5))))


% Se representa la superficie con el comando plot3 y surf:
figure
surf(x, y, Z, 'EdgeColor', 'none');
title('surf')
axis([a b c d -2 2]);
xlabel('x')
ylabel('y')
zlabel('z')

% Interpolación:
[xq,yq] = meshgrid(I,J)
zInt = griddata(x,y,Z,xq,yq,"cubic")
figure
plot3(x,y,Z, "mo")
hold on
surf(xq,yq,zInt,'FaceColor','black')
legend("Puntos","Superficie")

% Con imagen:
im = imread('cover1.jpg')
im = im2double(im)
im = flipud(im)
figure
surf(xq,yq,zInt,'FaceColor', 'texturemap','CData',im,'EdgeColor','none','AlphaData',zInt,'FaceAlpha','texture')

% EJERCICIO 2
%Skater1 
t = linspace(0,2*pi,100) 
curva1x = 2 + 1.8*cos(t)
curva1y = 0 + 1.8*sin(t)

%Skater 2
curva2x = 2 + 1.8*cos(t + pi/4) 
curva2y = 0 + 1.8*sin(t + pi/4)

%Skater 3
curva3x = 2 + 2*cos(t - pi/4)
curva3y = 0 + 2*sin(t - pi/4)

%Skater 4
curva4x = 2 + 2*cos(t + 3*pi/4)
curva4y = 0 + 2*sin(t + 3*pi/4)

%Skater 5
curva5x = 2 + 1.9*cos(t - 3*pi/4)
curva5y = 0 + 1.9*sin(t - 3*pi/4)


video = VideoWriter('video.avi','Motion JPEG AVI');
video.FrameRate = 30;
open(video)

figure;
axis tight manual
set(gca,"nextplot","replacechildren");

%Se calculan las trayectorias para el siguiente apartado utilizando el bucle
%Se hace un cálculo para cada coordenada porque así va a ser más sencillo
%para el calculo de la longitud segun la función definida

%Trayectoria Skater 1
trayectoriaSkater1x = []
trayectoriaSkater1y = []
trayectoriaSkater1z = []

%Trayectoria Skater 2
trayectoriaSkater2x = []
trayectoriaSkater2y = []
trayectoriaSkater2z = []

%Trayectoria Skater 3
trayectoriaSkater3x = []
trayectoriaSkater3y = []
trayectoriaSkater3z = []

%Trayectoria Skater 4
trayectoriaSkater4x = []
trayectoriaSkater4y = []
trayectoriaSkater4z = []

%Trayectoria Skater 5
trayectoriaSkater5x = []
trayectoriaSkater5y = []
trayectoriaSkater5z = []

for i = 1:numel(t)
    skater_x = curva1x(i)
    skater_y = curva1y(i)
    skater_z = interp2(x,y,Z,skater_x,skater_y)
    trayectoriaSkater1x = [trayectoriaSkater1x; skater_x]
    trayectoriaSkater1y = [trayectoriaSkater1y; skater_y]
    trayectoriaSkater1z = [trayectoriaSkater1z; skater_z]

    skater_x2 = curva2x(i)
    skater_y2 = curva2y(i)
    skater_z2 = interp2(x,y,Z,skater_x2,skater_y2)
    trayectoriaSkater2x = [trayectoriaSkater2x; skater_x2]
    trayectoriaSkater2y = [trayectoriaSkater2y; skater_y2]
    trayectoriaSkater2z = [trayectoriaSkater2z; skater_z2]

    skater_x3 = curva3x(i)
    skater_y3 = curva3y(i)
    skater_z3 = interp2(x,y,Z,skater_x3,skater_y3)
    trayectoriaSkater3x = [trayectoriaSkater3x; skater_x3]
    trayectoriaSkater3y = [trayectoriaSkater3y; skater_y3]
    trayectoriaSkater3z = [trayectoriaSkater3z; skater_z3]

    skater_x4 = curva4x(i)
    skater_y4 = curva4y(i)
    skater_z4 = interp2(x,y,Z,skater_x4,skater_y4)
    trayectoriaSkater4x = [trayectoriaSkater4x; skater_x4]
    trayectoriaSkater4y = [trayectoriaSkater4y; skater_y4]
    trayectoriaSkater4z = [trayectoriaSkater4z; skater_z4]

    skater_x5 = curva5x(i)
    skater_y5 = curva5y(i)
    skater_z5 = interp2(x,y,Z,skater_x5,skater_y5)
    trayectoriaSkater5x = [trayectoriaSkater5x;skater_x5]
    trayectoriaSkater5y = [trayectoriaSkater5y; skater_y5]
    trayectoriaSkater5z = [trayectoriaSkater5z; skater_z5]

    surf(x,y,Z,'EdgeColor','none')
    hold on
    plot3(skater_x,skater_y,skater_z,'ro','MarkerSize',10);
    plot3(skater_x2,skater_y2,skater_z2,'bo','MarkerSize',10);
    plot3(skater_x3,skater_y3,skater_z3,'go','MarkerSize',10);
    plot3(skater_x4,skater_y4,skater_z4,'mo','MarkerSize',10);
    plot3(skater_x5,skater_y5,skater_z5,'co','MarkerSize',10);
    axis('equal')
    hold off

    frame = getframe(gcf)
    writeVideo(video,frame)
    pause(0)
end

close(video)

% Ejercicio 3
%Se crea la función para calcular la longitud siguiendo el método de
%aproximacion poligonal. Dicha función se encuentra en otro documento
longitudS1 = calcularlongitud(trayectoriaSkater1x,trayectoriaSkater1y,trayectoriaSkater1z) %La longitud sale muy grande porque genera 100 puntos conforme hemos definido t
longitudS2 = calcularlongitud(trayectoriaSkater2x,trayectoriaSkater2y,trayectoriaSkater2z) %La longitud sale muy grande porque genera 100 puntos conforme hemos definido t
longitudS3 = calcularlongitud(trayectoriaSkater3x,trayectoriaSkater3y,trayectoriaSkater3z) %La longitud sale muy grande porque genera 100 puntos conforme hemos definido t
longitudS4 = calcularlongitud(trayectoriaSkater4x,trayectoriaSkater4y,trayectoriaSkater4z) %La longitud sale muy grande porque genera 100 puntos conforme hemos definido t
longitudS5 = calcularlongitud(trayectoriaSkater5x,trayectoriaSkater5y,trayectoriaSkater5z) %La longitud sale muy grande porque genera 100 puntos conforme hemos definido t

%El NT se saca a ojo del video viendo cuántas veces pasan por los trucos
NT1 = 5 % El primer skater empieza haciendo un truco en la rampa pequeña y acaba en el mismo sitio
NT2 = 4 % El segundo skater sólo pasa una vez por cada truco
NT3 = 4 % El tercer skater también pasa una vez únicamente por cada truco 
NT4 = 5 % El cuarto skater, al igual que el primero, empieza y acaba en un truco
NT5 = 4 % El quinto skater pasa una vez por cada truco únicamente

%Skater 1:
ps1x = polyfit(t,trayectoriaSkater1x,3)
x_interpol = polyval(ps1x,t)
ps1y = polyfit(t,trayectoriaSkater1y,3)
y_interpol = polyval(ps1y,t)
ps1z = polyfit(t,trayectoriaSkater1z,3)
z_interpol = polyval(ps1z,t)
% Se deriva la función: 
dx1 = diff(x_interpol)./diff(t)
dy1 = diff(y_interpol)./diff(t)
dz1 = diff(z_interpol)./diff(t)
v1 = sqrt(dx1.^2 + dy1.^2 + dz1.^2)
v1 = max(v1)

%Skater 2:
ps2x = polyfit(t,trayectoriaSkater2x,3)
x_interpol2 = polyval(ps2x,t)
ps2y = polyfit(t,trayectoriaSkater2y,3)
y_interpol2 = polyval(ps2y,t)
ps2z = polyfit(t,trayectoriaSkater2z,3)
z_interpol2 = polyval(ps2z,t)
dx2 = diff(x_interpol2)./diff(t)
dy2 = diff(y_interpol2)./diff(t)
dz2 = diff(z_interpol2)./diff(t)
v2 = sqrt(dx2.^2 + dy2.^2 + dz2.^2)
v2 = max(v2)

%Skater 3:
ps3x = polyfit(t,trayectoriaSkater3x,3)
x_interpol3 = polyval(ps3x,t)
ps3y = polyfit(t,trayectoriaSkater3y,3)
y_interpol3 = polyval(ps3y,t)
ps3z = polyfit(t,trayectoriaSkater3z,3)
z_interpol3 = polyval(ps3z,t)
dx3 = diff(x_interpol3)./diff(t)
dy3 = diff(y_interpol3)./diff(t)
dz3 = diff(z_interpol3)./diff(t)
v3 = sqrt(dx3.^2 + dy3.^2 + dz3.^2)
v3 = max(v3)

%Skater 4:
ps4x = polyfit(t,trayectoriaSkater4x,3)
x_interpol4 = polyval(ps4x,t)
ps4y = polyfit(t,trayectoriaSkater4y,3)
y_interpol4 = polyval(ps4y,t)
ps4z = polyfit(t,trayectoriaSkater4z,3)
z_interpol4 = polyval(ps4z,t)
dx4 = diff(x_interpol4)./diff(t)
dy4 = diff(y_interpol4)./diff(t)
dz4 = diff(z_interpol4)./diff(t)
v4 = sqrt(dx4.^2 + dy4.^2 + dz4.^2)
v4 = max(v4)

%Skater 5:
ps5x = polyfit(t,trayectoriaSkater5x,3)
x_interpol5 = polyval(ps5x,t)
ps5y = polyfit(t,trayectoriaSkater5y,3)
y_interpol5 = polyval(ps5y,t)
ps5z = polyfit(t,trayectoriaSkater5z,3)
z_interpol5 = polyval(ps5z,t)
dx5 = diff(x_interpol5)./diff(t)
dy5 = diff(y_interpol5)./diff(t)
dz5 = diff(z_interpol5)./diff(t)
v5 = sqrt(dx5.^2 + dy5.^2 + dz5.^2)
v5 = max(v5)

%Cálculo de puntuación
P1 = 0.25*NT1 + 0.5*longitudS1 + 0.25*v1 %108.34
P2 = 0.25*NT2 + 0.5*longitudS2 + 0.25*v2 %108.15
P3 = 0.25*NT3 + 0.5*longitudS3 + 0.25*v3 %113.77 -> Segundo puesto
P4 = 0.25*NT4 + 0.5*longitudS4 + 0.25*v4 %114.05 -> Primer puesto
P5 = 0.25*NT5 + 0.5*longitudS5 + 0.25*v5 %110.78 -> Tercer puesto
