%% Spring 2015 
close all
clear all
% x = -10:0.1:10;
% y = -10:0.1:10;

[x,y] = meshgrid(-5:0.1:5,-5:0.1:5);


z = ((x.^2) + (y.^2)).*exp(1).^(-((x.^2) + (y.^2)));

surf(z);

%or...

r = 0:0.01:10;

z1 = r.*exp(1).^-r;

figure(2)
plot(r,z1);


%%
% Clear memory, clean screen, close any figure
clear, clc, close all 

% Define our initial profile
x = linspace(0, pi/2, 20);
y = sin(x); 

% We draw the profile
subplot(221), plot(x,y), axis equal
title('Original function (profile)')
xlabel('x'); ylabel('y'); 

% We use the cylinder function to rotate and align
% with the z-axis, to produce a 3D solid 
[X,Y,Z] = cylinder(y);
subplot(222), surf(X,Y,Z), axis square
xlabel('z'); ylabel('y'); zlabel('x') 

% We can have another view, along the x-axis
subplot(223), surf(X,Y,Z), axis square
xlabel('z'); ylabel('y'); zlabel('x')
view(0,90)

% We produce another image, now a lateral view
subplot(224), surf(X,Y,Z), axis square
xlabel('z'); ylabel('y'); zlabel('x')
view(90,0)
 
%% Spring 2017 #2
w = -pi/2:0.01:pi/2;

r = cos(w-pi/8);

figure(1)
plot(w,r);

z = sin(2*w);

figure(2)
% plot(w,z);
plot(r,z);

%% Spring 2017 #3
clear
close all

xmin = -20;
xmax = 20;

ymin = -20;
ymax =20;

% min:1:max

[x,y] = meshgrid(xmin:1:xmax,ymin:1:ymax);

z = (x.^2)/2-(y.^2)/8;

surf(x,y,z);
zlim([-20 20])
xlabel('x');
ylabel('y');
zlabel('z');
hold on
x1 = -2;
y1 = 5;
z1 = -9/8;

hi = [x1 y1 z1];



A = -8;
B = -5;
C = -4;
D = 4.5;
% x = [1 -1 -1 1]; % Generate data for x vertices
% y = [1 1 -1 -1]; % Generate data for y vertices
% z = -1/C*(A*x + B*y + D); % Solve for z vertices data
% patch(x, y, z);



% [x y] = meshgrid(-1:0.1:1); % Generate x and y data
z = -1/C*(A*x + B*y + D); % Solve for z data
surf(x,y,z) %Plot the surface


scatter3(hi(1),hi(2),hi(3),300,'filled','k')

v = [-4, -5/2, -2];
% v = [-5/2,-4, -2];
% vnorm = -v;
vnorm = -30*v/norm(v);

hi1 = hi+vnorm;

line([hi(1) hi1(1)],[hi(2) hi1(2)],[hi(3) hi1(3)],'Color','black');



p1 = [-2 5 -9/8];
p2 = [0 0 9/8];
p3 = [0 9/10 0];

hi3 = cross(p2+p1,p3+p1)

line([hi(1) hi(1)+hi3(1)],[hi(2) hi(2)+hi3(2)],[hi(3) hi(3)+hi3(3)],'Color','black');


%%