clear; close all; clc;

%% Nearby geodesics on a unit sphere

R = 1;

% Small longitude separation
dphi = 0.15;

% Colatitude from equator to north pole
theta = linspace(pi/2, 0, 500);

% Two meridians
phi1 = zeros(size(theta));
phi2 = dphi * ones(size(theta));

% Convert spherical to Cartesian
x1 = R * sin(theta) .* cos(phi1);
y1 = R * sin(theta) .* sin(phi1);
z1 = R * cos(theta);

x2 = R * sin(theta) .* cos(phi2);
y2 = R * sin(theta) .* sin(phi2);
z2 = R * cos(theta);

% Distance between the two geodesics
dist = sqrt((x2-x1).^2 + (y2-y1).^2 + (z2-z1).^2);

%% Plot sphere and geodesics

figure; hold on; axis equal; grid on;

% Sphere surface
[X,Y,Z] = sphere(60);
surf(X,Y,Z, ...
    'FaceAlpha', 0.15, ...
    'EdgeAlpha', 0.15);

% Geodesics
plot3(x1,y1,z1,'LineWidth',3);
plot3(x2,y2,z2,'LineWidth',3);

% Start and end points
scatter3(x1(1),y1(1),z1(1),100,'filled');
scatter3(x2(1),y2(1),z2(1),100,'filled');
scatter3(0,0,1,150,'filled');

xlabel('x'); ylabel('y'); zlabel('z');
title('Nearby geodesics on a sphere converge');

%% Plot distance between geodesics

figure; hold on; grid on;

plot(theta, dist, 'LineWidth', 3);
set(gca,'XDir','reverse');

xlabel('\theta');
ylabel('Distance between geodesics');
title('Geodesic separation decreases toward the pole');