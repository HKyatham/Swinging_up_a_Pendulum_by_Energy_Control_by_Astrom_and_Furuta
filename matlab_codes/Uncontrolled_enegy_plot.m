% Parameters
n = 2.1; % normalized maximum acceleration
k = 100; % control gain
m = 0.1; % mass (kg)
l = 9.81; % length (m) adjusted for omega0 = 1
g = 9.81; % gravity (m/s^2)
J = m * l^2; % moment of inertia

% A grid with range of values for angle and angular velocity is defined
% here.
theta_vals = linspace(-2*pi, 2*pi, 100); % Angle from -2pi to 2pi
dtheta_vals = linspace(-4, 4, 100); % Angular velocity range from -4 to 4 rad/s

% A mesh grid is created for contour plot
[Theta, Dtheta] = meshgrid(theta_vals, dtheta_vals);

% Energy is calculated here as same as given in the paper.
Energy = 0.5 * J * Dtheta.^2 + m * g * l * (cos(Theta)-1); 


% Contour plot creation
figure;
contour(Theta, Dtheta, Energy, 50); % 50 contour levels are used
title('Energy of uncontrolled pendulum');
xlabel('Angle (rad)');
ylabel('Angular Velocity (rad/s)');
colorbar; % color bar for enegy values
