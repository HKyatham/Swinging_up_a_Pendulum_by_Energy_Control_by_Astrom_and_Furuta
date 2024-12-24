% Parameters for Pendulum 1 and Pendulum 2
m1 = 0.1; % mass of Pendulum 1 (kg)
l1 = 9.81; % length of Pendulum 1 (m)
m2 = 0.335; % mass of Pendulum 2 (kg)
l2 = 9.81 / 4; % length of Pendulum 2 (m)
g = 9.81; % gravity (m/s^2)
n = 1.5; % ratio of maximum acceleration to g
k = 20; % control gain, fixed at 20

% Initial conditions for both pendulums
theta1_0 = -pi + 0.05; % initial angle of Pendulum 1 (rad)
% Adding a small value +0.05 for numerical stability
dtheta1_0 = 0.001; % initial angular velocity of Pendulum 1 (rad/s)
theta2_0 = -pi; % initial angle of Pendulum 2 (rad)
dtheta2_0 = 0.001; % initial angular velocity of Pendulum 2 (rad/s)

% Time span for simulation
simTime = 10;
tspan = [0 simTime];


% State-space model for two pendulums with control input 'u'.
function dxdt = pendulumsStateSpace(~, x, m1, l1, m2, l2, g, k, n)
    theta1 = x(1);
    dtheta1 = x(2);
    theta2 = x(3);
    dtheta2 = x(4);
    
    % Energy calculation of Pendulum 1 and Pendulum 2
    E1 = 0.5 * m1 * l1^2 * dtheta1^2 + m1 * g * l1 * (cos(theta1) - 1);
    E2 = 0.5 * m2 * l2^2 * dtheta2^2 + m2 * g * l2 * (cos(theta2) - 1);
    
    % G value from the Lyapunov derivative
    G = m1 * l1 * E1 * dtheta1 * cos(theta1) + m2 * l2 * E2 * dtheta2 * cos(theta2);
    
    % Control input and saturating the same
    u_unsat = k * G;
    u = min(max(u_unsat, -n * g), n * g); % Saturate control input to prevent instability

    % Calculating the angular velocities of both the pendulums
    dxdt = [
        dtheta1;
        (m1 * g * l1 * sin(theta1) - m1 * l1 * u * cos(theta1)) / (m1 * l1^2); % Pendulum 1 dynamics
        dtheta2;
        (m2 * g * l2 * sin(theta2) - m2 * l2 * u * cos(theta2)) / (m2 * l2^2); % Pendulum 2 dynamics
    ];
end

% Solving the ODE for the pendulum system using ODE89
options = odeset('RelTol',1e-8, 'AbsTol',1e-8); 
[t, x] = ode89(@(t, x) pendulumsStateSpace(t, x, m1, l1, m2, l2, g, k, n), tspan, [theta1_0, dtheta1_0, theta2_0, dtheta2_0], options);

u_vals = zeros(size(t));
last_theta1 = theta1_0; % Initialize with initial angle
last_dtheta1 = dtheta1_0; % Initialize with initial velocity
last_theta2 = theta2_0; % Initialize with initial angle
last_dtheta2 = dtheta2_0; % Initialize with initial velocity

% Initializing the pendulum data structure to store angles
pendulum_angles = struct('theta1', zeros(size(t)), 'theta2', zeros(size(t)));
freeze_flag = false; % Flag to track when to freeze angles 
pendulum_1_flag = false;  % Upright position flag for pendulum 1
pendulum_2_flag = false;  % Upright position flag for pendulum 2
not_upright_flag=true;    % Flag for not upright check
threshold = 0.15; % Threshold for freezing angles

for i = 1:length(t)
    % Extract states at the current time step
    theta1 = x(i,1);
    dtheta1 = x(i,2);
    theta2 = x(i,3);
    dtheta2 = x(i,4);
    
    % Storing the angles in the pendulum_angles data structure
    pendulum_angles.theta1(i) = theta1;
    pendulum_angles.theta2(i) = theta2;
    
    % Check if theta1 and theta2 are near 0 or a multiple of 2*pi
    if abs(mod(theta1, 2*pi)) < threshold || abs(abs(mod(theta1, 2*pi)) - 2*pi) < threshold
        pendulum_1_flag = true;
    end

    if abs(mod(theta2, 2*pi)) < threshold || abs(abs(mod(theta2, 2*pi)) - 2*pi) < threshold
        pendulum_2_flag = true;
    end

% As pointed out in the report, the control strategy discussed takes the 
% pendulum to the upright position. This position is, however, not stable 
% and requires a separate control strategy to catch and hold the pendulums 
% near that position. However, this is out of the scope of the paper discussed, 
% and we are freezing the angles at that position in the plot to showcase the same.



    % If both angles are near multiples of 2*pi, freeze them
    if pendulum_1_flag && pendulum_2_flag
        freeze_flag = true; % Set the flag to freeze angles
    end

    if freeze_flag && not_upright_flag 
        pendulum_1_final = pendulum_angles.theta1(i);
        pendulum_2_final = pendulum_angles.theta2(i);
        not_upright_flag = false;
    end
    
    % If freeze_flag is true, stop updating angles in pendulum_angles
    if freeze_flag 
        pendulum_angles.theta1(i) = pendulum_1_final; % Freeze theta1
        pendulum_angles.theta2(i) = pendulum_2_final; % Freeze theta2
    end
    
    % Compute energies, G value, and control input for plotting
    E1 = 0.5 * m1 * l1^2 * dtheta1^2 + m1 * g * l1 * (cos(theta1) - 1);
    E2 = 0.5 * m2 * l2^2 * dtheta2^2 + m2 * g * l2 * (cos(theta2) - 1);
    G = m1 * l1 * E1 * dtheta1 * cos(theta1) + m2 * l2 * E2 * dtheta2 * cos(theta2);
    u_unsat = k * G;
    u_vals(i) = min(max(u_unsat, -n * g), n * g); % Saturate control input to prevent instability
end

% Plot results
figure;

% Plotting Pendulum 1 and Pendulum 2 Angles using the pendulum_angles structure
subplot(3,1,1);
plot(t, pendulum_angles.theta1, 'r', 'LineWidth', 1.5); % Pendulum 1 angle
hold on;
plot(t, pendulum_angles.theta2, 'b', 'LineWidth', 1.5); % Pendulum 2 angle
title('Pendulum Angles');
xlabel('Time (s)');
ylabel('Angle (rad)');
legend('Pendulum 1', 'Pendulum 2');

% Plotting the  Normalized Energy for Pendulum 1 and Pendulum 2
subplot(3,1,2);
E1_norm = (0.5 * m1 * l1^2 * x(:,2).^2 + m1 * g * l1 .* (cos(x(:,1)) - 1)) / (m1 * g * l1);
E2_norm = (0.5 * m2 * l2^2 * x(:,4).^2 + m2 * g * l2 .* (cos(x(:,3)) - 1)) / (m2 * g * l2);
plot(t, E1_norm, 'r', 'LineWidth', 1.5); % Normalized Energy for Pendulum 1
hold on;
plot(t, E2_norm, 'b', 'LineWidth', 1.5); % Normalized Energy for Pendulum 2
title('Normalized Energy');
xlabel('Time (s)');
ylabel('Normalized Energy');
legend('Pendulum 1', 'Pendulum 2');

% Plotting the Control Signal in terms of g-force
subplot(3,1,3);
plot(t, u_vals / g, 'k', 'LineWidth', 1.5); % Control signal in terms of g-force
title('Control Signal in Terms of g-force');
xlabel('Time (s)');
ylabel('Control Signal (g)');

%% Simulating the results
figure;
% Assuming theta representing the pendulum angles (in radians)
theta1 = pendulum_angles.theta1;  % First pendulum's angles
theta2 = pendulum_angles.theta2;  % Second pendulum's angles
time = linspace(0, simTime, length(theta1));  % Create a time vector for sim time seconds

% Getting the length of theta
len_theta = length(theta1);

% Define positions of the pendulums as functions of time
x1 = @(t) sin(interp1(time, theta1, t));  % x-coordinates of first pendulum
y1 = @(t) cos(interp1(time, theta1, t)); % y-coordinates of first pendulum
x2 = @(t) sin(interp1(time, theta2, t));  % x-coordinates of second pendulum
y2 = @(t) cos(interp1(time, theta2, t));  % y-coordinates of second pendulum

% Animation setup
hold on;
axis equal;
xlim([-2, 2]);
ylim([-2, 2]);

% Animate pendulums
fanimator(@(t) plot([0, x1(t)], [0, y1(t)], 'k-', 'LineWidth', 2));  % First pendulum
fanimator(@(t) plot([0, x2(t)], [0, y2(t)], 'r-', 'LineWidth', 2));  % Second pendulum
fanimator(@(t) plot(x1(t), y1(t), 'ko', 'MarkerFaceColor', 'k'));  % First pendulum
fanimator(@(t) plot(x2(t), y2(t), 'ro', 'MarkerFaceColor', 'r'));  % Second pendulum

% timer display
fanimator(@(t) text(-1.5, 1.5, sprintf('Time: %.2f s', t), 'FontSize', 12));

% Display animation
hold off;
playAnimation;