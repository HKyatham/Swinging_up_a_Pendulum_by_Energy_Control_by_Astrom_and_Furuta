% Parameters
n = 2.1; % ratio of maximum acceleration to g
k = 100; % control gain
m = 0.01; % mass (kg)
l = 9.81; % length (m) adjusted for omega0 = 1
g = 9.81; % gravity (m/s^2)
J = m * l^2; % moment of inertia

% Initial conditions
theta0 = -pi; % initial angle (rad), downward position
dtheta0 = 0.0001; % initial angular velocity (rad/s)

% Desired energy level at the upright position
E0 = 0;

% Time span
simTime = 6;
tspan = [0 simTime];

% State-space model function
function dxdt = pendulumStateSpace(~, x, J, m, g, l, k, n, E0)
    theta = x(1);
    dtheta = x(2);
    % Defnining Energy as per the equation 2
    E = 0.5 * J * dtheta^2 + m * g * l * (cos(theta) - 1);
    % Control equation as per the equation 8
    u_unsat = k * (E - E0) * sign(dtheta * cos(theta));
    % Applying the sat fuction in equation 8
    u = min(max(u_unsat, -n*g), n*g); % Saturate control signal
    
    dxdt = [dtheta;
            (m * g * l * sin(theta) - m * l * u * cos(theta)) / J];
end

% Solving ODE using ode45 with state-space model
[t, x] = ode45(@(t, x) pendulumStateSpace(t, x, J, m, g, l, k, n, E0), tspan, [theta0, dtheta0]);

% Calculating initial energy
E_initial = 0.5 * J * dtheta0^2 + m * g * l * (cos(theta0) - 1);
fprintf('Initial Energy E: %.2f J\n', E_initial);

% Calculating control signal over time and print initial value
u_control_g_force = zeros(length(t), 1); % Control signal in terms of g-force
for i = 1:length(t)
    % Initializing theta variable
    theta = x(i, 1);
    % Initializing dtheta
    dtheta = x(i, 2);
    % Definining Energy as per the equation 2
    E = 0.5 * J * dtheta^2 + m * g * l * (cos(theta) - 1);
    E = E / m*g*l ;
    % Control equation as per the equation 8
    u_unsat = k * (E - E0) * sign(dtheta * cos(theta));
    % Applying the sat fuction in equation 8
    u_control_g_force(i) = min(max(u_unsat / g, -n), n); % Normalize by g to express in terms of g-force
    % Printing the first value of control input
    if i == 1
        fprintf('Initial Control Signal: %.2f g\n', u_control_g_force(i));
    end
    
    % Stop applying force once the desired energy is reached
    if E >= E0 && i > 1
        u_control_g_force(i:end) = 0;
        break;
    end
end

% Plot results
figure;
subplot(3,1,1);
plot(t, x(:,1));
title('Pendulum Angle');
xlabel('Time (s)');
ylabel('\theta (rad)');

subplot(3,1,2);
E_norm = (0.5 * J * x(:,2).^2 + m * g * l .* (cos(x(:,1)) - 1)) / (m*g*l);
plot(t, E_norm);
title('Normalized Energy');
xlabel('Time (s)');
ylabel('Normalized Energy');


subplot(3,1,3);
plot(t(1:length(u_control_g_force)), u_control_g_force);
title('Control Signal in Terms of g-force');
xlabel('Time (s)');
ylabel('Control Signal (g)');



%% Simulating the results
figure;
% Assuming theta representing the pendulum angles (in radians)
theta = x(:,1);  % Actual vector of theta values
time = linspace(0, simTime, length(theta));  % Create a time vector for sim time seconds

% Getting the length of theta
len_theta = length(theta);

% Define x and y positions as a function of theta
x_pos = @(t) sin(interp1(time, theta, t));   % Interpolate theta for continuous time t
y_pos = @(t) cos(interp1(time, theta, t));  % Interpolate theta for continuous time t

% Set up animation loop
fanimator(@(t) plot(x_pos(t), y_pos(t), 'ko', 'MarkerFaceColor', 'k'), 'AnimationRange', [0 simTime], 'FrameRate', len_theta/simTime);
hold on;

% Plot the line connecting the origin to the pendulum bob
fanimator(@(t) plot([0 x_pos(t)], [0 y_pos(t)], 'k-'), 'AnimationRange', [0 simTime], 'FrameRate', len_theta/simTime);

% Display the timer as text (t maps to time directly)
fanimator(@(t) text(-0.3, 0.3, "Timer: " + num2str(t, 2) + " s"), 'AnimationRange', [0 simTime], 'FrameRate', len_theta/simTime);

% Display the animation
hold off;
playAnimation;