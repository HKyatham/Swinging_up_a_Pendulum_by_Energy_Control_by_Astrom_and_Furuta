% Parameters
n = 5; % ratio of maximum acceleration to g
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
simTime = 5;
tspan = [0 simTime];


% State-space model function
function dxdt = pendulumStateSpace(~, x, J, m, g, l, n, ~, ~)
    theta = x(1);
    dtheta = x(2);
    E = 0.5 * J * dtheta^2 + m * g * l * (cos(theta) - 1);
    theta_star = -asin(1 - 1/n);
    % Bang-bang control logic
    if theta <= -pi/2
        u = n * g; % Apply max positive input until horizontal position
    elseif theta > -pi/2 && theta <= theta_star
        u = -n * g; % Switch to max negative input until \theta^*
    elseif theta > theta_star && theta < -0.03
        u = n * g; % Switch back to max positive input until upright
    else
        u = 0;
    end
    
    dxdt = [dtheta;
            (m * g * l * sin(theta) - m * l * u * cos(theta)) / J];
end

% Solving ODE using ode45 with state-space model
[t, x] = ode45(@(t, x) pendulumStateSpace(t, x, J, m, g, l, n, E0), tspan, [theta0, dtheta0]);

% Calculating initial energy
E_initial = 0.5 * J * dtheta0^2 + m * g * l * (cos(theta0) - 1);
E_initial  = E_initial/m*g*l;
fprintf('Initial Energy E: %.2f J\n', E_initial);

% Calculating control signal over time
u_control_g_force = zeros(length(t), 1); % Control signal in terms of g-force
for i = 1:length(t)
    % Initializing theta variable
    theta = x(i, 1);
    % Initializing dtheta
    dtheta = x(i, 2);
    % Defnining Energy as per the equation 2
    theta_star = -asin(1 - 1/n);

    E(i) = 0.5 * J * dtheta^2 + m * g * l * (cos(theta) - 1)/ (m*g*l);
    % Bang-bang control logic
    if theta <= -pi/2
        u = n * g; % Apply max positive input until horizontal position
    elseif theta > -pi/2 && theta <= theta_star
        u = -n * g; % Switch to max negative input until \theta^*
    elseif theta > theta_star && theta < -0.03
        u = n * g; % Switch back to max positive input until upright
    else
        u = 0; % Stop input once upright position is reached
    end
    
    u_control_g_force(i) = u/g;

end
theta = x(:,1);
theta(theta>0) = 0;
% Plotting results
figure;
subplot(3,1,1);
plot(t, theta);
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
ylim([-5.1,5.1])
yticks([-4, 0, 4]); % Set y-axis ticks to -4, 0, and 4
