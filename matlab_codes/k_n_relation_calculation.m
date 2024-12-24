% Defining the range for x and the values of k
x_vals = linspace(0, pi, 1000);  
k_values = 1:10;                  % Range of k values

% Initializing array to store tan(x) values for the first root for each k
first_tan_value = zeros(1, length(k_values));

for idx = 1:length(k_values)
    k = k_values(idx);
    
    % Defining the equation as a function handle
    equation = @(x) 2 * sin(x) - (1 + cos((2 * k - 1) * x));
    
    % Finding the first root by detecting sign change and using fzero
    found_root = false;
    for i = 1:length(x_vals) - 1
        if sign(equation(x_vals(i))) ~= sign(equation(x_vals(i + 1)))
            % Refining the root using fzero starting at midpoint
            root = fzero(equation, (x_vals(i) + x_vals(i + 1)) / 2);
            first_tan_value(idx) = tan(root);  % Store tan(x) for first root
            found_root = true;
            break;  % Exiting loop after finding the first root
        end
    end
    
    % Displaying the result for current value of k
    if found_root
        fprintf('Value of tan(x) for k = %d: %f\n', k, first_tan_value(idx));
    else
        fprintf('No root found for k = %d within the range.\n', k);
    end
end

figure;
plot(k_values, first_tan_value, 'o-', 'MarkerSize', 8);
xlabel('k');
ylabel('n value satisfying the equation');
title('n v/s k');
grid on;