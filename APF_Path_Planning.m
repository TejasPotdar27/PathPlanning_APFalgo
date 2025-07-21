function APF_Path_Planning()
% Robot Path Planning using Artificial Potential Field Algorithm
% Enhanced version with collision checking and visualization

clf; clear; clc;

% ========== PARAMETERS ==========
rho = 0.8;          % Obstacle influence range
zeta = 0.1;         % Repulsive force coefficient
step = 0.09;        % Movement step size
sensor_range = 0.5; % Sensor detection radius
max_iterations = 1000; % Safety limit

% ========== ENVIRONMENT SETUP ==========
arena_limits = [0 10 0 10];
theta = linspace(0, 2*pi, 30).'; % Sensor ray angles

% Define obstacles as [x_min, x_max, y_min, y_max]
obstacles = {
    [1.5, 2.5, 1, 2]   % Obstacle 1
    [1, 2, 3.5, 4.5]    % Obstacle 2
    [7, 9, 1.5, 2.5]    % Obstacle 3
    [3, 6, 4, 6]        % Obstacle 4
    [5, 8.2, 7.5, 8.5]  % Obstacle 5
};

% ========== VISUALIZATION SETUP ==========
figure('Color', 'white', 'Position', [100, 100, 800, 800]);
hold on; grid on; axis equal;
xlim([0 10]); ylim([0 10]);
title('APF Path Planning');
xlabel('X Position'); ylabel('Y Position');

% Plot arena boundaries
rectangle('Position', [0,0,10,10], 'LineWidth', 2);

% Plot obstacles
for i = 1:length(obstacles)
    obs = obstacles{i};
    rectangle('Position', [obs(1), obs(3), obs(2)-obs(1), obs(4)-obs(3)], ...
              'FaceColor', [0.7, 0.9, 1], 'EdgeColor', 'b', 'LineWidth', 1.5);
end

% ========== USER INPUT ==========
% Start point selection
valid_start = false;
while ~valid_start
    uiwait(msgbox('Select START point outside obstacles','START'));
    [sx, sy] = ginput(1);
    valid_start = ~is_point_in_obstacle([sx, sy], obstacles);
    if ~valid_start
        warndlg('Start point inside obstacle! Select new point.', 'Invalid');
    end
end
start = [sx, sy];
plot(start(1), start(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);

% Goal point selection
valid_goal = false;
while ~valid_goal
    uiwait(msgbox('Select GOAL point outside obstacles','GOAL'));
    [gx, gy] = ginput(1);
    valid_goal = ~is_point_in_obstacle([gx, gy], obstacles);
    if ~valid_goal
        warndlg('Goal point inside obstacle! Select new point.', 'Invalid');
    end
end
goal = [gx, gy];
plot(goal(1), goal(2), 'r*', 'MarkerSize', 15, 'LineWidth', 2);

% ========== PATH PLANNING ==========
position = start;
path = position; % Store entire path
h_robot = plot(position(1), position(2), 'ko', 'MarkerSize', 8, 'LineWidth', 2);
h_path = plot(position(1), position(2), 'b-', 'LineWidth', 1.5);
h_sensors = gobjects(length(theta), 1); % Sensor ray handles

% Calculate GNRON (Goal Navigation Radius of Obstacle Nearness)
GNRON = calculate_gnron(goal, obstacles);

% Main navigation loop
iteration = 0;
distance_to_goal = norm(position - goal);

while distance_to_goal > 2*step && iteration < max_iterations
    iteration = iteration + 1;
    
    % Calculate distances to all obstacles
    [d_min, closest_points] = calculate_obstacle_distances(position, obstacles);
    [d_star, idx] = min(d_min);
    closest_point = closest_points(idx, :);
    
    % Calculate vectors
    to_goal = goal - position;
    to_obstacle = position - closest_point;
    
    % Normalize vectors
    if norm(to_goal) > 0
        attractive_dir = to_goal / norm(to_goal);
    else
        attractive_dir = [0, 0];
    end
    
    if norm(to_obstacle) > 0
        repulsive_dir = to_obstacle / norm(to_obstacle);
    else
        repulsive_dir = [0, 0];
    end
    
    % Calculate forces
    attractive_force = attractive_dir;
    
    if d_star <= rho
        repulsive_magnitude = zeta * (1/d_star - 1/rho) * (norm(to_goal)/d_star);
        repulsive_force = repulsive_magnitude * repulsive_dir;
    else
        repulsive_force = [0, 0];
    end
    
    % Special case for near obstacles at goal
    if d_star < 0.5*rho && norm(to_goal) < GNRON
        repulsive_force = repulsive_force * 2; % Boost repulsion
    end
    
    % Combine forces
    total_force = attractive_force + repulsive_force;
    if norm(total_force) > 0
        movement = step * total_force / norm(total_force);
    else
        movement = [0, 0];
    end
    
    % Update position
    position = position + movement;
    path = [path; position];
    
    % Update visualization
    set(h_robot, 'XData', position(1), 'YData', position(2));
    set(h_path, 'XData', path(:,1), 'YData', path(:,2));
    
    % Update sensor rays
    for i = 1:length(theta)
        sensor_end = position + sensor_range * [cos(theta(i)), sin(theta(i))];
        
        if ishandle(h_sensors(i))
            set(h_sensors(i), 'XData', [position(1), sensor_end(1)], ...
                              'YData', [position(2), sensor_end(2)]);
        else
            h_sensors(i) = plot([position(1), sensor_end(1)], ...
                                [position(2), sensor_end(2)], 'O:');
        end
    end
    
    % Check collision
    if is_point_in_obstacle(position, obstacles)
        warndlg('Collision detected! Stopping simulation.', 'Critical Error');
        break;
    end
    
    % Update goal distance
    distance_to_goal = norm(position - goal);
    drawnow;
end

% Final status
if distance_to_goal <= 2*step
    text(5, 9.5, 'SUCCESS: Goal Reached!', 'FontSize', 14, ...
        'FontWeight', 'bold', 'Color', 'green', 'HorizontalAlignment', 'center');
else
    text(5, 9.5, 'FAILED: Max Iterations Reached', 'FontSize', 14, ...
        'FontWeight', 'bold', 'Color', 'red', 'HorizontalAlignment', 'center');
end

% Add legend
legend([h_robot, h_path, h_sensors(1)], ...
    {'Robot', 'Path', 'Sensors'}, 'Location', 'northeast');

hold off;
end

% ========== HELPER FUNCTIONS ==========
function inside = is_point_in_obstacle(point, obstacles)
% Check if point is inside any obstacle
inside = false;
for i = 1:length(obstacles)
    obs = obstacles{i};
    if point(1) >= obs(1) && point(1) <= obs(2) && ...
       point(2) >= obs(3) && point(2) <= obs(4)
        inside = true;
        return;
    end
end
end

function [d_min, closest_points] = calculate_obstacle_distances(point, obstacles)
% Calculate distances to all obstacles
d_min = zeros(length(obstacles), 1);
closest_points = zeros(length(obstacles), 2);

for i = 1:length(obstacles)
    obs = obstacles{i};
    [d_min(i), xo, yo] = min_dist_obt2(obs(1), obs(3), obs(2), obs(4), point(1), point(2));
    closest_points(i,:) = [xo, yo];
end
end

function GNRON = calculate_gnron(goal, obstacles)
% Calculate Goal Navigation Radius of Obstacle Nearness
min_dist = inf;
for i = 1:length(obstacles)
    obs = obstacles{i};
    [d, ~, ~] = min_dist_obt2(obs(1), obs(3), obs(2), obs(4), goal(1), goal(2));
    if d < min_dist
        min_dist = d;
    end
end
GNRON = min_dist * 0.7; % Safety margin
end