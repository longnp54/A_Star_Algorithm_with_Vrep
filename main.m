function main()
    % Delete all previous commands
    clear;
    clc;
    % Installize connection with CoppeliaSim (ensure Coppelia is running)
    vrep = remApi('remoteApi');
    vrep.simxFinish(-1); % Kill a the previous connection
    clientID = vrep.simxStart('127.0.0.1', 19999, true, true, 5000, 5);

    if clientID > -1
        disp('Connected to CoppeliaSim');
    
        % Setting up mapsize
        map_size = [50, 50];
    
        % Definition Obstacles
        obstacles = defineObstacles(map_size);
    
        % Create grid
        grid_map = generate_grid(map_size, obstacles);
    
        % Definition Start and Goal point in map
        start_point = [4, 6];
        goal_point = [44, 44];
    
        % Find the shortest path by A* Algorithm
        path = A_star(grid_map, start_point, goal_point);
    
        % Plot path
        plot_grid(grid_map);
        hold on;
        plot_path(path, start_point, goal_point);
        % Send the path and dummies to CoppeliaSim 
        if ~isempty(path)
            dummy_positions = sendPathToCoppeliaSim(vrep, clientID, path);
            moveRobotAlongPath(vrep, clientID, dummy_positions);
        else
            disp('No path Found!');
        end
    % Ensure robot is completed the path before closing connection
    pause(1);
    vrep.simxFinish(clientID);
    else
        disp('Failed to connect to CoppeliaSim');
    end
        % Delete object by API
        vrep.delete();
end
%function create obstacles
function obstacles = defineObstacles(map_size)
  obs0 = [1:map_size(1), ...
                map_size(1):map_size(1):map_size(1) * map_size(2), ...
                1:map_size(1):(map_size(1) - 1) * map_size(2), ...
                map_size(1) * (map_size(2) - 1):map_size(1) * map_size(2)];
    obs1 = [370:1:380,...
        420:1:430,... 
        470:1:480,... 
        520:1:530,... 
        570:1:580,... 
        620:1:630,... 
        670:1:680,... 
        720:1:730,... 
        770:1:780,... 
        820:1:830,... 
        870:1:880,... 
        920:1:930];
    obs2 = [1215:1:1220, ...
        1265:1:1270, ...
        1315:1:1320, ...
        1365:1:1370, ...
        1415:1:1420, ...
        1465:1:1470, ...
        1515:1:1520, ...
        1565:1:1570, ...
        1615:1:1620];
    obs3 = [1435:1:1440, ...
        1485:1:1490, ...
        1535:1:1540, ...
        1585:1:1590, ...
        1635:1:1640, ...
        1685:1:1690, ...
        1735:1:1740, ...
        1785:1:1790, ...
        1835:1:1840, ...
        1885:1:1890, ...
        1935:1:1940, ...
        1985:1:1990];
    obstacles = [obs0, obs1, obs2, obs3];
end
% functon define grid, and obstacles
function grid_map = generate_grid(size, obstacle)
    % empty node is defined 1
    grid_map = ones(size(1), size(2));
    % obstacles is defined 2
    grid_map(obstacle) = 2;
end
% function plot full the map and path when completed
function plot_grid(grid_map)
    cmap = [1 1 1; 0 0 0];
    colormap(cmap);
    [rows, cols] = size(grid_map);
    image(1.5, 1.5, grid_map);
    grid on;
    set(gca, 'xtick', 1:cols, 'ytick', 1:rows);
    axis image;
    set(gca, 'YDir', 'normal');
    for row = 1:rows, line([1, cols + 1], [row, row]); end
    for col = 1:cols, line([col, col], [1, rows + 1]); end
end

function plot_path(path, start_point, goal_point)
    plot(start_point(2), start_point(1), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    plot(goal_point(2), goal_point(1), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
    for i = 1:size(path, 1), plot(path(i, 2), path(i, 1), 'g.', 'MarkerSize', 20); end
end
% Function exctuting A* algorithm
function path = A_star(grid_map, start, goal)
    % Definition of heuristic function (Euclidean distance)
    heuristic = @(a, b) norm(a - b);

    % Initialize open_list (using containers.Map)
    open_list = containers.Map('KeyType', 'char', 'ValueType', 'any');

   
    % Initialize closed_list (boolean matrix to keep track of approved nodes)
    closed_list = false(size(grid_map));

   
    % Create a start node
    start_node = struct('pos', start, 'g', 0, 'h', heuristic(start, goal), 'parent', []);
    start_node.f = start_node.g + start_node.h;
    open_list(mat2str(start)) = start_node;

    while ~isempty(open_list)
        % Find the node with the lowest f value in the open_list
        keys_open_list = keys(open_list);
        [~, idx] = min(cellfun(@(k) open_list(k).f, keys_open_list));
        current_key = keys_open_list{idx};  
        current_node = open_list(current_key);

        % Checks if the current node is a target
        if isequal(current_node.pos, goal)    
            % Regenerates the path from the target to the starting point
            path = [];
            while ~isempty(current_node.parent)
                path = [current_node.pos; path];
                current_node = current_node.parent;
            end
            path = [current_node.pos; path];
            return;
        end

        % Move the current node from open_list to closed_list
        remove(open_list, current_key);
        closed_list(current_node.pos(1), current_node.pos(2)) = true;

        % Create adjacent nodes
        for dx = -1:1
            for dy = -1:1
                if dx == 0 && dy == 0
                    % Ignore the current node
                    continue; 
                end
                neighbor_pos = current_node.pos + [dx, dy];
                % Ignore obstructive positions and go beyond the boundary
                if any(neighbor_pos < 1) || any(neighbor_pos > size(grid_map)) || grid_map(neighbor_pos(1), neighbor_pos(2)) == 2
                    continue;
                end
                % Diagonal movement cost
                if dx ~= 0 && dy ~= 0
                    cost = sqrt(2);
                % Cost of moving horizontally or vertically
                else
                    cost = 1;
                end
                % Ignore approved nodes
                if closed_list(neighbor_pos(1), neighbor_pos(2))
                    continue; 
                end
                neighbor_key = mat2str(neighbor_pos);
                % Update travel costs
                g = current_node.g + cost;
                h = heuristic(neighbor_pos, goal);
                f = g + h;
                % Ignore if a neighbor is already in the open list with a lower f value
                if isKey(open_list, neighbor_key) && open_list(neighbor_key).f <= f
                    continue; 
                end
                % Thêm hàng xóm vào open list
                open_list(neighbor_key) = struct('pos', neighbor_pos, 'g', g, 'h', h, 'f', f, 'parent', current_node);
            end
        end

    % Không tìm thấy đường đi
    path = [];
    end
end 


function dummy_positions = sendPathToCoppeliaSim(vrep, clientID, path)
    % Initialize dummy_positions list
    dummy_positions = [];

    % Browse through each point in the path (path)
    for i = 1:size(path, 1)
        pos = path(i, :);
        x = (pos(2) - 1) * 0.25; % Tính toán tọa độ x
        y = (pos(1) - 1) * 0.25; % Tính toán tọa độ y
        z = 0; % Tọa độ z (mặt phẳng)
        colorArray = [0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0]; % Mảng màu cho dummy object
        [res, dummyHandle] = vrep.simxCreateDummy(clientID, 0.1, colorArray, vrep.simx_opmode_blocking);
        if res == vrep.simx_return_ok
            % Đặt vị trí cho dummy object
            vrep.simxSetObjectPosition(clientID, dummyHandle, -1, [x, y, z], vrep.simx_opmode_blocking);
            dummy_positions = [dummy_positions; [x, y]]; % Add coordinates to dummy_positions list
        end
    end
end

% function inistallize sensors, motors of Robot on Vrep
function moveRobotAlongPath(vrep, clientID, dummy_positions)
    [~, left_motor] = vrep.simxGetObjectHandle(clientID, 'motor_left', vrep.simx_opmode_blocking);
    [~, right_motor] = vrep.simxGetObjectHandle(clientID, 'motor_right', vrep.simx_opmode_blocking);
    [~, robotHandle1] = vrep.simxGetObjectHandle(clientID, 'GPS', vrep.simx_opmode_blocking);
    [~, robotHandle2] = vrep.simxGetObjectHandle(clientID, 'GyroSensor', vrep.simx_opmode_blocking);

    [~, Posirobot] = vrep.simxGetObjectPosition(clientID, robotHandle1, -1, vrep.simx_opmode_streaming);
    [~, Orierobot] = vrep.simxGetObjectOrientation(clientID, robotHandle2, -1, vrep.simx_opmode_streaming);

    for i = 1:size(dummy_positions, 1)
        goal = [dummy_positions(i, 1), dummy_positions(i, 2)];
        isReached = false;
        while ~isReached
            isReached = move2goal(clientID, vrep, left_motor, right_motor, robotHandle1, robotHandle2, goal);
        end
    end
    
    % Ensure the robot has fully stopped
    pause(1); % Add a delay to ensure the robot has stopped
    vrep.simxSetJointTargetVelocity(clientID, left_motor, 0, vrep.simx_opmode_oneshot);
    vrep.simxSetJointTargetVelocity(clientID, right_motor, 0, vrep.simx_opmode_oneshot);
end

function [isReached] = move2goal(clientID, vrep, left_motor, right_motor, robotHandle1, robotHandle2, goal)
    persistent prevError integError
    if isempty(prevError)
        prevError = 0;
        integError = 0;
    end
     
    d_m2g = 0.1; % offset to goal
    a_m2g = 70;
    % PID parameters
    Kp = 700;
    Ki = 0;
    Kd = 0;
    dt = 0.05;

    R = 0.03; % wheel's radius (m)
    L = 0.1665; % the length between two wheels (m)
    MAX_SPEED = 5;

    [~, robotPosition] = vrep.simxGetObjectPosition(clientID, robotHandle1, -1, vrep.simx_opmode_buffer);
    robotOxy = [robotPosition(1), robotPosition(2)];

    [~, robotHeading] = vrep.simxGetObjectOrientation(clientID, robotHandle2, -1, vrep.simx_opmode_buffer);
    V_m2g = goal - robotOxy;
    dist2goal = norm(V_m2g);

    if dist2goal < d_m2g
        isReached = true;
    else
        isReached = false;
    end

    desiredOrientation = atan2(V_m2g(2), V_m2g(1));
    errorAngle = desiredOrientation - robotHeading(3);

    if abs(errorAngle) > pi
        errorAngle = errorAngle - sign(errorAngle) * 2 * pi;
    end
    
    integError = (integError + errorAngle) * dt;
    derivError = (errorAngle - prevError) / dt;
    prevError = errorAngle;

    omega = Kp * errorAngle + Ki * integError + Kd * derivError;
    v = a_m2g * dist2goal;
    % 
    vr = (2 * v + omega * L) / (2 * R);  
    vl = (2 * v - omega * L) / (2 * R);

    vr = max(min(vr, MAX_SPEED), -MAX_SPEED);
    vl = max(min(vl, MAX_SPEED), -MAX_SPEED);

    % Set target velocity (stop if the robot reached goal)
    if isReached == false
        vrep.simxSetJointTargetVelocity(clientID, left_motor, vl, vrep.simx_opmode_oneshot);
        vrep.simxSetJointTargetVelocity(clientID, right_motor, vr, vrep.simx_opmode_oneshot);
    else
        vrep.simxSetJointTargetVelocity(clientID, left_motor, 0, vrep.simx_opmode_oneshot);
        vrep.simxSetJointTargetVelocity(clientID, right_motor, 0, vrep.simx_opmode_oneshot);
    end
    disp(goal);
end
