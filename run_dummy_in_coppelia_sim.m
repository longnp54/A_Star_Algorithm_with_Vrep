function run_dummy_in_coppelia_sim()
    % Clear the workspace and command window
    clear;
    clc;

    % Initialize the remote API (make sure to have CoppeliaSim running)
    vrep = remApi('remoteApi');
    vrep.simxFinish(-1); % Close any previously opened connections
    clientID = vrep.simxStart('127.0.0.1', 19999, true, true, 5000, 5);

    if clientID > -1
        disp('Connected to CoppeliaSim');

        % Create map and get dummy positions
        [grid_map, dummy_positions] = create_map_and_dummy(vrep, clientID);

        % Move robot along the path of dummy positions
        if ~isempty(dummy_positions)
            moveRobotAlongPath(vrep, clientID, dummy_positions);
        else
            disp('No dummy positions found!');
        end

        % Close the connection to CoppeliaSim
        vrep.simxFinish(clientID);
    else
        disp('Failed to connect to CoppeliaSim');
    end

    % Delete the remote API object
    vrep.delete();
end

function moveRobotAlongPath(vrep, clientID, dummy_positions)
    % Define robot handles and other necessary parameters
    [~, left_motor] = vrep.simxGetObjectHandle(clientID, 'motor_left', vrep.simx_opmode_blocking);
    [~, right_motor] = vrep.simxGetObjectHandle(clientID, 'motor_right', vrep.simx_opmode_blocking);
    [~, robotHandle1] = vrep.simxGetObjectHandle(clientID, 'GPS', vrep.simx_opmode_blocking);
    [~, robotHandle2] = vrep.simxGetObjectHandle(clientID, 'GyroSensor', vrep.simx_opmode_blocking);

    % Stream robot positions and orientations
    [~, Posirobot] = vrep.simxGetObjectPosition(clientID, robotHandle1, -1, vrep.simx_opmode_streaming);
    [~, Orierobot] = vrep.simxGetObjectOrientation(clientID, robotHandle2, -1, vrep.simx_opmode_streaming);

    % Move the robot along the path of dummy positions
    for i = 1:size(dummy_positions, 1)
        goal = [dummy_positions(i, 1), dummy_positions(i, 2)];
        isReached = false;
        while ~isReached
            isReached = move2goal(clientID, vrep, left_motor, right_motor, robotHandle1, robotHandle2, goal);
        end
    end
end

function [isReached] = move2goal(clientID, vrep, left_motor, right_motor, robotHandle1, robotHandle2, goal)
    % Define PID controller parameters
    persistent prevError integError
    if isempty(prevError)
        prevError = 0;
        integError = 0;
    end

    d_m2g = 0.1; % Offset to the goal
    a_m2g = 70; % Linear velocity gain
    Kp = 300; % Proportional gain
    Ki = 0.01; % Integral gain
    Kd = 50; % Derivative gain
    dt = 0.05; % Time step

    R = 0.03; % Wheel radius (m)
    L = 0.1665; % Distance between wheels (m)
    MAX_SPEED = 5; % Maximum wheel speed

    % Get robot position and orientation
    [~, robotPosition] = vrep.simxGetObjectPosition(clientID, robotHandle1, -1, vrep.simx_opmode_buffer);
    robotOxy = [robotPosition(1), robotPosition(2)];

    [~, robotHeading] = vrep.simxGetObjectOrientation(clientID, robotHandle2, -1, vrep.simx_opmode_buffer);
    V_m2g = goal - robotOxy;
    dist2goal = norm(V_m2g);

    % Check if the goal is reached
    if dist2goal < d_m2g
        isReached = true;
    else
        isReached = false;
    end

    % Compute desired orientation and error angle
    desiredOrientation = atan2(V_m2g(2), V_m2g(1));
    errorAngle = desiredOrientation - robotHeading(3);

    if abs(errorAngle) > pi
        errorAngle = errorAngle - sign(errorAngle) * 2 * pi;
    end

    % Update PID errors
    integError = integError + errorAngle * dt;
    derivError = (errorAngle - prevError) / dt;
    prevError = errorAngle;

    % Compute angular velocity (omega)
    omega = Kp * errorAngle + Ki * integError + Kd * derivError;

    % Compute linear velocity (v)
    v = a_m2g * dist2goal;

    % Compute wheel speeds (vr and vl)
    vr = (2 * v + omega * L) / (2 * R);
    vl = (2 * v - omega * L) / (2 * R);

    % Apply speed limits
    vr = max(min(vr, MAX_SPEED), -MAX_SPEED);
    vl = max(min(vl, MAX_SPEED), -MAX_SPEED);

    % Set target velocities
    if isReached == false
