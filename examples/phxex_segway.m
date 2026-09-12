function phxex_segway(Kp, Kd, push)
% PHXEX_SEGWAY Self-balancing robot scanning its way with a ray sensor
%
% An inverted-pendulum body sits on an axle between two wheels. The body is
% inherently unstable - left alone it topples over. A PD controller reads
% the body's pitch angle and pitch rate each simulation step and applies a
% drive torque to the wheels so that the robot drives itself under its own
% centre of mass and stays upright, exactly like a real self-balancing
% scooter. A speed term added to the same torque makes it cruise forward.
%
% A fan of rays leaves the top of the body and scans the space ahead. The
% steering law is as simple as it gets: the free distance reported by the
% leftmost ray is compared with the rightmost one and the difference is fed
% to the wheels as a differential torque, so the robot turns towards
% whichever side has more room. With nothing in sight the fan reads the same
% on both sides and the robot holds its original heading.
%
% Two walls stand in the way. The tall one is angled across the course, so
% the rays hit its near half while they still run past its far end - the
% asymmetry tells the robot which side is open and it swerves around it.
% The low wall is below the fan: the rays fly clean over it, the robot never
% sees it and drives straight through, which is the blind spot every
% single-plane scanner has.
%
% The wheels propel the robot only through their friction contact with the
% ground. Each wheel is driven by the motor of its phx.RevoluteJoint, run as
% a pure torque source: an unreachable TargetVelocity keeps the motor
% saturated, so it delivers exactly MaxTorque. Because the motor is part of
% the joint, the chassis feels the equal and opposite reaction - the term a
% real segway leans on. On the straight between the two walls an external
% push shoves the body to show the controller recovering from a kick.
%
% Input Arguments:
%     Kp   - proportional gain on body pitch angle
%     Kd   - derivative gain on body pitch rate
%     push - magnitude of the disturbance impulse force
%
% Example:
%     phxex_segway                     % dodges the tall wall, breaks the low one
%     phxex_segway(20, 2)              % low gains -> topples over
%     phxex_segway(6000, 1800, -120000) % strong shove to test recovery

%   Copyright 2026 HUMUSOFT s.r.o.

    arguments
        Kp   (1, 1) double = 6000
        Kd   (1, 1) double = 3000
        push (1, 1) double = -40000
    end

    % Figure setup
    figure(1);
    [viewer, ax] = phx.extra.Viewer("clear", "DefaultCameraTarget", [-4 0 0.6], ...
        "DefaultCameraPosition", [-10 -10 4]);

    % Ground
    phx.Body(ax, "Type", "static", "Position", [-15 0 -0.1], ...
        "Shape", {"Box", "Size", [40 30 0.2], "Color", 1, "Material", "matte", ...
                  "Texture", "checker", "TextureBlend", 0.3}, "Friction", [0.9 0 0]);

    % Dimensions
    wheelRad = 0.35;          % wheel radius
    wheelW = 0.2;           % wheel width
    track  = 0.8;           % distance between wheels
    bodyH  = 1.2;           % height of the body (raises the CoG -> unstable)
    axleZ  = wheelRad;        % axle height above ground

    % Body: a tall block whose centre of mass sits well above the axle,
    % which is what makes the system an unstable inverted pendulum.
    body = phx.Body(ax, "Position", [0 0 axleZ + bodyH/2], ...
        "Shape", {"Box", "Size", [0.5 track-wheelW-0.1 bodyH], "Color", [0.3 0.5 0.9]});

    % Obstacle
    phx.Body(ax, "Position", [-4 0 1], ...
        "Shape", {"Cylinder", "Axis", "y" "Diameter", 0.2, "Height", 2, "Color", [0.9 0.9 0]}, "Friction", 1);
    phx.Body(ax, "Position", [-3.8 0 2], ...
        "Shape", {"Box", "Size", [3 2 0.05], "Color", [0.9 0.5 0.5], "Density", 100}, "Friction", 1);

    % Tall wall, set at an angle across the course so that the ray fan reads
    % a different free distance to the left and to the right of the robot.
    phx.Body(ax, "Type", "static", "Position", [-15.5 1.5 1.5], "EulerAngles", [0 0 pi/4], ...
        "Shape", {"Box", "Size", [7 0.3 3], "Color", [0.6 0.6 0.65], "Texture", "tiles"});

    % Low wall of loose bricks, standing below the ray fan
    bricks = phx.assembly.wall(ax, "Size", [3 0.2 1.0], "Rows", 5, "Columns", 5, ...
        "Position", [-28 -3.4 0], "EulerAngles", [0 0 pi/2], ...
        "Color", [0.8 0.45 0.35], "Density", 900);
    brickPos = reshape([bricks.Position], 3, [])';

    % Two wheels as cylinders, axis along Y (the track direction)
    wheelShape = {"Cylinder", "Diameter", 2*wheelRad, "Height", wheelW, ...
                  "Color", [0.2 0.2 0.2], "Texture", "checker", "TextureBlend", 0.5};
    wheelL = phx.Body(ax, "Position", [0  track/2 axleZ], "EulerAngles", [pi/2 0 0], ...
        "Shape", wheelShape, "Friction", 1);
    wheelR = phx.Body(ax, "Position", [0 -track/2 axleZ], "EulerAngles", [pi/2 0 0], ...
        "Shape", wheelShape, "Friction", 1);

    % Revolute joints connect each wheel to the body, spinning about Y. Their
    % motors are the drive, so the reaction torque lands on the chassis.
    hubL = phx.RevoluteJoint(body, wheelL, "PointA", [0  track/2 -(bodyH/2)], ...
        "PointB", [0 0 0], "AxisA", [0 1 0], "AxisB", [0 0 -1]);
    hubR = phx.RevoluteJoint(body, wheelR, "PointA", [0 -track/2 -(bodyH/2)], ...
        "PointB", [0 0 0], "AxisA", [0 1 0], "AxisB", [0 0 -1]);

    % Ray sensor: a fan leaving the top of the body and aimed forward (its
    % local -X direction). Being anchored to the body, it sweeps with the
    % robot - and pitches with it.
    nRays = 7;
    rayLen = 9;             % sensor range
    fan = linspace(-1, 1, nRays)*deg2rad(35);
    tilt = deg2rad(10);     % lifted off the body plane, so the forward lean
                            % the robot needs to drive does not aim it down
    scan = phx.Raycast(body, "Origins", [0; 0; bodyH/2], "Ends", [0; 0; bodyH/2] + ...
        rayLen*[-cos(fan)*cos(tilt); sin(fan)*cos(tilt); sin(tilt)*ones(1, nRays)]);

    % Loggers for the body pitch angle and for the sensor readings
    logBody = phx.Logger(body, "Frequency", 100, "Parameters", "EulerAngles");
    logScan = phx.Logger(scan, "Frequency", 100, "Parameters", "Distances");

    % Chase camera riding behind the robot and aimed at its axle
    phx.Camera(body, body, "PointA", [7 -3 1], "PointB", [0 -1 1], "TrackingLag", 0.6);

    sim = phx.Simulation;

    % Gains of the two loops added on top of the balancing PD
    vRef = 3;               % cruise speed, m/s
    Kv = 2000;              % speed term -> the lean the robot commands (Kv/Kp)
    Ks = 700;               % steering gain on the left/right ray difference
    Kh = 800;               % heading term holding the original course

    dt = 0.01;              % small step (0.005-0.01) balancing needs a fast loop
    nSteps = 3000;
    pushStep = round(8.5/dt);   % shove it once the tall wall is behind
    seen = false;
    for k = 1:nSteps
        % --- sensing: pose, forward speed, pitch rate and the ray fan ---
        yaw = body.EulerAngles(3);
        pitch = body.EulerAngles(2);
        heading = [-cos(yaw) -sin(yaw) 0];      % where the robot points
        pitchAxis = [-sin(yaw) cos(yaw) 0];     % what it pitches about
        speed = body.LinearVelocity*heading';
        pitchRate = body.AngularVelocity*pitchAxis';

        range = scan.Distances;
        range(isnan(range)) = rayLen;           % a ray that misses reads free

        % --- control law: PD on the pitch plus a speed term, output is the
        % common wheel drive torque. Driving the wheels forward pulls the
        % base under the body and corrects a forward lean (sign tuned for
        % this convention).
        tau = Kp*pitch + Kd*pitchRate + Kv*(vRef - speed);

        % --- steering: turn towards the side with more free space, and back
        % onto the original heading once the way is clear ---
        turn = (range(1) - range(end))/rayLen;
        tauTurn = Ks*turn - Kh*yaw;

        % --- actuation: the common torque plus the steering difference, fed
        % to the hub motors as a signed torque. MaxTorque = 0 when the demand
        % is zero, which switches the motor off rather than braking. ---
        tauL = tau - tauTurn;  tauR = tau + tauTurn;
        hubL.TargetVelocity = sign(tauL)*1e6;  hubL.MaxTorque = abs(tauL);
        hubR.TargetVelocity = sign(tauR)*1e6;  hubR.MaxTorque = abs(tauR);

        % External disturbance: a sideways shove on the body
        if k == pushStep
            body.applyForce([push 0 0], [], false);
            viewer.displayText("Disturbance push!");
        end

        sim.step(dt, 1, 1);

        if ~seen && min(range) < 8
            seen = true;
            viewer.displayText("Wall detected - steering around it");
        elseif mod(k, 20) == 0
            if any(scan.Hits)
                viewer.displayText(sprintf("nearest ray = %4.1f m   |   steering = %5.0f Nm", ...
                    min(range), tauTurn));
            else
                viewer.displayText(sprintf("nothing in sight   |   speed = %4.1f m/s", speed));
            end
        end

        % Stop if the robot has fallen over
        if abs(pitch) > pi/3
            viewer.displayText("Crashed!");
            sim.step(2, 200, 1);            % let it settle for the view
            break;
        end
    end
    delete(sim);

    % Report
    moved = vecnorm(reshape([bricks.Position], 3, [])' - brickPos, 2, 2) > 0.2;
    fprintf("Travelled %.1f m and knocked %d of %d bricks out of the low wall.\n", ...
        -body.Position(1), nnz(moved), numel(bricks));

    % Plot the body pitch and what the sensor saw
    clf(figure(2));
    tiledlayout(2, 1);
    nexttile;
    e = logBody.getChannel(1);             % EulerAngles [x y z]
    plot(logBody.Time, e(:, 2)*180/pi, "LineWidth", 1.4);
    grid on; ylabel("body pitch [deg]");
    title(sprintf("Self-balancing segway (Kp = %.0f, Kd = %.0f)", Kp, Kd));
    yline(0, "--k");
    xline(pushStep*dt, "--r", "push");
    nexttile;
    plot(logScan.Time, min(logScan.getChannel(1), [], 2), "LineWidth", 1.4);
    grid on; xlabel("time [s]"); ylabel("nearest ray [m]");
    ylim([0 rayLen]);

end
