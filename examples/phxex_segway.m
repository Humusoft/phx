function phxex_segway(Kp, Kd, push)
% PHXEX_SEGWAY Self-balancing two-wheeled robot (segway)
%
% An inverted-pendulum body sits on an axle between two wheels. The body is
% inherently unstable - left alone it topples over. A PD controller reads
% the body's pitch angle and pitch rate each simulation step and applies a
% drive torque to the wheels so that the robot drives itself under its own
% centre of mass and stays upright, exactly like a real self-balancing
% scooter.
%
% The wheels propel the robot only through their friction contact with the
% ground. The wheel torque is applied with phx.Body.applyTorque, which acts
% for one step and then resets - a natural control input.
%
% Halfway through, an external disturbance push is applied to the body to
% show the controller recovering from a kick.
%
% Input Arguments:
%     Kp   - proportional gain on body pitch angle
%     Kd   - derivative gain on body pitch rate
%     push - magnitude of the disturbance impulse force
%
% Example:
%     phxex_segway                % balances and recovers from a push
%     phxex_segway(20, 2)         % low gains -> topples over
%     phxex_segway(60, 12, 20)    % strong shove to test recovery

%   Copyright 2026 HUMUSOFT s.r.o.

    arguments
        Kp   (1, 1) double = 4000
        Kd   (1, 1) double = 380
        push (1, 1) double = -40000
    end

    % Resources directory
    resdir = fullfile(fileparts(mfilename("fullpath")), "res", " ");

    % Figure setup
    figure(1);
    [~, ax] = phx.extra.Viewer("clear", "DefaultCameraTarget", [-4 0 0.6], ...
        "DefaultCameraPosition", [-10 -10 4]);

    % Ground
    ground = phx.Body(ax, "Type", "static", "Position", [0 0 -0.1], ...
        "Shape", {"Box", "Size", [40 40 0.2], "Color", 1, "Material", "matte", ...
                  "Texture", resdir+"checker4.png", "TextureBlend", 0.2}, ...
        "Friction", [0.9 0 0]);

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

    % Two wheels as cylinders, axis along Y (the track direction)
    wheelShape = {"Cylinder", "Diameter", 2*wheelRad, "Height", wheelW, ...
                  "Color", [0.2 0.2 0.2], "Texture", resdir+"checker4.png", "TextureBlend", 0.5};
    wheelL = phx.Body(ax, "Position", [0  track/2 axleZ], "EulerAngles", [pi/2 0 0], ...
        "Shape", wheelShape, "Friction", 1);
    wheelR = phx.Body(ax, "Position", [0 -track/2 axleZ], "EulerAngles", [pi/2 0 0], ...
        "Shape", wheelShape, "Friction", 1);

    % Revolute joints connect each wheel to the body, spinning about Y.
    phx.RevoluteJoint(body, wheelL, "PointA", [0  track/2 -(bodyH/2)], ...
        "PointB", [0 0 0], "AxisA", [0 1 0], "AxisB", [0 0 -1]);
    phx.RevoluteJoint(body, wheelR, "PointA", [0 -track/2 -(bodyH/2)], ...
        "PointB", [0 0 0], "AxisA", [0 1 0], "AxisB", [0 0 -1]);

    % Logger for the body pitch angle
    logBody = phx.Logger(body, "Frequency", 100, "Parameters", "EulerAngles");

    phx.Camera(ground, body, "PointA", [-8 -6 2.5]);

    % On-screen readout
    label = uilabel(gcf, "FontSize", 18, "FontColor", [1 1 1], ...
        "Position", [20 20 380 40], "Text", "Balancing...");

    sim = phx.Simulation;

    dt = 0.005;             % small step - balancing needs a fast loop
    nSteps = 1200;
    pushStep = round(nSteps/4);
    for k = 1:nSteps
        % --- sensing: body pitch (tilt about Y) and pitch rate ---
        pitch = body.EulerAngles(2);
        pitchRate = body.AngularVelocity(2);

        % --- control law: PD on the pitch, output is wheel drive torque ---
        % Driving the wheels forward pulls the base under the body and
        % corrects a forward lean (sign tuned for this convention).
        tau = Kp*pitch + Kd*pitchRate;

        % --- actuation: equal torque on both wheels (about their spin axis Z
        % in local frame, since wheels were rotated by pi/2 about X) ---
        wheelL.applyTorque(-[0 0  tau], true);
        wheelR.applyTorque(-[0 0 tau], true);

        % External disturbance: a sideways shove on the body
        if k == pushStep
            body.applyForce([push 0 0], [], false);
            label.Text = "Disturbance push!";
        end

        sim.step(dt, 1, 1);

        if mod(k, 20) == 0
            label.Text = sprintf("pitch = %6.1f deg   |   torque = %5.1f Nm", ...
                pitch*180/pi, tau);
            pause(0);
        end

        % Stop if the robot has fallen over
        if abs(pitch) > pi/3
            label.Text = "Fell over!";
            sim.step(dt, 40, 4);            % let it settle for the view
            break;
        end
    end
    delete(sim);

    % Report
    finalPitch = body.EulerAngles(2)*180/pi;
    if abs(finalPitch) < 15
        fprintf("Stayed upright (final pitch %.1f deg, Kp = %.0f, Kd = %.0f).\n", ...
            finalPitch, Kp, Kd);
    else
        fprintf("Toppled (final pitch %.1f deg, Kp = %.0f, Kd = %.0f).\n", ...
            finalPitch, Kp, Kd);
    end

    % Plot body pitch over time
    figure(2);
    e = logBody.getChannel(1);             % EulerAngles [x y z]
    plot(logBody.Time, e(:, 2)*180/pi, "LineWidth", 1.4);
    grid on; xlabel("time [s]"); ylabel("body pitch [deg]");
    title(sprintf("Self-balancing segway (Kp = %.0f, Kd = %.0f)", Kp, Kd));
    yline(0, "--k");
    xline(pushStep*dt, "--r", "push");

end