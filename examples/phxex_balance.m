function phxex_balance(Kp, Kd)
% PHXEX_BALANCE Balancing a ball on a tilting plate (ball-and-plate)
%
% A ball is dropped from a random position onto a square plate. The plate
% is a kinematic body that we tilt about its X and Y axes. A PD controller
% reads the ball's position and velocity each step and computes the plate
% tilt that drives the ball back toward the centre - a closed control loop
% running in MATLAB around the physics engine.
%
% The ball is not constrained to the plate in any way: it stays on the
% surface purely through contact, and it rolls because of the contact
% between ball and plate as the plate tilts. Without a contact/collision
% solver there would be no ball-plate interaction to control at all, so
% this example combines collision handling with feedback control.
%
% The control law maps the desired in-plane acceleration to a small plate
% tilt (a ball on an incline accelerates as a = (5/7) g sin(theta), so the
% needed tilt is roughly theta = a / ((5/7) g) for small angles).
%
% Input Arguments:
%     Kp - proportional gain on ball position error
%     Kd - derivative gain on ball velocity
%
% Example:
%     phxex_balance              % default well-tuned response
%     phxex_balance(6, 1.0)      % high Kp, low Kd -> oscillatory
%     phxex_balance(1.5, 3.0)    % sluggish, heavily damped

%   Copyright 2026 HUMUSOFT s.r.o.

    arguments
        Kp (1, 1) double = 0.6
        Kd (1, 1) double = 0.4
    end

    % Resources directory
    resdir = fullfile(fileparts(mfilename("fullpath")), "res", " ");

    % Figure setup
    figure(1);
    [~, ax] = phx.extra.Viewer("clear", "DefaultCameraTarget", [0 0 0], ...
        "DefaultCameraPosition", [4 -4 4]);

    % Plate geometry
    plateSize = [4 4 0.2];
    ballR = 0.3;

    % Catch floor far below in case the ball falls off
    phx.Body(ax, "Type", "static", "Position", [0 0 -6], ...
        "Shape", {"Box", "Size", [40 40 1], "Color", 1});

    % Kinematic plate - we drive its tilt; the ball only feels it via contact
    plate = phx.Body(ax, "Type", "kinematic", "Position", [0 0 0], ...
        "Shape", {"Box", "Size", plateSize, "Style", "edged", ...
                  "Color", [0.6 0.7 0.8], "Texture", resdir+"checker4.png", ...
                  "TextureBlend", 0.3});

    % Ball dropped from a random in-plane position above the plate
    %p0 = (rand(1, 2) - 0.5) .* (plateSize(1:2) - 4*ballR);
    p0 = [1.5 1.5];
    ball = phx.Body(ax, "Position", [p0, 2.0], ...
        "Shape", {"Globe", "Radius", ballR, "Color", [1 0.5 0.2], ...
                  "Material", "shiny", "Texture", resdir+"checker2.png", "TextureBlend", 0.2});
    ball.LinearVelocity = [-1 0 -1];

    % Visual trail of the ball path on the plate
    phx.Trace(ball, "TracePoints", 400, "Color", [1 0.7 0.3]);

    % Log the ball position so the settling behaviour can be analysed
    logBall  = phx.Logger(ball,  "Frequency", 50, "Parameters", "Position");
    logPlate = phx.Logger(plate, "Frequency", 50, "Parameters", "EulerAngles");

    % On-screen readout
    label = uilabel(gcf, "FontSize", 18, "FontColor", [1 1 1], ...
        "Position", [20 20 380 40], "Text", "Dropping ball...");

    % Closed-loop balancing: the PD control law lives in the simulation
    % pipeline as a phx.Function bound to the ball (sensor) and the plate
    % (actuator), so it runs every (sub)step. The run loop below only
    % advances the simulation and updates the on-screen readout.
    g = 9.81;
    maxTilt = 0.25;            % clamp the plate tilt for realism [rad]
    dt = 0.01;
    phx.Function({ball, plate}, @(o, p, ~, ~) balanceLaw(p, Kp, Kd, g, maxTilt));

    % Simulation
    sim = phx.Simulation;

    for k = 1:1500
        sim.step(dt, 1, 1);

        if mod(k, 10) == 0
            label.Text = sprintf("err = [% .2f % .2f] m   |   tilt = [% .2f % .2f] rad", ...
                ball.Position(1), ball.Position(2), ...
                plate.EulerAngles(1), plate.EulerAngles(2));
            pause(0);
        end

        % Stop early if the ball has fallen off the plate
        if ball.Position(3) < -1
            label.Text = "Ball fell off the plate!";
            break;
        end
    end
    delete(sim);

    % Report final settling error
    finalErr = norm(ball.Position(1:2));
    fprintf("Final distance from centre: %.3f m (Kp = %.1f, Kd = %.1f).\n", ...
        finalErr, Kp, Kd);

    % Plot ball position error and plate tilt over time
    figure(2);
    bp = logBall.getChannel(1);            % ball Position [x y z]
    pt = logPlate.getChannel(1);           % plate EulerAngles [x y z]
    subplot(2, 1, 1);
    plot(logBall.Time, bp(:, 1), logBall.Time, bp(:, 2), "LineWidth", 1.3);
    grid on; ylabel("ball pos [m]"); legend("x", "y");
    title(sprintf("Ball-and-plate balancing (Kp = %.1f, Kd = %.1f)", Kp, Kd));
    subplot(2, 1, 2);
    plot(logPlate.Time, pt(:, 1)*180/pi, logPlate.Time, pt(:, 2)*180/pi, "LineWidth", 1.3);
    grid on; xlabel("time [s]"); ylabel("plate tilt [deg]"); legend("about X", "about Y");

end

function balanceLaw(parents, Kp, Kd, g, maxTilt)
% PD control law run each step by the phx.Function: map the ball's in-plane
% position and velocity error to a small plate tilt that rolls the ball back
% toward the centre (a ball on an incline accelerates as a = (5/7) g sin(theta)).
    ball  = parents{1};
    plate = parents{2};
    pos = ball.Position;
    vel = ball.LinearVelocity;

    % desired in-plane acceleration -> plate tilt
    axDes = -Kp*pos(1) - Kd*vel(1);
    ayDes = -Kp*pos(2) - Kd*vel(2);
    k_roll = (5/7)*g;

    % Tilt about Y moves the ball in X, tilt about X moves it in Y.
    tiltY = max(min( axDes/k_roll, maxTilt), -maxTilt);
    tiltX = max(min(-ayDes/k_roll, maxTilt), -maxTilt);

    plate.EulerAngles = [tiltX tiltY 0];
end