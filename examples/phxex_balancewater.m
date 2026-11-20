function phxex_balancewater(Kp, Kd)
% PHXEX_BALANCEWATER Balancing a half-filled water ball on a tilting plate
%
% A variation of phxex_balance: the ball is a hollow sphere twice the size,
% filled about halfway with water. The same PD controller tilts the plate
% to bring the ball back to the centre, but the water does not roll with
% the shell - it slides and sloshes inside, shifting the weight the
% controller has to fight. The plate tilts at a limited rate, as a real
% actuator would: a jump of the tilt would slam the heavy ball.
%
% Input Arguments:
%     Kp - proportional gain on ball position error
%     Kd - derivative gain on ball velocity
%
% Example:
%     phxex_balancewater          % lightly damped: the ball rolls back and forth
%     phxex_balancewater(0.6, 0.4)    % the gains of phxex_balance: settles sooner

%   Copyright 2026 HUMUSOFT s.r.o.

    arguments
        Kp (1, 1) double = 0.6
        Kd (1, 1) double = 0.2
    end

    % Figure setup
    figure(1);
    [viewer, ax] = phx.extra.Viewer("clear", "DefaultCameraTarget", [0 0 0.4], "DefaultCameraPosition", [4.5 -4.5 4]);

    % Plate geometry
    plateSize = [4 4 0.2];
    ballR = 0.6;
    wall = 0.03;

    % Kinematic plate - we drive its tilt; the ball only feels it via contact
    plate = phx.Body(ax, "Type", "kinematic", "Position", [0 0 0], ...
        "Shape", {"Box", "Size", plateSize, "Color", 1, "Texture", "xor", "TextureBlend", 0.5});

    % Hollow ball resting off-centre on the plate, its wall half see-through;
    % a checkerboard of 8 x 4 squares shows it rolling
    p0 = [1.2 1.2 plateSize(3)/2 + ballR];
    ball = phx.Body(ax, "Position", p0, "Shape", {"SphereShell", "Radius", ballR, "WallThickness", wall, ...
        "Color", [1 0.5 0.2], "Material", "glossy", "Texture", phx.base.Shape.checker([1024 512], [8 4]), ...
        "TextureBlend", 0.5, "Alpha", 0.5});

    % Water filling the lower half of the cavity, particles 4 cm apart
    d = 0.04;
    r = ballR - wall - d/2;
    [x, y, z] = ndgrid(-r:d:r, -r:d:r, -r:d:0);
    P = [x(:) y(:) z(:)];
    P = P(vecnorm(P, 2, 2) <= r, :) + p0;
    phx.Liquid(ax, "Positions", P, "Spacing", d, "SoundSpeed", 10, "Viscosity", 0.1);

    % Visual trail of the ball path on the plate
    phx.Trace(ball, "TracePoints", 400, "Color", [1 0.7 0.3]);

    % Log the ball position and the plate tilt
    logBall  = phx.Logger(ball,  "Frequency", 50, "Parameters", "Position");
    logPlate = phx.Logger(plate, "Frequency", 50, "Parameters", "EulerAngles");

    % Closed-loop balancing: the PD control law runs every step as a
    % phx.Function bound to the ball (sensor) and the plate (actuator)
    g = 9.81;
    maxTilt = 0.25;            % clamp the plate tilt for realism [rad]
    maxRate = 0.5;             % fastest tilting of the plate [rad/s]
    phx.Function({ball, plate}, @(o, p, dt, ~) balanceLaw(p, dt, Kp, Kd, g, maxTilt, maxRate));

    % Simulation; contact tolerances of the order of the water spacing and
    % steps short enough for the water
    sim = phx.Simulation(ax, "EngineSettings", phx.engine.Settings("LengthScale", d, "Threads", 8));

    dt = 0.01;
    for k = 1:2500
        sim.step(dt, 10);

        if mod(k, 10) == 0
            viewer.displayText(sprintf("err = [% .2f % .2f] m   |   tilt = [% .2f % .2f] rad", ...
                ball.Position(1), ball.Position(2), plate.EulerAngles(1), plate.EulerAngles(2)));
        end

        % Stop early if the ball has fallen off the plate
        if ball.Position(3) < -1
            viewer.displayText("Ball fell off the plate!");
            break;
        end
    end
    delete(sim);

    % Plot ball position error and plate tilt over time
    clf(figure(2));
    bp = logBall.getChannel(1);            % ball Position [x y z]
    pt = logPlate.getChannel(1);           % plate EulerAngles [x y z]
    subplot(2, 1, 1);
    plot(logBall.Time, bp(:, 1), logBall.Time, bp(:, 2), "LineWidth", 1.3);
    grid on; ylabel("ball pos [m]"); legend("x", "y");
    title(sprintf("Water ball balancing (Kp = %.1f, Kd = %.1f)", Kp, Kd));
    subplot(2, 1, 2);
    plot(logPlate.Time, pt(:, 1)*180/pi, logPlate.Time, pt(:, 2)*180/pi, "LineWidth", 1.3);
    grid on; xlabel("time [s]"); ylabel("plate tilt [deg]"); legend("about X", "about Y");

end

function balanceLaw(parents, dt, Kp, Kd, g, maxTilt, maxRate)
% PD control law of phxex_balance: map the ball's in-plane position and
% velocity error to a small plate tilt that rolls the ball back toward the
% centre (a solid ball on an incline accelerates as a = (5/7) g sin(theta)),
% approached at no more than maxRate.
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

    % Move towards that tilt at the limited rate
    current = plate.EulerAngles(1:2);
    change = max(min([tiltX tiltY] - current, maxRate*dt), -maxRate*dt);
    plate.EulerAngles = [current + change 0];
end
