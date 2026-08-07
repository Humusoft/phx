function phxex_swingover
% PHXEX_SWINGOVER Swing a pendulum load over a wall, then knock the wall down
%
% A trolley rides a horizontal rail on a phx.PrismaticJoint and carries a
% rigid arm with a heavy ball on a phx.RevoluteJoint - a cart with a
% pendulum, driven only by a horizontal force on the trolley. A wall of
% dry-stacked bricks stands in the middle of the rail, taller than the
% hanging ball, so the load cannot be driven across: it has to go over the
% top.
%
% The controller works in four phases. It first pumps the pendulum at
% resonance, pushing the trolley against the swing (a force proportional to
% the swing rate) until the amplitude clears the wall with room to spare -
% this needs a tenth of the actuator force. A quarter of a swing later it
% releases the trolley: a sustained thrust carries the trolley across while
% the load hangs back and high, so the ball passes over the bricks. Once the
% ball is clear, an LQR state feedback catches the swing and parks the
% trolley at the far end. Finally the trolley simply drives back to where it
% started - and the hanging ball, now at rest, takes the wall with it.
%
% See also phx.PrismaticJoint, phx.RevoluteJoint, phx.Simulation, phx.Trace

%   Copyright 2026 HUMUSOFT s.r.o.

    % --- Mechanism ------------------------------------------------------
    zPivot = 1.75;               % hinge height above the ground (m)
    armLen = 1.2;                % hinge to ball centre (m)
    rBall = 0.13;                % ball radius (m)
    mBall = 14;                  % ball mass (kg)
    mTrolley = 8;                % trolley mass (kg)

    % --- Wall (dry-stacked blocks) --------------------------------------
    wallSize = [1.5 0.1 1];
    wallHalf = wallSize(2)/2;

    % --- Travel and control ---------------------------------------------
    xStart = -2.2;               % parking position (m)
    xEnd = 2.6;                  % target position beyond the wall (m)
    thGo = 70*pi/180;            % swing amplitude to build up before the run
    tau = 0.6;                   % release delay after the swing passes bottom (s)
    fPump = 30;                  % force limit while pumping (N)
    kPump = 250;                 % pump gain, force per unit swing rate (Ns)
    kPark = [250 120];           % holds the trolley in place while pumping
    fMax = 300;                  % actuator force limit (N)
    fDash = 250;                 % thrust during the run across (N)
    vCruise = 3.5;               % speed cap during the run across (m/s)
    margin = 0.15;               % how far past the wall the ball must be (m)

    % LQR gains for the linearized trolley-pendulum (state
    % [x-xEnd; dx/dt; sin(angle); rate*cos(angle)], Q = diag([3000 300 1e4 3000]), R = 1)
    K = [54.8 63.6 -90.3 -10.9];

    % Angle at which the ball just clears the top of the wall
    thClear = acos((zPivot - wallSize(3) - rBall)/armLen);

    % --- Scene ----------------------------------------------------------
    figure(1);
    [viewer, ax] = phx.extra.Viewer("clear", "DefaultCameraPosition", [-5.8 -5.9 2.2], "DefaultCameraTarget", [-1 0 0.6]);

    % Ground
    phx.Body(ax, "Type", "static", "Position", [0 0 -0.15], "Shape", {"Box", "Size", [12 6 0.3], "Color", 0.85, "Texture", "tiles"});

    % Rail beam on two posts, well clear of the trolley travel, with the
    % trolley hanging under the beam
    zRail = zPivot + 0.23;
    for xPost = [-3.5 3.5]
        phx.Body(ax, "Type", "static", "Position", [xPost 0 (zRail - 0.07)/2], ...
            "Shape", {"Box", "Size", [0.14 0.14 zRail - 0.07], "Color", [0.45 0.47 0.5]});
    end
    rail = phx.Body(ax, "Type", "static", "Position", [0 0 zRail], ...
        "Shape", {"Box", "Size", [7.2 0.14 0.14], "Color", [0.45 0.47 0.5]});
    trolley = phx.Body(ax, "Position", [xStart 0 zPivot + 0.08], "Mass", mTrolley, ...
        "Shape", {"Box", "Size", [0.3 0.2 0.16], "Color", [0.85 0.5 0.15]});
    phx.PrismaticJoint(rail, trolley, "PointA", [0 0 -0.15], ...
        "AxisA", [1 0 0], "AxisB", [1 0 0], "Visible", false);

    % The arm starts a few degrees off vertical - the nudge the pump grows
    beta = -4*pi/180;                       % rotation about y, ball towards +x
    dir = [-sin(beta) 0 -cos(beta)];        % from the hinge towards the ball
    pivot = [xStart 0 zPivot];
    arm = phx.Body(ax, "Position", pivot + armLen/2*dir, "EulerAngles", [0 beta 0], ...
        "Shape", {"Cylinder", "Axis", "z", "Radius", 0.025, "Height", armLen, ...
                  "Color", [0.35 0.37 0.4], "Material", "metal"});
    ball = phx.Body(ax, "Position", pivot + armLen*dir, ...
        "Shape", {"Sphere", "Radius", rBall, "Color", 0.3, "Material", "metal"}, ...
        "Mass", mBall, "Inertia", 2/5*mBall*rBall^2);
    phx.Trace(ball, "TracePoints", 900, "Color", [1 0.8 0.2], "Overlay", true);
    phx.RevoluteJoint(trolley, arm, "PointA", [0 0 -0.08], "PointB", [0 0 armLen/2], ...
        "AxisA", [0 1 0], "AxisB", [0 1 0], "Visible", false);
    phx.FixedJoint(arm, ball, "PointA", [0 0 -armLen/2], "Visible", false);

    % The wall
    phx.assembly.wall("Size", wallSize, "EulerAngles", [0 0 pi/2], "Rows", 6, "Columns", 5, "Color", [0.72 0.4 0.32]);

    sim = phx.Simulation(ax);

    % --- Run ------------------------------------------------------------
    dt = 0.004;                  % small step - the stacked wall needs it
    phase = 1;                   % 1 pump, 2 run across, 3 catch, 4 return
    tPhase = 0; tGo = NaN; swingPeak = 0; tCatch = 4; step = 0;
    log = struct("t", [], "x", [], "th", [], "F", [], "phase", [], "gap", []);
    names = ["pumping the swing", "running across", "catching the swing", "driving back"];
    fPeak = zeros(1, 4);

    sim.step(0.5, 125, 5);       % let the wall settle onto the ground

    t = 0;
    while true
        % --- sensing: trolley state and pendulum angle from vertical ----
        x = trolley.Position(1);
        v = trolley.LinearVelocity(1);
        d = ball.Position - (trolley.Position - [0 0 0.08]);
        th = atan2(d(1), -d(3));
        w = -arm.AngularVelocity(2);
        swingPeak = max(swingPeak, abs(th));

        % --- control law -----------------------------------------------
        switch phase
            case 1   % resonant pumping, stopped at the target amplitude
                F = -kPump*w*cos(th)*(swingPeak < thGo) ...
                    - kPark(1)*(x - xStart) - kPark(2)*v;
                F = max(min(F, fPump), -fPump);
                if isnan(tGo) && swingPeak >= thGo && abs(th) < 0.05 && w > 0
                    tGo = t;
                end
                if ~isnan(tGo) && t >= tGo + tau
                    phase = 2; tPhase = t;
                end
            case 2   % sustained thrust: the load stays back and high
                F = min(fDash, 400*(vCruise - v));
                if x + armLen*sin(th) > wallHalf + rBall + margin
                    phase = 3; tPhase = t;
                end
            case 3   % LQR catch, parking the trolley at the far end
                F = -K*[x - xEnd; v; sin(th); w*cos(th)];
                if t - tPhase > tCatch
                    phase = 4; tPhase = t;
                end
            case 4   % plain smooth move back - this is what hits the wall
                [xr, vr] = trapezoid(t - tPhase, xStart - xEnd, 2.5, 3);
                F = 1500*(xEnd + xr - x) + 800*(vr - v);
        end
        F = max(min(F, fMax), -fMax);
        fPeak(phase) = max(fPeak(phase), abs(F));

        trolley.applyForce([F 0 0], [], false);
        step = step + 1;
        sim.step(dt, 1, mod(step, 2) == 0);
        t = t + dt;

        % --- log: gap between the ball and the wall block ---------------
        p = ball.Position;
        log.t(end + 1) = t;
        log.x(end + 1) = x;
        log.th(end + 1) = th;
        log.F(end + 1) = F;
        log.phase(end + 1) = phase;
        log.gap(end + 1) = hypot(max(abs(p(1)) - wallHalf, 0), ...
            max(p(3) - wallSize(3), 0)) - rBall;

        if mod(step, 25) == 0
            viewer.displayText(sprintf("%s   |   swing %5.1f deg   |   force %6.1f N", ...
                names(phase), th*180/pi, F));
        end
        if (phase == 4 && abs(x - xStart) < 0.05 && abs(v) < 0.05) || t > 30
            break;
        end
    end

    sim.step(1, 250, 2);         % let the rubble come to rest
    viewer.displayText("");
    crossed = log.phase <= 3;
    delete(sim);

    % --- Plots ----------------------------------------------------------
    clf(figure(2));
    subplot(2, 2, [1 2]);
    ballX = log.x + armLen*sin(log.th);
    ballZ = zPivot - armLen*cos(log.th);
    plot(ballX(crossed), ballZ(crossed), "LineWidth", 1.4); hold on;
    plot(ballX(~crossed), ballZ(~crossed), "LineWidth", 1.4);
    rectangle("Position", [-wallHalf 0 2*wallHalf wallSize(3)], "FaceColor", [0.72 0.4 0.32]);
    axis equal; grid on; xlabel("x [m]"); ylabel("z [m]");
    axis([xStart - 0.8, max(ballX) + 0.2, 0, zPivot + 0.2]);
    legend("over the wall", "back through it", "Location", "southeast");
    title("Path of the ball");
    subplot(2, 2, 3);
    plot(log.t, log.th*180/pi, "LineWidth", 1.4); grid on; ylabel("swing [deg]");
    yl = [yline(thClear*180/pi, "--", "clears the wall"), yline(-thClear*180/pi, "--")];
    set(yl, "Color", "r");
    title("Swing angle");
    subplot(2, 2, 4);
    plot(log.t, log.F, "LineWidth", 1.4); grid on;
    xlabel("time [s]"); ylabel("drive force [N]");
    title("Drive force");
end

function [x, v] = trapezoid(t, dist, vMax, aMax)
% Position and speed on a trapezoidal velocity profile (rest-to-rest by dist)
    s = sign(dist); dist = abs(dist);
    if dist < vMax^2/aMax
        vMax = sqrt(dist*aMax);
    end
    tAcc = vMax/aMax;
    tTotal = tAcc + dist/vMax;
    t = min(max(t, 0), tTotal);
    if t < tAcc
        x = aMax*t^2/2;
        v = aMax*t;
    elseif t < tTotal - tAcc
        x = vMax^2/(2*aMax) + vMax*(t - tAcc);
        v = vMax;
    else
        x = dist - aMax*(tTotal - t)^2/2;
        v = aMax*(tTotal - t);
    end
    x = s*x;
    v = s*v;
end