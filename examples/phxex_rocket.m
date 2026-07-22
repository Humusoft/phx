function phxex_rocket(hopHeight)
% PHXEX_ROCKET Rocket hop: takeoff and a controlled landing with one engine
%
% A small rocket stands on a launch pad and has a single phx.Thruster at
% the base of its frame. Unlike the quadcopter, all the control authority
% comes from one engine: the throttle commands the total thrust and the
% engine gimbal - the runtime-writable Direction property of the thruster
% - vectors it by a few degrees to steer the attitude, exactly like a
% real rocket (the drawn exhaust vector shows both the flame length and
% the gimbal deflection).
%
% The guidance is a cascade: a position PD loop turns the waypoint error
% into a desired acceleration, which defines the required thrust and the
% desired tilt of the airframe; an attitude PD loop converts the tilt
% error into body torques; and the gimbal deflection that produces these
% torques is computed from the engine lever arm, saturated at +-10 deg.
%
% The rocket performs a hop between two neighbouring platforms: it lifts
% off, translates sideways and descends to a soft touchdown on the other
% pad, where the engine cuts off. The report and plots show the touchdown
% speed, the tilt, the landing offset, and the throttle and gimbal
% histories.
%
% Input Arguments:
%     hopHeight - apex altitude of the hop (default 6)
%
% Example:
%     phxex_rocket        % default hop
%     phxex_rocket(10)    % higher hop

%   Copyright 2026 HUMUSOFT s.r.o.

    arguments
        hopHeight (1, 1) double {mustBeGreaterThanOrEqual(hopHeight, 4)} = 6
    end

    % Rocket parameters (the STL is scaled to a ~2.8 m tall vehicle)
    scl = 0.06;
    hBase = 1.12;          % engine plane below the center of mass
    mRocket = 50;          % mass (kg)
    iRocket = [40 40 10];  % inertia (kg*m^2)
    maxThrust = 800;       % N, thrust-to-weight ~1.6
    engineLag = 0.2;       % engine time constant
    gimbalMax = tand(15);  % gimbal deflection limit (degrees)
    grav = 9.81;           % gravitational acceleration
    padBx = 5;             % landing pad center x
    padBtilt = 5;          % landing pad tilt (degrees)

    % Controller gains; the attitude loop must be well faster than the
    % position loop (the gimbaled rocket is non-minimum phase laterally)
    KpP = 1.0; KdP = 1.8;          % position -> acceleration
    KattP = 350; KattD = 170;      % tilt -> body torque

    % Resources directory
    resdir = fullfile(fileparts(mfilename("fullpath")), "res", " ");

    % Figure setup
    figure(1);
    [viewer, ax] = phx.extra.Viewer("clear", "DefaultCameraTarget", [padBx/2 0 3.0], ...
        "DefaultCameraPosition", [padBx/2 - 12, -8, 5], "Texture", "defaultNebula");

    % Ground and the two neighbouring platforms
    phx.Body(ax, "Type", "static", "Position", [padBx/2 0 -0.5], ...
        "Shape", {"Box", "Size", [4*padBx, 2*padBx, 1], "Color", [1 1 1]});
    padA = phx.Body(ax, "Type", "static", "Position", [0 0 0.2], ...
        "Shape", {"Box", "Size", [3 3 0.4], "Color", [0.35 0.35 0.4]});
    padB = phx.Body(ax, "Type", "static", "Position", [padBx 0 0.2], "EulerAngles", [padBtilt*pi/180 0 0], ...
        "Shape", {"Box", "Size", [3 3 0.4], "Color", [0.3 0.45 0.3]});

    % The rocket; mass properties are set explicitly
    rocket = phx.Body(ax, "Position", [0 0 0.4 + hBase + 0.01], ...
        "Shape", {resdir+"rocket.stl", "Scale", scl*[1 1 1], "Color", [1 1 1], ...
        "Material", "shiny"});
    rocket.Mass = mRocket;
    rocket.Inertia = iRocket;

    % The single gimbaled engine at the base; the drawn exhaust vector
    % visualizes both the thrust and the gimbal deflection
    eng = phx.Thruster(rocket, "Point", [0 0 -hBase], "Direction", [0 0 1], ...
        "MaxThrust", maxThrust, "TimeConstant", engineLag, ...
        "ForceVectorSize", 0.004, "Color", [1 0.55 0.15]);

    phx.Trace(rocket, "TracePoints", 1200, "Overlay", true, "Color", [1 1 0]);

    % Hop profile: ascend, traverse, descend, touch down on pad B
    wp = [0 0 hopHeight; padBx 0 hopHeight; padBx 0 2.6; padBx 0 1.6];
    wpIdx = 1;

    viewer.displayText("Lift-off...");

    sim = phx.Simulation(ax);
    dt = 0.005;
    subSteps = 10;
    tMax = 60;
    log = struct("t", [], "p", [], "thr", [], "gim", []);
    t = 0;
    tLand = NaN;
    vTouch = NaN;
    while t < tMax && (isnan(tLand) || t < tLand + 2)
        for s = 1:subSteps
            M = rocket.Transform;
            R = M(1:3, 1:3);
            p = rocket.Position;
            v = rocket.LinearVelocity;
            w = rocket.AngularVelocity;

            % Waypoint switching; before starting the descent the rocket
            % must settle into a steady hover above the pad (tight bounds),
            % so it does not carry the traverse momentum into the landing
            e = wp(wpIdx, :) - p;
            if wpIdx == 1
                tol = 0.5; vTol = 1.2;
            else
                tol = 0.35; vTol = 0.7;
            end
            if norm(e) < tol && norm(v) < vTol && wpIdx < size(wp, 1)
                wpIdx = wpIdx + 1;
            end

            % Touchdown: engine cut-off
            if isnan(tLand) && wpIdx == size(wp, 1) && p(3) < 0.4 + hBase + 0.4
                tLand = t;
                vTouch = v(3);
                tilt = acosd(R(3, 3));
            end

            if ~isnan(tLand)
                eng.Throttle = 0;
            else
                % Position loop: desired acceleration, gentle on descent
                aDes = KpP*e - KdP*v;
                aDes(1:2) = max(min(aDes(1:2), 2.5), -2.5);
                aDes(3) = max(min(aDes(3), 5), -3.5);
                Fvec = mRocket*(aDes + [0 0 grav]);

                % Attitude loop: align the body Z axis with the thrust vector
                nDes = Fvec/norm(Fvec);
                nCur = R(:, 3)';
                eAtt = cross(nCur, nDes);
                tauW = KattP*eAtt - KattD*[w(1) w(2) 0];
                tauB = R'*tauW';

                % Throttle and the gimbal deflection producing the torques
                T = max(min(Fvec*nCur', 0.95*maxThrust), 0.15*mRocket*grav);
                eng.Throttle = T/maxThrust;
                gim = max(min([-tauB(2), tauB(1)]/(T*hBase), gimbalMax), -gimbalMax);
                eng.Direction = [gim(1) gim(2) 1];
            end

            sim.step(dt, 1, 1); %2*(s == subSteps) - 1);
            t = t + dt;
        end

        log.t(end + 1) = t;
        log.p(:, end + 1) = rocket.Position;
        log.thr(end + 1) = eng.Throttle;
        log.gim(end + 1) = atan2d(norm(eng.Direction(1:2)), eng.Direction(3));
        viewer.displayText(sprintf("Waypoint %d / %d   alt %.2f m   throttle %3.0f %%   gimbal %.1f deg", ...
            wpIdx, size(wp, 1), rocket.Position(3), 100*eng.Throttle, log.gim(end)));
    end
    delete(sim);

    pEnd = rocket.Position;
    Mend = rocket.Transform;
    if isnan(tLand)
        fprintf("The hop timed out at waypoint %d.\n", wpIdx);
    else
        fprintf("Touchdown at t = %.1f s: vertical speed %.2f m/s, tilt %.1f deg.\n", ...
            tLand, vTouch, tilt);
        fprintf("Final position: %.2f m from the pad B center, standing tilt %.1f deg.\n", ...
            norm(pEnd(1:2) - [padBx 0]), acosd(Mend(3, 3)));
    end
    viewer.displayText(sprintf("Landed:  offset %.2f m,  touchdown %.2f m/s", ...
        norm(pEnd(1:2) - [padBx 0]), vTouch));

    % Flight profile plots
    clf(figure(2));
    subplot(2, 1, 1);
    plot(log.p(1, :), log.p(3, :), "LineWidth", 1.5); hold on
    plot([-1.5 1.5], [0.4 0.4], "Color", padA.Color, "LineWidth", 5);
    plot([padBx-1.5 padBx+1.5], [0.4 0.4], "Color", padB.Color, "LineWidth", 5);
    grid on; axis equal;
    xlabel("x [m]"); ylabel("z [m]");
    title("Hop trajectory (side view)");
    subplot(2, 1, 2);
    yyaxis left
    plot(log.t, 100*log.thr, "LineWidth", 1.5); ylabel("throttle [%]");
    yyaxis right
    plot(log.t, log.gim, "LineWidth", 1); ylabel("gimbal deflection [deg]");
    grid on; xlabel("time [s]");
    title("Throttle and gimbal");

end
