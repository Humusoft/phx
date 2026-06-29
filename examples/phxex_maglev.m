function phxex_maglev(kp, kd)
% PHXEX_MAGLEV Active magnetic levitation with a PD controller
%
% An electromagnet (a static body with a controllable phx.Monopole charge)
% hangs above the floor and attracts a steel ball below it. By Earnshaw's
% theorem such levitation is inherently unstable: the attraction grows as
% the ball approaches, so a constant coil current cannot hold it - the
% demo first shows exactly that. With the coil current perfectly matched
% to the ball weight and the ball displaced by one centimeter, the ball
% runs away within a second.
%
% Then a PD controller is switched on: each simulation step it reads the
% air gap and its rate from a phx.Measure object and modulates the coil
% charge (the Charge property is writable while the simulation runs).
% The closed loop holds the ball in mid-air, follows setpoint changes of
% the gap and recovers from force disturbances kicking the ball. The
% controller gains are derived from the linearized model, so they adapt
% to the chosen geometry and mass automatically.
%
% Input Arguments:
%     kp - proportional gain multiplier (default 1; 0.3 is below the
%          stability limit and the levitation fails)
%     kd - derivative gain multiplier (default 1; small values make the
%          hover oscillatory)
%
% Example:
%     phxex_maglev          % stable levitation
%     phxex_maglev(0.3)     % too soft -> Earnshaw wins even with control

%   Copyright 2026 HUMUSOFT s.r.o.

    arguments
        kp (1, 1) double {mustBeNonnegative} = 1
        kd (1, 1) double {mustBeNonnegative} = 1
    end

    % Geometry and the levitated ball
    zCoil = 4;             % coil center height
    g0 = 1.2;              % nominal air gap (coil center to ball center)
    d = 0.4;               % ball diameter
    rho = 300;             % ball density -> mass ~10 kg
    qBall = 1;             % ball charge (the coil charge is the control input)

    mBall = rho*4/3*pi*(d/2)^3;
    u0 = mBall*9.81*g0^2/qBall;    % feedforward charge holding the weight at g0

    % PD gains from the linearized model: the instability stiffness is
    % 2*m*g/g0, the proportional term must outweigh it (Kp > 2*u0/g0)
    Kp = kp*6*u0/g0;
    Kd = kd*0.16*6*u0/g0;
    uMax = 4*u0;                   % coil saturation

    % Figure setup
    figure(1);
    [viewer, ax] = phx.extra.Viewer("clear", "DefaultCameraTarget", [0 0 2.2], ...
        "DefaultCameraPosition", [5.6 -7.2 3.6]);

    % Static floor and the stand holding the electromagnet
    phx.Body(ax, "Type", "static", "Position", [0 0 -0.5], ...
        "Shape", {"Box", "Size", [8 8 1], "Color", [1 1 1]});
    phx.Body(ax, "Type", "static", "Position", [-1.6 0 zCoil/2 + 0.25], ...
        "Shape", {"Box", "Size", [0.3 0.3 zCoil + 0.5], "Color", [0.55 0.55 0.6]});
    phx.Body(ax, "Type", "static", "Position", [-0.8 0 zCoil + 0.38], ...
        "Shape", {"Box", "Size", [1.9 0.3 0.25], "Color", [0.55 0.55 0.6]});

    % The electromagnet: a coil with a core, the body carries the charge
    coil = phx.Body(ax, "Type", "static", "Position", [0 0 zCoil], ...
        "Shape", {"Cylinder", "Diameter", 1.0, "Height", 0.5, ...
        "Color", [0.75 0.55 0.3], "Material", "metal"});
    phx.Body(ax, "Type", "static", "Position", [0 0 zCoil - 0.05], ...
        "Shape", {"Cylinder", "Diameter", 0.35, "Height", 0.7, ...
        "Color", [0.35 0.35 0.4], "Material", "metal"});

    % The levitated ball
    ball = phx.Body(ax, "Position", [0 0 zCoil - g0], ...
        "Shape", {"Sphere", "Diameter", d, "Density", rho, ...
        "Color", [0.4 0.55 0.9], "Material", "metal"});

    % Controllable magnetic interaction + field visualization in XZ plane
    mono = phx.Monopole([coil ball], "Charge", [u0 -qBall]', "Attractivity", -1, ...
        "VectorFieldCenter", [0 0 zCoil - g0/2], "VectorFieldSize", [3 0 2.5], ...
        "VectorFieldStep", 0.5, "VectorLength", 0.5, "VectorSegments", 4, ...
        "Color", [0.8 0.8 0.8]);

    % Gap sensor
    gap = phx.Measure(coil, ball, "Overlay", true);

    % On-screen readout
    viewer.displayText("Earnshaw...");

    % Sleeping must stay disabled: a hovering ball is almost at rest and
    % a deactivated body would ignore the magnetic force and disturbances
    sim = phx.Simulation(ax, "EngineSettings", ...
        phx.engine.BulletSettings("AutoActivated", false));
    dt = 0.005;
    subSteps = 10;
    log = struct("t", [], "gap", [], "ref", [], "u", []);

    % Phase A - Earnshaw's theorem: constant coil charge exactly balancing
    % the weight, ball displaced 1 cm up -> the equilibrium runs away
    ball.Position = [0 0 zCoil - g0 + 0.01];
    mono.Charge = [u0 -qBall]';
    t = -3;
    tCrash = NaN;
    while t < 0
        for s = 1:subSteps
            sim.step(dt, 1, 1);
            t = t + dt;
            log.t(end + 1) = t;
            log.gap(end + 1) = gap.Distance;
            log.ref(end + 1) = NaN;
            log.u(end + 1) = u0;
        end
        if isnan(tCrash) && abs(gap.Distance - g0) > 0.4
            tCrash = t + 3;
            viewer.displayText(sprintf("Earnshaw: unstable after %.2f s", tCrash));
        end
    end
    if gap.Distance < g0
        fprintf("Earnshaw phase: the ball crashed into the coil after %.2f s.\n", tCrash);
    else
        fprintf("Earnshaw phase: the ball fell down after %.2f s.\n", tCrash);
    end

    % Phase B - PD control of the coil charge; setpoint steps and two
    % force disturbances test the closed loop
    ball.Position = [0 0 zCoil - g0 - 0.3];   % release below the setpoint
    ball.LinearVelocity = [0 0 0];
    ball.AngularVelocity = [0 0 0];

    tEnd = 12;
    kicks = [6, 0 0 -250; 10, 180 0 0];       % [time, force vector]
    while t < tEnd
        for s = 1:subSteps
            % Gap setpoint profile
            if t < 4
                gRef = g0;
            elseif t < 8
                gRef = g0 - 0.3;
            else
                gRef = g0 + 0.2;
            end

            % PD law on the coil charge, with saturation; the feedforward
            % term is scheduled with the setpoint (the charge needed to
            % hold the weight grows with the gap squared), so the PD part
            % only fights deviations and there is no steady-state droop
            e = gap.Distance - gRef;
            eRate = dot(gap.Position, gap.Velocity)/max(gap.Distance, 0.01);
            uFF = u0*(gRef/g0)^2;
            u = min(max(uFF + Kp*e + Kd*eRate, 0), uMax);
            mono.Charge = [u -qBall]';

            % Force disturbances
            for k = 1:size(kicks, 1)
                if t >= kicks(k, 1) && t < kicks(k, 1) + 0.05
                    ball.applyForce(kicks(k, 2:4), [0 0 0], false);
                end
            end

            sim.step(dt, 1, 1);
            t = t + dt;
            log.t(end + 1) = t;
            log.gap(end + 1) = gap.Distance;
            log.ref(end + 1) = gRef;
            log.u(end + 1) = u;
        end
        viewer.displayText(sprintf("PD control:  gap %.3f m (setpoint %.2f)   coil %4.0f %%", ...
            gap.Distance, gRef, 100*u/u0));
    end
    delete(sim);

    % Report the tracking quality in the steady part of each segment
    seg = [2 4; 6.5 8; 10.5 12];
    segRef = [g0, g0 - 0.3, g0 + 0.2];
    for k = 1:3
        id = log.t > seg(k, 1) & log.t < seg(k, 2);
        fprintf("Setpoint %.2f m: mean gap %.3f m, RMS error %.4f m.\n", ...
            segRef(k), mean(log.gap(id)), rms(log.gap(id) - segRef(k)));
    end

    % Plot the gap tracking and the control effort
    figure(2);
    subplot(2, 1, 1);
    plot(log.t, log.gap, "LineWidth", 1.5); hold on
    plot(log.t, log.ref, "--", "LineWidth", 1.5);
    xline(0, ":", "controller ON");
    xline(kicks(:, 1), ":", "kick");
    grid on; xlabel("time [s]"); ylabel("air gap [m]");
    legend("measured gap", "setpoint");
    title(sprintf("Magnetic levitation (Kp x%.2g, Kd x%.2g)", kp, kd));
    subplot(2, 1, 2);
    plot(log.t, 100*log.u/u0, "LineWidth", 1.5); hold on
    yline(100*uMax/u0, "--", "saturation");
    xline(0, ":");
    grid on; xlabel("time [s]"); ylabel("coil charge [% of nominal]");

end
