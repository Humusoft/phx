function phxex_optimize(targetX, targetH)
% PHXEX_OPTIMIZE Optimizing a projectile launch with PHX as the model
%
% This demo shows PHX used as a black-box physics model inside a MATLAB
% optimization loop. A projectile is launched with some speed and elevation
% angle, flies under gravity (with quadratic aerodynamic drag from
% phx.Resistance) and bounces off the terrain. We want it to land on a target. The launch speed
% and angle are tuned automatically with fminsearch (base MATLAB, no toolbox
% required) so that the landing point hits the target.
%
% Each optimization iteration runs a full PHX simulation WITHOUT graphics
% (phx.Body([], ...) and a head-less phx.Simulation), which is fast and
% deterministic - exactly the workflow where coupling a physics engine to
% MATLAB's solvers pays off and where a pure game engine would not help.
%
% After the optimum is found, the winning launch is replayed once with the
% viewer so the trajectory and the hit can be seen.
%
% Input Arguments:
%     targetX - horizontal distance to the target
%     targetH - height of the target above ground
%
% Example:
%     phxex_optimize             % hit a target 30 m away, 3 m up
%     phxex_optimize(45, 5)      % farther, higher target

%   Copyright 2026 HUMUSOFT s.r.o.

    arguments
        targetX (1, 1) double = 30
        targetH (1, 1) double = 3
    end

    fprintf("Optimizing launch to hit target at x = %.1f m, z = %.1f m ...\n", ...
        targetX, targetH);

    % --- optimization: tune [speed, angle] to minimize the miss distance ---
    % Cost = distance between the projectile and the target at closest
    % approach during a head-less PHX simulation.
    cost = @(p) launchMiss(p(1), p(2), targetX, targetH, false);

    % Initial guess: a ballistic estimate ignoring drag and bounce
    ang0 = pi/4;
    v0 = sqrt(targetX*9.81 / sin(2*ang0));
    p0 = [v0, ang0];

    opts = optimset("Display", "iter", "TolX", 1e-2, "TolFun", 1e-2);
    pOpt = fminsearch(cost, p0, opts);

    vOpt = pOpt(1);
    angOpt = pOpt(2);
    finalMiss = launchMiss(vOpt, angOpt, targetX, targetH, false);
    fprintf("\nOptimum: speed = %.2f m/s, angle = %.1f deg  ->  miss = %.3f m\n", ...
        vOpt, angOpt*180/pi, finalMiss);

    % --- replay the optimized launch with visualization ---
    fprintf("Replaying optimized launch...\n");
    launchMiss(vOpt, angOpt, targetX, targetH, true);

end

% ------------------------------------------------------------------------
function miss = launchMiss(speed, angle, targetX, targetH, showView)
% Runs one PHX projectile launch and returns the closest distance the
% projectile gets to the target. With showView=false the run is head-less
% (no graphics) so it is fast enough to call repeatedly from the optimizer.

    % Guard against the optimizer trying non-physical values
    if speed <= 0 || angle <= 0 || angle >= pi/2
        miss = 1e3;
        return;
    end

    rng(0);     % keep every evaluation deterministic

    if showView
        [~, ax] = phx.extra.Viewer("clear", "DefaultCameraTarget", [targetX/2 0 3], ...
            "DefaultCameraPosition", [targetX/2 -targetX 0.6*targetX]);
    else
        ax = [];    % head-less: no axes -> no rendering
    end

    % Ground / terrain the projectile can bounce on
    ground = phx.Body(ax, "Type", "static", "Position", [targetX/2 0 -0.5], ...
        "Shape", {"Box", "Size", [4*targetX 20 1], "Color", [0.8 0.85 0.7]}, ...
        "Friction", [0.6 0 0]);

    % Target marker
    targetPos = [targetX 0 targetH/2];
    target = phx.Body(ax, "Position", targetPos, "Shape", {"Box", "Size", [1 1 targetH], "Color", [1 0.2 0.2]});

    % Projectile launched from the origin
    ball = phx.Body(ax, "Position", [0 0 0.5], ...
        "Shape", {"Sphere", "Radius", 0.25, "Color", [1 1 1]}, ...
        "Mass", 300, "Friction", [0.6 0 0], ...
        "LinearVelocity", [speed*cos(angle) 0 speed*sin(angle)]);

    % Aerodynamic drag so the model is more than a textbook parabola.
    % Ffactors define F = f(1)*v^0 + f(2)*v^1 + f(3)*v^2 ; we use a
    % quadratic (~v^2) drag, typical for a body moving through air.
    phx.Resistance(ball, "VelocityFactors", [0 0 0.02]);

    if showView
        phx.Trace(ball, "TracePoints", 300, "Color", [1 1 1]);
    end

    % Head-less simulation, tracking closest approach to the target
    bodies = [ground ball target];
    sim = phx.Simulation(bodies);

    miss = inf;
    dt = 0.01;
    for k = 1:400
        if showView
            sim.step(dt, 1, 1);
        else
            sim.step(dt, 1);            % no redraw
        end
        d = norm(ball.Position - targetPos);
        miss = min(miss, d);
        % Stop once the projectile is clearly past and below the target
        if ball.Position(1) > targetX + 2 && ball.Position(3) < targetH
            break;
        end
    end
    delete(sim);

    if showView
        fprintf("Replay closest approach: %.3f m\n", miss);
    end

end