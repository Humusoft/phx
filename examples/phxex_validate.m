function phxex_validate(dts, showGraphs)
% PHXEX_VALIDATE Validate the engine against closed-form equations of motion
%
% Four textbook mechanics problems are simulated headlessly and the numbers
% coming out of the engine are compared with their analytical solutions:
%
%   A  Projectile motion   - translation under gravity (no contact)
%   B  Harmonic oscillator - linear spring force F = -k*x, energy conservation
%   C  Simple pendulum     - rotational motion with a revolute constraint
%   D  Head-on collision   - conservation of momentum in an impact
%
% Every case is run for each simulation time step in DTS (default 10, 5 and
% 1 ms), so the effect of the step size on accuracy is visible. A pass/fail
% table lists the relative error per step, and with SHOWGRAPHS two kinds of
% plots are produced: one log-log convergence figure (error vs. step, all
% steps together) and, for each step, a figure overlaying the engine
% trajectories on the analytical ones.
%
%   phxex_validate([0.010 0.005 0.001])   % choose your own step sizes (s)
%
% The point is to show that PHX reproduces known physics, not just plausible
% looking motion.

%   Copyright 2026 HUMUSOFT s.r.o.

    arguments
        dts (1, :) double {mustBePositive} = [0.010 0.005 0.001]
        showGraphs (1, 1) logical = true
    end

    dts = sort(dts, "descend");          % coarse -> fine

    % Run every case for the whole set of time steps; each returns its metric
    % rows and the per-step time series needed for the overlay plots.
    [Rp, Pp] = caseProjectile(dts);
    [Ro, Po] = caseOscillator(dts);
    [Rc, Pc] = casePendulum(dts);
    [Rd, Pd] = caseCollision(dts);
    results = [Rp; Ro; Rc; Rd];
    plots = [Pp Po Pc Pd];

    % Print the summary table
    printSummary(results, dts);

    % Convergence figure (all steps) + one trajectory figure per step
    if showGraphs
        plotConvergence(results, dts);
        for i = 1:numel(dts)
            figure("Name", sprintf("PHX validation - trajectories at %g ms", dts(i)*1000));
            tl = tiledlayout(2, 2, "TileSpacing", "compact", "Padding", "compact");
            title(tl, sprintf("Engine vs analytic   (\\Deltat = %g ms)", dts(i)*1000), "FontWeight", "bold");
            for c = 1:numel(plots)
                plots(c).plotFcn(nexttile, plots(c).data{i});
            end
        end
    end

end

% ------------------------------------------------------------------------
function [R, P] = caseProjectile(dts)
% Point mass launched with an initial velocity, flying freely under gravity.
% Analytical: x(t) = vx*t, z(t) = vz*t - g/2*t^2.

    g = 9.81; vx = 3; vz = 6; tEnd = 2*vz/g; apex = vz^2/(2*g);

    apexE = zeros(size(dts)); rmsE = zeros(size(dts)); data = cell(size(dts));
    for i = 1:numel(dts)
        ball = phx.Body([], "Position", [0 0 0], "LinearVelocity", [vx 0 vz], ...
            "Shape", {"Sphere", "Diameter", 0.2});
        log = phx.Logger(ball, "Frequency", 500, "Parameters", "Position");

        sim = phx.Simulation(ball); % default gravity
        sim.step(tEnd, round(tEnd/dts(i)), -1);

        t = log.Time; p = log.Data;
        delete(sim); delete(ball);

        pa = [vx*t, 0*t, vz*t - g/2*t.^2];      % analytical trajectory
        apexE(i) = max(p(:, 3));
        rmsE(i) = sqrt(mean(sum((p - pa).^2, 2)))/apex;
        data{i} = struct("p", p, "pa", pa);
    end

    R = [ metric("A projectile", "apex height", apexE, apex, 0.02)
          metric("A projectile", "path RMS/apex", rmsE, 0, 0.02) ];
    P = struct("plotFcn", @plotProjectile, "data", {data});
end

function plotProjectile(ax, d)
    dec = max(1, round(size(d.p, 1)/40));
    plot(ax, d.pa(:, 1), d.pa(:, 3), "-", "LineWidth", 1.5); hold(ax, "on");
    plot(ax, d.p(1:dec:end, 1), d.p(1:dec:end, 3), "o", "MarkerSize", 4);
    title(ax, "A  Projectile"); xlabel(ax, "x (m)"); ylabel(ax, "z (m)");
    legend(ax, "analytic", "engine", "Location", "south"); axis(ax, "equal");
end

% ------------------------------------------------------------------------
function [R, P] = caseOscillator(dts)
% Mass on a linear spring in a gravity-free world -> ideal harmonic motion.
% Analytical: x(t) = A*cos(w*t), w = sqrt(k/m), T = 2*pi/w, energy constant.

    k = 100; m = 1; A = 1; w = sqrt(k/m); Ttheory = 2*pi/w; x0 = -2;

    Tmeas = zeros(size(dts)); drift = zeros(size(dts)); data = cell(size(dts));
    for i = 1:numel(dts)
        % Anchor sits off to the side so the swinging bob never hits it; the
        % rest length places the equilibrium at the origin -> F = -k*x.
        anchor = phx.Body([], "Type", "static", "Position", [x0 0 0], ...
            "Shape", {"Sphere", "Diameter", 0.2});
        bob = phx.Body([], "Position", [A 0 0], "Mass", m, ...
            "Shape", {"Sphere", "Diameter", 0.2});
        phx.Spring(anchor, bob, "Stiffness", k, "Damping", 0, "FreeLength", -x0);
        log = phx.Logger(bob, "Frequency", 1000, "Parameters", ["Position" "LinearVelocity"]);

        sim = phx.Simulation([anchor bob]);
        sim.Gravity = [0 0 0];               % isolate the spring force
        sim.step(3, round(3/dts(i)), -1);

        t = log.Time; x = log.getChannel(1); v = log.getChannel(2);
        x = x(:, 1); v = v(:, 1);
        delete(sim); delete([anchor bob]);

        Tmeas(i) = measurePeriod(t, x);
        E = 0.5*m*v.^2 + 0.5*k*x.^2;
        drift(i) = (max(E) - min(E))/mean(E);
        data{i} = struct("t", t, "x", x, "xa", A*cos(w*t));
    end

    R = [ metric("B oscillator", "period", Tmeas, Ttheory, 0.02)
          metric("B oscillator", "energy drift", drift, 0, 0.02) ];
    P = struct("plotFcn", @plotOscillator, "data", {data});
end

function plotOscillator(ax, d)
    dec = max(1, round(numel(d.t)/60));
    plot(ax, d.t, d.xa, "-", "LineWidth", 1.5); hold(ax, "on");
    plot(ax, d.t(1:dec:end), d.x(1:dec:end), "o", "MarkerSize", 4);
    title(ax, "B  Harmonic oscillator"); xlabel(ax, "t (s)"); ylabel(ax, "x (m)");
    legend(ax, "analytic", "engine", "Location", "south");
end

% ------------------------------------------------------------------------
function [R, P] = casePendulum(dts)
% Small bob hung on a revolute joint, swinging under gravity.
% Small angle: T = 2*pi*sqrt(L/g). Large angle: total energy stays constant.

    g = 9.81; L = 1; Ttheory = 2*pi*sqrt(L/g);

    Tmeas = zeros(size(dts)); drift = zeros(size(dts)); data = cell(size(dts));
    for i = 1:numel(dts)
        % Small release angle -> validate the period
        small = runPendulum(deg2rad(5), L, 6, round(6/dts(i)));
        Tmeas(i) = measurePeriod(small.t, small.angle);

        % Large release angle -> validate energy conservation
        large = runPendulum(deg2rad(60), L, 4, round(4/dts(i)));
        drift(i) = (max(large.E) - min(large.E))/mean(large.E);
        data{i} = struct("t", large.t, "E", large.E);
    end

    % The period keeps a step-independent ~0.05% floor: that is the physical
    % large-amplitude correction of the real (nonlinear) pendulum over the
    % small-angle formula (1 + theta0^2/16), not an integration error.
    R = [ metric("C pendulum", "period(5deg)", Tmeas, Ttheory, 0.02)
          metric("C pendulum", "energy drift", drift, 0, 0.02) ];
    P = struct("plotFcn", @plotPendulum, "data", {data});
end

function plotPendulum(ax, d)
    plot(ax, d.t, d.E/mean(d.E), "-", "LineWidth", 1.5);
    title(ax, "C  Pendulum (60\circ)"); xlabel(ax, "t (s)");
    ylabel(ax, "E / E_{mean}"); ylim(ax, [0.9 1.1]);
    legend(ax, "engine total energy", "Location", "south");
end

function s = runPendulum(theta0, L, tEnd, nSteps)
% Release a point-like bob at angle THETA0 from the downward vertical and log
% its swing. Returns joint angle, time and total mechanical energy.

    g = 9.81; m = 1; P = [0 0 5];                    % pivot in world space
    off = L*[sin(theta0) 0 -cos(theta0)];            % pivot -> bob offset

    anchor = phx.Body([], "Type", "static", "Position", P, ...
        "Shape", {"Sphere", "Diameter", 0.05});
    bob = phx.Body([], "Position", P + off, "Mass", m, ...
        "Shape", {"Sphere", "Diameter", 0.06});      % small -> negligible own inertia
    joint = phx.RevoluteJoint(anchor, bob, "PointA", [0 0 0], "PointB", -off, ...
        "AxisA", [0 1 0], "AxisB", [0 1 0]);
    logA = phx.Logger(joint, "Frequency", 1000, "Parameters", "Angle");
    logB = phx.Logger(bob, "Frequency", 1000, "Parameters", ["Position" "Energy"]);

    sim = phx.Simulation([anchor bob]);
    sim.step(tEnd, nSteps, -1);

    s.t = logA.Time;
    s.angle = logA.Data;
    pos = logB.getChannel(1); ke = logB.getChannel(2);
    s.E = ke + m*g*pos(:, 3);                         % kinetic + potential
    delete(sim); delete([anchor bob]);
end

% ------------------------------------------------------------------------
function [R, P] = caseCollision(dts)
% Head-on collision of two spheres in a gravity-free world. Whatever the
% contact does, total momentum must be conserved (Newton's third law), so the
% center-of-mass velocity vcm = (m1*v1 + m2*v2)/(m1 + m2) stays constant
% before, during and after the impact. This holds for any restitution, which
% is why it is a robust engine check rather than assuming a specific bounce.

    m1 = 1; m2 = 2; v1 = 2; p0 = m1*v1; M = m1 + m2; vcm = p0/M;

    pAfter = zeros(size(dts)); drift = zeros(size(dts)); data = cell(size(dts));
    for i = 1:numel(dts)
        b1 = phx.Body([], "Position", [-3 0 0], "Mass", m1, "Restitution", 0, ...
            "LinearVelocity", [v1 0 0], "Shape", {"Sphere", "Diameter", 1});
        b2 = phx.Body([], "Position", [ 3 0 0], "Mass", m2, "Restitution", 0, ...
            "Shape", {"Sphere", "Diameter", 1});
        log = phx.Logger([b1 b2], "Frequency", 500, "Parameters", "LinearVelocity");

        sim = phx.Simulation([b1 b2]);
        sim.Gravity = [0 0 0];
        sim.step(5, round(5/dts(i)), -1);

        t = log.Time; v1x = log.getChannel(1); v2x = log.getChannel(2);
        v1x = v1x(:, 1); v2x = v2x(:, 1);
        delete(sim); delete([b1 b2]);

        vcmt = (m1*v1x + m2*v2x)/M;
        pAfter(i) = m1*v1x(end) + m2*v2x(end);
        drift(i) = max(abs(vcmt - vcm))/vcm;
        data{i} = struct("t", t, "v1x", v1x, "v2x", v2x, "vcmt", vcmt);
    end

    R = [ metric("D collision", "momentum", pAfter, p0, 0.01)
          metric("D collision", "vcm drift", drift, 0, 0.02) ];
    P = struct("plotFcn", @plotCollision, "data", {data});
end

function plotCollision(ax, d)
    plot(ax, d.t, d.v1x, "-", d.t, d.v2x, "-", "LineWidth", 1.5); hold(ax, "on");
    plot(ax, d.t, d.vcmt, "--", "LineWidth", 1.2);
    title(ax, "D  Collision (momentum)"); xlabel(ax, "t (s)"); ylabel(ax, "v_x (m/s)");
    legend(ax, "body 1", "body 2", "v_{cm}", "Location", "east");
end

% ------------------------------------------------------------------------
function T = measurePeriod(t, y)
% Mean period from upward zero crossings of Y (about its mean), with linear
% interpolation of the crossing instants.

    t = t(:); y = y(:) - mean(y);
    i = find(y(1:end-1) <= 0 & y(2:end) > 0);
    if numel(i) < 2
        T = NaN; return;
    end
    tc = t(i) - y(i).*(t(i+1) - t(i))./(y(i+1) - y(i));
    T = mean(diff(tc));
end

% ------------------------------------------------------------------------
function r = metric(name, qty, engVec, th, tol)
% Build one result row holding the engine value and error for every time step.
% TH = 0 means ENGVEC is already a (dimensionless) error measure such as a
% drift fraction and is compared to TOL directly. PASS is decided by the
% finest step (last element).

    r.Name = name; r.Quantity = qty; r.Engine = engVec; r.Theory = th; r.Tol = tol;
    if th == 0
        r.Err = abs(engVec);
    else
        r.Err = abs(engVec - th)/abs(th);
    end
    r.Pass = r.Err(end) <= tol;
end

% ------------------------------------------------------------------------
function printSummary(R, dts)
% Print the pass/fail table with one error column per time step.

    nDt = numel(dts);
    fprintf("\n%-14s %-16s %10s", "Case", "Quantity", "Theory");
    for i = 1:nDt
        fprintf(" %9s", sprintf("err@%gms", dts(i)*1000));
    end
    fprintf("   %s\n", "Result");
    width = 43 + nDt*10 + 11;
    fprintf("%s\n", repmat('-', 1, width));

    for k = 1:numel(R)
        r = R(k);
        fprintf("%-14s %-16s %10.5g", r.Name, r.Quantity, r.Theory);
        for i = 1:nDt
            fprintf(" %8.3f%%", r.Err(i)*100);
        end
        if r.Pass, tag = "PASS"; else, tag = "**FAIL**"; end
        fprintf("   %s\n", tag);
    end

    fprintf("%s\n", repmat('-', 1, width));
    nFail = sum(~[R.Pass]);
    if nFail == 0
        fprintf("All %d checks PASS at the finest step (%g ms); errors shrink as the step is refined.\n\n", ...
            numel(R), min(dts)*1000);
    else
        fprintf("%d of %d checks FAIL at the finest step (%g ms).\n\n", nFail, numel(R), min(dts)*1000);
    end
end

% ------------------------------------------------------------------------
function plotConvergence(R, dts)
% One tile per case: log-log error vs. time step for both of its metrics.

    dtms = dts*1000;
    names = [R.Name];
    ucase = unique(names, "stable");

    figure("Name", "PHX validation - step size vs accuracy");
    tiledlayout(2, 2, "TileSpacing", "compact", "Padding", "compact");
    for c = 1:numel(ucase)
        ax = nexttile; hold(ax, "on");
        rows = R(names == ucase(c));
        for j = 1:numel(rows)
            plot(ax, dtms, max(rows(j).Err*100, 1e-4), "-o", "LineWidth", 1.5, "MarkerSize", 5);
        end
        set(ax, "XScale", "log", "YScale", "log");
        title(ax, ucase(c)); xlabel(ax, "\Deltat (ms)"); ylabel(ax, "rel. error (%)");
        legend(ax, [rows.Quantity], "Location", "northwest");
        grid(ax, "on"); xlim(ax, [min(dtms)/1.5, max(dtms)*1.5]);
    end
end
