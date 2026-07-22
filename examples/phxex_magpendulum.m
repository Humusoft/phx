function phxex_magpendulum(nGrid, tMaxRun)
% PHXEX_MAGPENDULUM Chaotic magnetic pendulum and its fractal basin map
%
% A pendulum rod with a magnetic tip (phx.Dipole - the attracting pole
% sits at the lower physical end of the rod and follows its orientation)
% swings on a phx.SphericalJoint above three attracting magnets. Which
% magnet finally captures the tip depends extremely sensitively on the
% initial position - the famous fractal-basin chaotic system.
%
% The demo runs in two parts:
%   1. Two pendulums whose starting points differ by one millimeter swing
%      side by side (each in its own dipole group, so they do not feel
%      each other). Their traces split after a few swings and they may end
%      up at different magnets; the distance between the two tips grows
%      exponentially - the butterfly effect, quantified in a semilog plot.
%   2. The same scene is then reused as a fast solver: the pendulum is
%      teleported to a grid of starting positions and stepped without
%      rendering until a magnet captures it. The resulting basin map,
%      colored by the capturing magnet, reveals the fractal boundaries.
%      The deterministic engine guarantees the map is reproducible.
%
% Input Arguments:
%     nGrid   - basin map resolution, nGrid x nGrid runs (default 21)
%     tMaxRun - time limit of a single basin map run (default 20)
%
% Example:
%     phxex_magpendulum         % default resolution
%     phxex_magpendulum(41)     % finer fractal, ~4x the runtime

%   Copyright 2026 HUMUSOFT s.r.o.

    arguments
        nGrid (1, 1) double {mustBeInteger, mustBeGreaterThanOrEqual(nGrid, 5)} = 21
        tMaxRun (1, 1) double {mustBePositive} = 20
    end

    % Pendulum and magnet layout
    L = 4.2;               % pendulum rod length
    zA = 5;                % anchor height
    Rmag = 1.0;            % magnet circle radius
    magAng = [90 210 330]; % magnet positions on the circle (deg)
    magClr = [0.9 0.25 0.25; 0.25 0.75 0.3; 0.3 0.45 1];
    qRod = 1;              % tip pole charge
    qMag = 16;             % magnet pole charge
    cDamp = 2.5;           % air drag factor (sets the settling time)
    dt = 0.01;
    mapRange = 2.8;        % basin map half-extent

    % Figure setup
    figure(1);
    [viewer, ax] = phx.extra.Viewer("clear", "DefaultCameraTarget", [0 0 2], ...
        "DefaultCameraPosition", [9 -12 7]);

    % Base plate and the three magnets
    phx.Body(ax, "Type", "static", "Position", [0 0 -0.1], ...
        "Shape", {"Box", "Size", [9 9 0.2], "Color", [1 1 1]});
    for k = 1:3
        magPos(k, :) = Rmag*[cosd(magAng(k)) sind(magAng(k))]; %#ok<AGROW> 3 rows
        magnets(k) = phx.Body(ax, "Type", "static", "Position", [magPos(k, :) 0.2], ...
            "Shape", {"Cylinder", "Diameter", 1, "Height", 0.4, "Color", magClr(k, :)}); %#ok<AGROW> 3 magnets
    end

    % Anchor point of the pendulums
    anchor = phx.Body(ax, "Type", "static", "Position", [0 0 zA], ...
        "Shape", {"Sphere", "Diameter", 0.25, "Division", 2, "Color", [0.3 0.3 0.3]}, ...
        "Collisions", false);

    % Part 1 - twin pendulums starting 1 mm apart ----------------------------
    tip0 = [1.3 0.55];
    rod1 = makeRod(tip0, [0.95 0.75 0.2]);
    [rod2, dip2, j2, tr2] = makeRod(tip0 + [1e-3 0], [0.2 0.85 0.95]);

    viewer.displayText("Twin pendulums...");

    % Sleeping must stay off: a slowly swinging or settled rod is near rest
    % and would otherwise freeze and ignore gravity and the dipole field
    sim = phx.Simulation(ax);
    tLive = 12;
    nFrames = round(tLive/(5*dt));
    dist = zeros(1, nFrames);
    tVec = (1:nFrames)*5*dt;
    for f = 1:nFrames
        sim.step(5*dt, 5, 5);
        pause(0);
        dist(f) = norm(tipOf(rod1) - tipOf(rod2));
        viewer.displayText(sprintf("Twin pendulums:  t = %.1f s   tip distance = %.4f", ...
            tVec(f), dist(f)));
    end
    [m1, d1] = nearestMagnet(rod1);
    [m2, d2] = nearestMagnet(rod2);
    fprintf("Twin A ended near magnet %d (%.2f away), twin B near magnet %d (%.2f away).\n", ...
        m1, d1, m2, d2);

    % Part 2 - basin of attraction map ---------------------------------------
    % Remove the second pendulum: the engine constraint disappears with the
    % simulation rebuild, then the orphaned objects can be deleted safely
    delete(sim);
    delete(dip2); delete(j2); delete(tr2); delete(rod2);
    sim = phx.Simulation(ax);

    xs = linspace(-mapRange, mapRange, nGrid);
    basin = zeros(nGrid);          % 0 = not settled, 1..3 = magnet, 4 = center
    total = nGrid^2;
    done = 0;
    tStart = tic;
    for iy = 1:nGrid
        for ix = 1:nGrid
            done = done + 1;
            if xs(ix)^2 + xs(iy)^2 > (0.85*L)^2
                continue
            end

            % Teleport the pendulum to the new start and run until capture;
            % stepping without rendering makes the scene a pure solver.
            % Capture = slow tip lingering close above a magnet (or resting
            % at the center), confirmed over two consecutive checks
            setRod(rod1, [xs(ix) xs(iy)]);
            settled = 0;
            for t = 0.25:0.25:tMaxRun
                sim.step(0.25, 25, -1);
                [k, dmin] = nearestMagnet(rod1);
                tip = tipOf(rod1);
                if norm(tip(1:2)) < 0.3
                    k = 4;             % the central rest is a 4th attractor
                    dmin = 0;
                end
                if t > 1 && dmin < 0.5 && norm(rod1.LinearVelocity) < 0.15
                    settled = settled + 1;
                    if settled == 2
                        basin(iy, ix) = k;
                        break
                    end
                else
                    settled = 0;
                end
            end
        end
        elapsed = toc(tStart);
        viewer.displayText(sprintf("Basin map:  %d / %d runs   (~%.0f s left)", ...
            done, total, elapsed/done*(total - done)));
        pause(0);
    end
    delete(sim);
    viewer.displayText("Done.");

    shares = arrayfun(@(k) nnz(basin == k), 1:4)/nnz(basin);
    fprintf("Basin shares: %.0f %% / %.0f %% / %.0f %% magnets, %.0f %% center (%d settled, %.0f s).\n", ...
        100*shares, nnz(basin), toc(tStart));

    % Plot the butterfly effect and the fractal basin map
    figure(2);
    semilogy(tVec, max(dist, 1e-6), "LineWidth", 1.5);
    grid on; xlabel("time [s]"); ylabel("distance of the twin tips");
    title("Butterfly effect: twins started 1 mm apart");

    figure(3);
    imagesc(xs, xs, basin);
    axis equal tight xy;
    colormap([0.12 0.12 0.12; magClr; 0.75 0.75 0.75]);   % black = not settled,
    clim([-0.5 4.5]);                                     % gray = central rest
    hold on
    plot(magPos(:, 1), magPos(:, 2), "wo", "MarkerSize", 8, "LineWidth", 1.5);
    xlabel("start x"); ylabel("start y");
    title(sprintf("Which magnet captures the pendulum (%d x %d starts)", nGrid, nGrid));

    % ------------------------------------------------------------------------
    function [rod, dip, j, tr] = makeRod(tipXY, clr)
    % Creates a pendulum rod with a magnetic tip: a thin cylinder hanging
    % from the anchor, its dipole pole placed at the lower physical end
        rod = phx.Body(ax, "Shape", {"Cylinder", "Diameter", 0.15, "Height", L, ...
            "Segments", 10, "Density", 50, "Color", clr}, "Collisions", false);
        setRod(rod, tipXY);
        % A long thin rod has a ~500:1 inertia anisotropy that makes the
        % constraint solver spin it up about its own axis; an isotropic
        % inertia removes this (axial spin is irrelevant for a pendulum)
        rod.Inertia = rod.Inertia(1)*[1 1 1];
        % The joint point [0 0 L/2] is the upper end of the rod, pinned to
        % the anchor center
        j = phx.SphericalJoint(anchor, rod, "PointB", [0 0 L/2]);
        % One dipole group per rod: the rod interacts with the magnets but
        % not with the other rod. The rod's +pole is at its tip; each magnet
        % carries the opposite charge with its +pole on top, so the magnet's
        % near (top) pole attracts the descending tip
        dip = phx.Dipole([rod magnets], "Charge", [qRod -qMag -qMag -qMag]', ...
            "Attractivity", -1, "Axis", [0 0 -L/2; repmat([0 0 0.5], 3, 1)], ...
            "VectorFieldSize", [0 0 0], "Color", [0.6 0.6 0.6]);
        phx.Resistance(rod, "VelocityFactors", [0 cDamp]);
        tr = phx.Trace(rod, "Point", [0 0 -L/2], "TracePoints", 500, ...
            "Overlay", true, "Color", clr);
    end

    function setRod(rod, tipXY)
    % Teleports the rod so that its tip is at the given horizontal position
    % (on the sphere of radius L around the anchor) with zero velocity
        zTip = zA - sqrt(L^2 - tipXY(1)^2 - tipXY(2)^2);
        dir = ([tipXY zTip] - [0 0 zA])/L;     % unit vector anchor -> tip
        up = -dir;                             % rod local +Z points to anchor
        ref = [1 0 0];
        if abs(dot(ref, up)) > 0.9
            ref = [0 1 0];
        end
        u = cross(ref, up); u = u/norm(u);
        v = cross(up, u);
        rod.Position = [0 0 zA] + dir*L/2;
        rod.Orientation = [u' v' up'];
        rod.LinearVelocity = [0 0 0];
        rod.AngularVelocity = [0 0 0];
    end

    function tip = tipOf(rod)
        tip = phx.internal.transformPoint(rod.Transform, [0 0 -L/2]);
    end

    function [k, dmin] = nearestMagnet(rod)
        tip = tipOf(rod);
        [dmin, k] = min(vecnorm(magPos - tip(1:2), 2, 2));
    end

end
