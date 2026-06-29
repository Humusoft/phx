function phxex_crystal(nBalls, annealTime, seed)
% PHXEX_CRYSTAL Ionic self-assembly of charged particles by annealing
%
% An equal number of positively (red) and negatively (blue) charged balls
% is dropped into an arena. All of them interact through a single
% phx.Monopole group: opposite charges attract, like charges repel. On
% top of that, every ball is kicked by a random "thermal" force whose
% amplitude decreases linearly to zero - simulated annealing. While the
% temperature is high the balls wander and explore; as it cools down they
% lock into low-energy aggregates with alternating charges, like a 2D
% ionic crystal. Nobody programs the structures - they emerge purely from
% the interplay of the force field, collisions and friction.
%
% The degree of order is measured during the whole run: a bond is a pair
% of touching balls, and the ionic order is the fraction of bonds that
% connect opposite charges (100 % = perfect salt crystal, ~50 % = random
% packing). The final plot shows the order parameter rising as the system
% cools, together with the histogram of the assembled cluster sizes.
%
% Input Arguments:
%     nBalls     - number of balls, rounded up to an even count (default 40)
%     annealTime - duration of the cooling phase in seconds (default 10)
%     seed       - random seed (default 0)
%
% Example:
%     phxex_crystal            % default run
%     phxex_crystal(40, 1)     % fast quench -> more fragments and defects

%   Copyright 2026 HUMUSOFT s.r.o.

    arguments
        nBalls (1, 1) double {mustBeInteger, mustBePositive} = 40
        annealTime (1, 1) double {mustBePositive} = 10
        seed (1, 1) double = 0
    end

    rng(seed); % Random seed for reproducible spawn and thermal noise

    % Particles and forces
    d = 0.5;               % ball diameter
    rho = 100;             % ball density -> mass ~6.5 kg
    q = 8;                 % charge magnitude (contact force ~4x the weight)
    fThermal = 400;        % initial thermal force amplitude (N); above the
                           % contact binding force, so the hot phase melts
                           % any clusters formed during the drop
    mu = [0.05 0.01 0];    % low friction lets the clusters rearrange
    arena = 9;             % arena size

    nBalls = 2*ceil(nBalls/2);

    % Figure setup
    figure(1);
    [viewer, ax] = phx.extra.Viewer("clear", "DefaultCameraTarget", [0 0 0.5], ...
        "DefaultCameraPosition", [0 -12 9]);

    % Static arena: floor and low walls
    phx.Body(ax, "Type", "static", "Position", [0 0 -0.5], ...
        "Shape", {"Box", "Size", [arena + 2, arena + 2, 1], "Color", [1 1 1]}, ...
        "Friction", mu);
    for k = 1:4
        a = k*pi/2;
        phx.Body(ax, "Type", "static", ...
            "Position", [cos(a), sin(a), 0]*arena/2 + [0 0 0.6], ...
            "EulerAngles", [0 0 a], ...
            "Shape", {"Box", "Size", [0.2, arena + 0.2, 1.2], "Color", [0.8 0.8 0.85]}, ...
            "Friction", mu);
    end

    % Charged balls: half positive (red), half negative (blue), spawned on
    % a jittered grid above the arena
    chg = q*[ones(1, nBalls/2), -ones(1, nBalls/2)];
    chg = chg(randperm(nBalls));
    cols = ceil(sqrt(nBalls));
    pitch = (arena - 2)/(cols - 1);
    for i = 1:nBalls
        gx = mod(i - 1, cols); gy = floor((i - 1)/cols);
        p = [gx*pitch - (arena - 2)/2, gy*pitch - (arena - 2)/2] + (rand(1, 2) - 0.5)*0.4;
        if chg(i) > 0
            clr = [1 0.4 0.35];
        else
            clr = [0.4 0.55 1];
        end
        balls(i) = phx.Body(ax, "Position", [p, 1.5 + rand], ...
            "Shape", {"Sphere", "Diameter", d, "Division", 2, "Density", rho, ...
            "Color", clr}, "Friction", mu); %#ok<AGROW> small fixed size
    end

    % One interaction group for all balls: opposite charges attract
    phx.Monopole(balls, "Charge", chg', "Attractivity", -1, ...
        "VectorFieldSize", [0 0 0], "Visible", false);

    % On-screen readout
    viewer.displayText("Dropping...");

    % Sleeping must stay disabled, otherwise slow-moving balls would stop
    % responding to the field and thermal forces
    sim = phx.Simulation(ax, "EngineSettings", ...
        phx.engine.BulletSettings("AutoActivated", false));
    dt = 0.005;
    subSteps = 10;

    % Phase 1 - drop and a short hot dwell
    sim.step(1.5, 300, 30);

    % Phase 2 - annealing: random thermal kicks with a linearly decreasing
    % amplitude; the order parameter is recorded as the system cools
    hist = struct("t", [], "T", [], "order", [], "bonds", []);
    lastOrder = 0;
    frame = 0;
    t = 0;
    while t < annealTime
        T = 1 - t/annealTime;                 % relative temperature
        for i = 1:nBalls
            dir = randn(1, 3).*[1 1 0.3];
            balls(i).applyForce(fThermal*T*dir/norm(dir), [0 0 0], false);
        end
        sim.step(dt*subSteps, subSteps, subSteps);
        t = t + dt*subSteps;
        frame = frame + 1;
        pause(0);

        if mod(frame, 10) == 0                % sample the order every 0.5 s
            [lastOrder, nBonds] = bondOrder(balls, chg, d);
            hist.t(end + 1) = t;
            hist.T(end + 1) = T;
            hist.order(end + 1) = lastOrder;
            hist.bonds(end + 1) = nBonds;
        end
        viewer.displayText(sprintf("Annealing:  temperature %3.0f %%   ionic order %3.0f %%", ...
            100*T, 100*lastOrder));
    end

    % Phase 3 - cold settle
    settleTimeout = 5;
    while settleTimeout > 0
        sim.step(0.1, 20, 20);
        pause(0);
        settleTimeout = settleTimeout - 0.1;
        vMax = 0;
        for i = 1:nBalls
            vMax = max(vMax, norm(balls(i).LinearVelocity));
        end
        viewer.displayText(sprintf("Settling...   max speed: %.2f", vMax));
        if vMax < 0.05
            break
        end
    end
    delete(sim);

    % Final structure analysis: bonds, ionic order and cluster sizes
    [order, nBonds, adj] = bondOrder(balls, chg, d);
    clusterId = zeros(1, nBalls);
    nClusters = 0;
    for i = 1:nBalls
        if clusterId(i) == 0
            nClusters = nClusters + 1;
            stack = i;
            while ~isempty(stack)
                j = stack(end); stack(end) = [];
                if clusterId(j) == 0
                    clusterId(j) = nClusters;
                    stack = [stack find(adj(j, :) & clusterId == 0)]; %#ok<AGROW> DFS
                end
            end
        end
    end
    sizes = accumarray(clusterId', 1);

    viewer.displayText(sprintf("Done:  %d clusters, ionic order %.0f %%", ...
        nnz(sizes > 1), 100*order));
    fprintf("Assembled %d bonds with %.0f %% ionic order (opposite-charge contacts).\n", ...
        nBonds, 100*order);
    fprintf("Clusters of 2+ balls: %d, largest cluster: %d balls, singles: %d.\n", ...
        nnz(sizes > 1), max(sizes), nnz(sizes == 1));

    % Crystallization curve and the cluster size histogram
    figure(2);
    subplot(2, 1, 1);
    yyaxis left
    plot(hist.t, hist.bonds, "LineWidth", 1.5); hold on
    plot(hist.t, hist.bonds.*hist.order, ":", "LineWidth", 1.5);
    ylabel("bonds");
    yyaxis right
    plot(hist.t, 100*hist.T, "--", "LineWidth", 1.5); ylabel("temperature [%]");
    grid on; xlabel("annealing time [s]");
    legend("bonds", "opposite-charge bonds", "temperature", "Location", "west");
    title(sprintf("Crystallization: bonds form as the system cools (%d balls)", nBalls));
    subplot(2, 1, 2);
    histogram(sizes, 0.5:1:max(sizes) + 0.5);
    grid on; xlabel("cluster size [balls]"); ylabel("count");
    title(sprintf("Final clusters: ionic order %.0f %%", 100*order));

    function [order, nBonds, adj] = bondOrder(balls, chg, d)
    % Bonds are pairs of (nearly) touching balls; the ionic order is the
    % fraction of bonds connecting opposite charges
        n = numel(balls);
        P = zeros(n, 3);
        for b = 1:n
            P(b, :) = balls(b).Position;
        end
        D = sqrt(sum((permute(P, [1 3 2]) - permute(P, [3 1 2])).^2, 3));
        adj = D > 0 & D < 1.25*d;
        [bi, bj] = find(triu(adj));
        nBonds = numel(bi);
        if nBonds > 0
            order = mean(chg(bi).*chg(bj) < 0);
        else
            order = 0;
        end
    end

end
