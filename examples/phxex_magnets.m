function phxex_magnets(nBalls, annealTime, seed)
% PHXEX_MAGNETS Self-assembly of spherical magnets into chains
%
% A handful of spherical magnets (the "neocube" toy) is dropped into an
% arena. Every ball carries a phx.Dipole with the poles buried inside the
% sphere and its polarity painted on the surface - the red half is the
% north pole. The balls are kicked by random thermal forces and torques
% whose amplitude anneals to zero. While hot, they tumble and explore;
% as the system cools, the dipole interaction takes over and the magnets
% snap together head-to-tail, growing chains and occasionally closing
% rings - the same structures the real toy forms, emerging purely from
% the dipole forces, collisions and friction.
%
% The bonds are classified by the mutual orientation of the two magnetic
% moments: a chain bond has the moments parallel and pointing along the
% bond. The final report counts the chains, their lengths and possible
% rings, and the plot shows the growth of bonds and of the head-to-tail
% alignment during the annealing.
%
% Input Arguments:
%     nBalls     - number of magnets (default 24)
%     annealTime - duration of the cooling phase in seconds (default 10)
%     seed       - random seed (default 0)
%
% Example:
%     phxex_magnets            % default run
%     phxex_magnets(36, 15)    % more magnets, slower annealing

%   Copyright 2026 HUMUSOFT s.r.o.

    arguments
        nBalls (1, 1) double {mustBeInteger, mustBePositive} = 24
        annealTime (1, 1) double {mustBePositive} = 10
        seed (1, 1) double = 0
    end

    rng(seed); % Random seed for reproducible spawn and thermal noise

    % Magnets and forces
    d = 0.5;               % ball diameter
    rho = 50;              % ball density -> mass ~3.3 kg
    h = 0.15;              % pole offset from the center (inside the ball)
    q = 8;                 % pole charge: capture radius ~3 ball diameters
    fThermal = 200;        % initial thermal force amplitude (N)
    tThermal = 4;          % initial thermal torque amplitude (N*m)
    mu = [0.2 0.15 0.15];  % rolling/spinning friction dissipates the
                           % rotational energy released by snapping magnets
    arena = 6;             % arena size

    % Figure setup
    figure(1);
    [viewer, ax] = phx.extra.Viewer("clear", "DefaultCameraTarget", [0 0 0.5], ...
        "DefaultCameraPosition", [0 -11 8]);

    % Static arena: floor and low walls
    phx.assembly.arena("Size", [arena, arena, 0.8], "Thickness", 0.2, "Color", [1 1 1], "Friction", mu);

    % Spherical magnets: a globe shape with the north half painted red and
    % the south half blue; the dipole axis runs through the texture poles
    shp = phx.shape.Globe("Diameter", d, "Density", rho, "Segments", 24);
    shp = shp.colormapTexture([1; 0], [0.35 0.45 1; 1 0.35 0.3]);
    cols = ceil(sqrt(nBalls));
    pitch = (arena - 2)/(cols - 1);
    for i = 1:nBalls
        gx = mod(i - 1, cols); gy = floor((i - 1)/cols);
        p = [gx*pitch - (arena - 2)/2, gy*pitch - (arena - 2)/2] + (rand(1, 2) - 0.5)*0.4;
        balls(i) = phx.Body(ax, "Position", [p, 1 + rand], ...
            "Shape", shp, "Friction", mu); %#ok<AGROW> small fixed size
        % Random initial orientation
        [R, ~] = qr(randn(3));
        balls(i).Orientation = R*diag([det(R) 1 1]);
    end

    % One dipole group for all magnets; the poles sit on the local Z axis
    phx.Dipole(balls, "Charge", q*ones(nBalls, 1), "Attractivity", -1, ...
        "Axis", repmat([0 0 h], nBalls, 1), "VectorFieldSize", [0 0 0], ...
        "Visible", false);

    % Air drag dissipates the energy released by the snapping magnets
    phx.Resistance(balls, "VelocityFactors", [0 5]);

    % On-screen readout
    viewer.displayText("Dropping...");

    % Sleeping must stay disabled, otherwise slow magnets would stop
    % responding to the dipole field
    sim = phx.Simulation(ax, "EngineSettings", ...
        phx.engine.BulletSettings("AutoActivated", false));
    dt = 0.005;
    subSteps = 10;

    % Phase 1 - drop
    sim.step(1, 200, 20);

    % Phase 2 - annealing with random forces and torques; the bond count
    % and the head-to-tail alignment are recorded as the system cools
    hist = struct("t", [], "T", [], "bonds", [], "align", []);
    lastAlign = 0;
    frame = 0;
    t = 0;
    while t < annealTime
        T = 1 - t/annealTime;
        for i = 1:nBalls
            dir = randn(1, 3).*[1 1 0.3];
            balls(i).applyForce(fThermal*T*dir/norm(dir), [0 0 0], false);
            balls(i).applyTorque(tThermal*T*randn(1, 3), false);
        end
        sim.step(dt*subSteps, subSteps, subSteps);
        t = t + dt*subSteps;
        frame = frame + 1;
        pause(0);

        if mod(frame, 10) == 0                % sample every 0.5 s
            [adj, alignment] = bondAlignment(balls, d);
            lastAlign = sum(alignment(adj(:)))/max(nnz(adj), 1);
            hist.t(end + 1) = t;
            hist.T(end + 1) = T;
            hist.bonds(end + 1) = nnz(triu(adj));
            hist.align(end + 1) = lastAlign;
        end
        viewer.displayText(sprintf("Annealing:  temperature %3.0f %%   head-to-tail alignment %3.0f %%", ...
            100*T, 100*lastAlign));
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

    % Final structure analysis: chains, rings and bond alignment
    [adj, alignment] = bondAlignment(balls, d);
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
    deg = sum(adj, 2)';
    nRings = 0;
    for c = 1:nClusters
        id = clusterId == c;
        if sizes(c) >= 3 && all(deg(id) == 2)
            nRings = nRings + 1;
        end
    end
    meanAlign = sum(alignment(adj(:)))/max(nnz(adj), 1);

    viewer.displayText(sprintf("Done:  longest chain %d, mean alignment %.0f %%", ...
        max(sizes), 100*meanAlign));
    fprintf("Bonds: %d, mean head-to-tail alignment %.0f %%.\n", ...
        nnz(triu(adj)), 100*meanAlign);
    fprintf("Chains of 2+ magnets: %d (longest %d), rings: %d, singles: %d.\n", ...
        nnz(sizes > 1), max(sizes), nRings, nnz(sizes == 1));

    % Assembly curve and the chain length histogram
    figure(2);
    subplot(2, 1, 1);
    yyaxis left
    plot(hist.t, hist.bonds, "LineWidth", 1.5); hold on
    plot(hist.t, hist.bonds.*hist.align, ":", "LineWidth", 1.5);
    ylabel("bonds");
    yyaxis right
    plot(hist.t, 100*hist.T, "--", "LineWidth", 1.5); ylabel("temperature [%]");
    grid on; xlabel("annealing time [s]");
    legend("bonds", "aligned head-to-tail", "temperature", "Location", "west");
    title(sprintf("Magnet chains grow as the system cools (%d magnets)", nBalls));
    subplot(2, 1, 2);
    histogram(sizes, 0.5:1:max(sizes) + 0.5);
    grid on; xlabel("chain length [magnets]"); ylabel("count");
    title(sprintf("Final chains: longest %d magnets, %d ring(s)", max(sizes), nRings));

end

function [adj, alignment] = bondAlignment(balls, d)
% Bonds are pairs of touching magnets; the alignment of a bond is how
% well the two moments agree with the ideal head-to-tail geometry
% (moments parallel to each other and to the bond direction)

    n = numel(balls);
    P = zeros(n, 3);
    M = zeros(n, 3);
    for b = 1:n
        T = balls(b).Transform;
        P(b, :) = T(13:15);
        M(b, :) = T(9:11);            % local Z axis = dipole direction
    end
    D = sqrt(sum((permute(P, [1 3 2]) - permute(P, [3 1 2])).^2, 3));
    adj = D > 0 & D < 1.25*d;
    alignment = zeros(n);
    [bi, bj] = find(adj);
    for k = 1:numel(bi)
        r = (P(bj(k), :) - P(bi(k), :))/D(bi(k), bj(k));
        alignment(bi(k), bj(k)) = (M(bi(k), :)*r')*(M(bj(k), :)*r');
    end

end
