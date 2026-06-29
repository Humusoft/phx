function phxex_galton(nBalls, nRows, seed)
% PHXEX_GALTON Galton board - a binomial distribution built from collisions
%
% Balls are released one by one above a triangular grid of pins. Each pin
% deflects a falling ball randomly to the left or to the right, so after
% passing all rows the ball lands in one of the bins at the bottom. The
% growing piles reproduce the binomial (approximately normal) distribution
% - a statistical law emerging from nothing but rigid-body contacts
% between hundreds of bodies. Resolving contacts among arbitrary body
% pairs is exactly what a collision engine does for free, while defining
% every contact pair by hand would be impractical.
%
% The balls do not exist when the simulation starts - each one is created
% on the fly and added to the running simulation with
% phx.Simulation.addObjects, demonstrating runtime changes of the scene.
% The path of the first ball is drawn with phx.Trace. After all balls
% settle, they are recolored by the bin they ended up in and the measured
% histogram is compared with the theoretical binomial expectation.
%
% Input Arguments:
%     nBalls - number of released balls (default 150)
%     nRows  - number of pin rows, giving nRows+1 bins (default 9)
%     seed   - random seed for the release jitter (default 0)
%
% Example:
%     phxex_galton            % default board
%     phxex_galton(300, 11)   % more balls and rows -> smoother histogram

%   Copyright 2026 HUMUSOFT s.r.o.

    arguments
        nBalls (1, 1) double {mustBeInteger, mustBePositive} = 150
        nRows (1, 1) double {mustBeInteger, mustBeGreaterThanOrEqual(nRows, 3)} = 9
        seed (1, 1) double = 0
    end

    rng(seed); % Random seed for reproducible release jitter

    % Board proportions (everything derives from the ball diameter)
    d = 1;                 % ball diameter
    sx = 3*d;              % horizontal pin pitch = bin width
    pinD = 0.8*d;          % pin diameter
    dz = 1.6*d;            % vertical distance between pin rows
    gapY = 1.25*d;         % depth between the front and back glass
    mu = [0.05 0.01 0];    % low friction [drag roll spin] keeps balls flowing

    nBins = nRows + 1;
    halfW = nBins/2*sx;    % board half-width (outer bin edges)

    % Theoretical binomial distribution and the bin height needed to hold it
    pmf = arrayfun(@(k) nchoosek(nRows, k), 0:nRows)*0.5^nRows;
    hBin = ceil(nBalls*max(pmf)*1.3/1.7) + 2;

    % Vertical layout: bins at the bottom, pin rows above, drop point on top
    zRowBot = hBin + 1.5;
    zRowTop = zRowBot + (nRows - 1)*dz;
    zDrop = zRowTop + 2.0;
    hWall = zDrop + 2;

    % Figure setup
    figure(1);
    [viewer, ax] = phx.extra.Viewer("clear", "DefaultCameraTarget", [0 0 hWall/2], ...
        "DefaultCameraPosition", [0 -2.0*hWall 0.6*hWall]);

    % Static floor
    phx.Body(ax, "Type", "static", "Position", [0 0 -0.5], ...
        "Shape", {"Box", "Size", [2*halfW + 6, 6, 1], "Color", [1 1 1]}, "Friction", mu);

    % Back glass (solid) and front glass (wireframe, so we can see inside);
    % both collide with the balls and keep the board effectively 2D
    phx.Body(ax, "Type", "static", "Position", [0 gapY/2 + 0.15, hWall/2], ...
        "Shape", {"Box", "Size", [2*halfW + 0.4, 0.3, hWall], "Color", [0.85 0.87 0.92]}, ...
        "Friction", mu);
    phx.Body(ax, "Type", "static", "Position", [0, -gapY/2 - 0.15, hWall/2], ...
        "Shape", {"Box", "Size", [2*halfW + 0.4, 0.3, hWall], ...
        "Style", "wireframe", "Color", [0.6 0.7 0.8], "ForcePatch", true}, "Friction", mu);

    % Side walls (outer edges of the first and last bin)
    for sgn = [-1 1]
        phx.Body(ax, "Type", "static", "Position", [sgn*(halfW + 0.1), 0, hWall/2], ...
            "Shape", {"Box", "Size", [0.2 gapY hWall], "Color", [0.5 0.5 0.55]}, ...
            "Friction", mu);
    end

    % Bin dividers (interior bin edges only, side walls close the outer bins)
    for j = 1:nRows
        phx.Body(ax, "Type", "static", "Position", [(j - nBins/2)*sx, 0, hBin/2], ...
            "Shape", {"Box", "Size", [0.2 gapY hBin], "Color", [0.5 0.5 0.55]}, ...
            "Friction", mu);
    end

    % Pin grid (quincunx): odd rows have a pin right below the drop point,
    % even rows are shifted by half a pitch. The outermost pins sit half
    % buried in the side walls, so there is no free corridor along them.
    m = round(halfW/sx);
    for r = 1:nRows
        z = zRowTop - (r - 1)*dz;
        if mod(r, 2) == 1
            xPins = (-m:m)*sx;
        else
            xPins = ((-m:m-1) + 0.5)*sx;
        end
        for x = xPins
            phx.Body(ax, "Type", "static", "Position", [x 0 z], ...
                "Shape", {"Cylinder", "Diameter", pinD, "Height", gapY, "Axis", "y", ...
                "Segments", 18, "Color", [0.75 0.75 0.78], "Material", "metal"}, ...
                "Friction", mu);
        end
    end

    % On-screen readout
    viewer.displayText("Released: 0 / " + nBalls);

    % Simulation of the static board; balls are added later, on the fly
    sim = phx.Simulation;

    % Phase 1 - release the balls one by one above the top pin and let the
    % board do its work; each ball is spawned into the running simulation
    shp = phx.shape.Sphere("Diameter", d, "Division", 2);
    releasePeriod = 0.7;             % time between two released balls
                                     % (long enough so the balls do not
                                     % collide with each other in the grid)
    dt = 0.05;                       % outer simulation step
    released = 0;
    while released < nBalls
        if sim.Time >= released*releasePeriod
            jit = (rand(1, 1) - 0.5)*1.0;    % entry jitter randomizes the path
                                             % (the ball still hits the top pin)
            shp = shp.nextColor;
            balls(released + 1) = phx.Body(ax, "Position", [jit 0 zDrop], ...
                "Shape", shp, "Friction", mu, "Mass", 1, "Inertia", 0.1); %#ok<AGROW> unknown rate
            % Mild air drag damps the sideways motion between pin rows, so
            % the ball makes a proper left/right decision at every row
            % phx.Resistance(balls(released + 1), "VelocityFactors", [0 2]);
            if released == 0
                phx.Trace(balls(1), "TracePoints", 600, "Overlay", true);
            end
            sim.addObjects(balls(released + 1));
            released = released + 1;
            viewer.displayText(sprintf("Released: %d / %d", released, nBalls));
        end
        sim.step(dt, 5, 5);
        pause(0);
    end

    % Phase 2 - let the last balls trickle down and the piles settle
    settleTimeout = sim.Time + 20;
    while sim.Time < settleTimeout
        sim.step(0.1, 10, 10);
        %pause(0);
        vMax = 0;
        for i = 1:nBalls
            vMax = max(vMax, norm(balls(i).LinearVelocity));
        end
        viewer.displayText(sprintf("Settling...   max speed: %.2f", vMax));
        if vMax < 0.15
            break
        end
    end
    delete(sim);

    % Classify every ball into its bin by the final x position
    xFinal = zeros(1, nBalls);
    for i = 1:nBalls
        p = balls(i).Position;
        xFinal(i) = p(1);
    end
    binIdx = max(0, min(nRows, round(xFinal/sx + nRows/2)));
    counts = accumarray(binIdx(:) + 1, 1, [nBins 1]);

    % Recolor the balls by their bin to highlight the histogram in the scene
    clr = (jet(nBins) + 1)/2;
    for i = 1:nBalls
        balls(i).Color = clr(binIdx(i) + 1, :);
    end
    drawnow;
    viewer.displayText(sprintf("Done: %d balls in %d bins", nBalls, nBins));

    % Report the sample statistics against the binomial theory
    stdTheory = sqrt(nRows)*sx/2;
    fprintf("Balls per bin:    %s\n", join(string(counts'), " "));
    fprintf("Expected per bin: %s\n", join(string(round(nBalls*pmf, 1)), " "));
    fprintf("Final x position: mean %.2f (theory 0), std %.2f (theory %.2f).\n", ...
        mean(xFinal), std(xFinal), stdTheory);

    % Compare the measured histogram with the binomial expectation
    figure(2);
    xc = ((0:nRows) - nRows/2)*sx;
    bar(xc, counts, 0.9);
    hold on
    plot(xc, nBalls*pmf, "o-", "LineWidth", 1.5);
    grid on; xlabel("bin position x"); ylabel("number of balls");
    legend("simulated", "binomial expectation");
    title(sprintf("Galton board: %d balls, %d pin rows", nBalls, nRows));

end
