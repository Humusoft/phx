function phxex_galton(nBalls, nRows, seed)
% PHXEX_GALTON Galton board - a binomial distribution built from collisions
%
% The balls start in a hopper above a triangular grid of pins and trickle
% out through the gap in its middle. Each pin deflects a falling ball
% randomly to the left or to the right, so after passing all rows the ball
% lands in one of the bins at the bottom. The growing piles reproduce the
% binomial (approximately normal) distribution - a statistical law
% emerging from nothing but rigid-body contacts between hundreds of bodies.
% Resolving contacts among arbitrary body pairs is exactly what a
% collision engine does for free, while defining every contact pair by
% hand would be impractical.
%
% After all balls settle, they are recolored by the bin they ended up in
% and the measured histogram is compared with the theoretical binomial
% expectation.
%
% Input Arguments:
%     nBalls - number of balls in the hopper (default 150)
%     nRows  - number of pin rows, giving nRows+1 bins (default 9)
%     seed   - random seed for the jitter of the initial packing (default 0)
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

    rng(seed); % Random seed for a reproducible initial packing

    % Board proportions (everything derives from the ball diameter)
    d = 0.012;             % ball diameter: glass marbles of 12 mm
    sx = 3.0*d;            % horizontal pin pitch = bin width
    pinD = 0.7*d;          % pin diameter
    dz = 1.9*d;            % vertical distance between pin rows
    gapY = 1.2*d;          % depth between the front and back glass

    nBins = nRows + 1;
    halfW = nBins/2*sx;    % board half-width (outer bin edges)

    % Theoretical binomial distribution and the bin height needed to hold it
    pmf = arrayfun(@(k) nchoosek(nRows, k), 0:nRows)*0.5^nRows;
    hBin = (ceil(nBalls*max(pmf)*1.3/1.7) + 2)/100;   % 7.6 mm per ball, plus 20 mm

    % Vertical layout: bins at the bottom, pin rows above, the outlet of the
    % hopper just above the top pin
    zRowBot = hBin + 0.015;
    zRowTop = zRowBot + (nRows - 1)*dz;
    zOut = zRowTop + 0.025;

    % Hopper: two plates sloping down from the side walls to a gap in the
    % middle, filled with balls in a hexagonal packing, lowest places first
    outlet = 1.4*d;        % width of the gap between the plates
    slope = deg2rad(30);   % inclination of the plates
    plateZ = @(x) zOut + (abs(x) - outlet/2)*tan(slope);   % top surface of the plates
    pitch = 1.02*d;
    [col, row] = meshgrid(-ceil(halfW/pitch):ceil(halfW/pitch), 0:ceil(2*nBalls*pitch/halfW));
    xs = (col(:) + mod(row(:), 2)/2)*pitch;
    zs = zOut + d/2 + row(:)*pitch*sqrt(3)/2;
    fits = abs(xs) < halfW - d/2 - 0.001 & (abs(xs) < outlet/2 - d/2 | zs > plateZ(xs) + d/2/cos(slope) + 0.001);
    xs = xs(fits); zs = zs(fits);
    [~, order] = sortrows([zs abs(xs)]);
    xs = xs(order(1:nBalls)) + (rand(nBalls, 1) - 0.5)*0.02*d;
    zs = zs(order(1:nBalls));
    hWall = max(zs) + d;

    % Figure setup
    figure(1);
    [viewer, ax] = phx.extra.Viewer("clear", "DefaultCameraTarget", [0 0 hWall/2], ...
        "DefaultCameraPosition", [0 -2.0*hWall 0.6*hWall]);

    % Static floor (also the anchor for the bin zones)
    floor = phx.Body(ax, "Type", "static", "Position", [0 0 -0.005], ...
        "Shape", {"Box", "Size", [2*halfW + 0.06, 0.06, 0.01], "Color", [1 1 1]});

    % Back glass (solid) and front glass (wireframe, so we can see inside);
    % both collide with the balls and keep the board effectively 2D
    phx.Body(ax, "Type", "static", "Position", [0 gapY/2 + 0.0015, hWall/2], ...
        "Shape", {"Box", "Size", [2*halfW + 0.004, 0.003, hWall], "Color", [0.85 0.87 0.92]});
    phx.Body(ax, "Type", "static", "Position", [0, -gapY/2 - 0.0015, hWall/2], ...
        "Shape", {"Box", "Size", [2*halfW + 0.004, 0.003, hWall], ...
        "Style", "wireframe", "Color", [0.6 0.7 0.8], "ForcePatch", true});

    % Side walls (outer edges of the first and last bin)
    for sgn = [-1 1]
        phx.Body(ax, "Type", "static", "Position", [sgn*(halfW + 0.001), 0, hWall/2], ...
            "Shape", {"Box", "Size", [0.002 gapY hWall], "Color", [0.5 0.5 0.55]});
    end

    % Bin dividers (interior bin edges only, side walls close the outer bins)
    for j = 1:nRows
        phx.Body(ax, "Type", "static", "Position", [(j - nBins/2)*sx, 0, hBin/2], ...
            "Shape", {"Box", "Size", [0.002 gapY hBin], "Color", [0.5 0.5 0.55]});
    end

    % Pin grid (quincunx): odd rows have a pin right below the drop point,
    % even rows are shifted by half a pitch.
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
                "Shape", {"Cylinder", "Diameter", pinD, "Height", gapY, "Axis", "y", "Color", 0.8});
        end
    end

    % Hopper plates, each from a side wall down to the edge of the outlet
    span = halfW - outlet/2;
    for sgn = [-1 1]
        phx.Body(ax, "Type", "static", ...
            "Position", [sgn*(outlet/2 + span/2), 0, zOut + span/2*tan(slope) - 0.0015/cos(slope)], ...
            "Orientation", [cos(slope) 0 -sgn*sin(slope); 0 1 0; sgn*sin(slope) 0 cos(slope)], ...
            "Shape", {"Box", "Size", [span/cos(slope), gapY, 0.003], "Color", [0.5 0.5 0.55]});
    end

    % Balls, packed in the hopper
    shp = phx.shape.Sphere("Diameter", d, "Segments", 10, "Density", 2500);
    for i = 1:nBalls
        shp = shp.nextColor;
        balls(i) = phx.Body(ax, "Position", [xs(i) 0 zs(i)], "Shape", shp, "Friction", 0.1); %#ok<AGROW> preallocation of handle objects
    end

    % One passive phx.Zone per bin (a full-height x-slab) so the piles count
    % themselves; tallied once at the end with update(), free during the run.
    bins = phx.Zone.empty;
    for k = 0:nRows
        bins(k + 1) = phx.Zone(floor, "Position", [(k - nRows/2)*sx, 0, hWall/2], ...
            "Size", [sx, gapY + 0.004, hWall], "SimulationOrder", "none", ...
            "Visible", false);
    end

    sim = phx.Simulation;

    % Phase 1 - the hopper empties over the pins, in substeps of 1 ms; it
    % ends early when no ball has come through for 3 s (a jammed hopper)
    inBins = 0;
    lastArrival = 0;
    while inBins < nBalls && sim.Time - lastArrival < 3
        sim.step(0.05, 50, 1);
        p = vertcat(balls.Position);
        n = nnz(p(:, 3) < zRowBot - d);
        if n > inBins
            lastArrival = sim.Time;
        end
        inBins = n;
        viewer.displayText(sprintf("t = %4.1f s   %d of %d balls in the bins", sim.Time, inBins, nBalls));
    end
    if inBins < nBalls
        viewer.displayText(sprintf("The hopper jammed: %d of %d balls in the bins", inBins, nBalls));
    end

    % Phase 2 - let the piles settle
    sim.step(1, 1000, 1);

    % Tally the settled piles from the zones and recolor each ball by its bin
    update(bins);
    counts = [bins.Count]';
    clr = (jet(nBins) + 1)/2;
    for b = 1:nBins
        set(bins(b).Contents, "Color", clr(b, :));
    end
    delete(sim);

    % Final x positions are still needed for the sample statistics below
    drawnow;
    viewer.displayText(sprintf("Done: %d balls in %d bins", nBalls, nBins));

    % Compare the measured histogram with the binomial expectation
    clf(figure(2));
    xc = ((0:nRows) - nRows/2)*sx;
    bar(xc, counts, 0.9);
    hold on
    plot(xc, nBalls*pmf, "o-", "LineWidth", 1.5);
    grid on; xlabel("bin position x"); ylabel("number of balls");
    legend("simulated", "binomial expectation");
    title(sprintf("Galton board: %d balls, %d pin rows", nBalls, nRows));

end