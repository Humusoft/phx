function phxex_sorter(nBalls, fieldStrength, seed)
% PHXEX_SORTER Electrostatic sorting of falling balls by their charge
%
% Balls with a positive, negative or zero charge fall one by one through
% the gap between two charged electrode plates - a parallel-plate
% capacitor. The electric force (phx.Monopole) deflects every ball
% sideways in proportion to its charge, so the balls sort themselves into
% three bins: positive balls toward the negative plate on the left,
% neutral in the middle, negative toward the positive plate on the right.
% The same principle drives a mass spectrometer or an electrostatic
% precipitator.
%
% Each ball gets its own phx.Monopole group, so it interacts with the
% plates but not with the other balls; a separate Monopole draws the
% field lines between the plates. After all balls settle, the landing
% position is compared with the known charge - the final scatter plot
% shows the linear charge-to-deflection relation and the sorting accuracy.
%
% Input Arguments:
%     nBalls        - number of dropped balls, a multiple of 3 (default 45)
%     fieldStrength - relative electrode charge scale (default 1)
%     seed          - random seed for charges and jitter (default 0)
%
% Example:
%     phxex_sorter            % default sorter
%     phxex_sorter(45, 0.5)   % weaker field -> deflection may not suffice

%   Copyright 2026 HUMUSOFT s.r.o.

    arguments
        nBalls (1, 1) double {mustBeInteger, mustBePositive} = 45
        fieldStrength (1, 1) double {mustBePositive} = 1
        seed (1, 1) double = 0
    end

    rng(seed); % Random seed for reproducible charges and jitter

    % Geometry: bins at the bottom, plates above, drop point on top
    d = 0.6;               % ball diameter
    xGap = 3;              % plate inner faces at +-xGap
    zPlate = [4 10];       % vertical extent of the plates
    zDrop = 11;            % drop height
    binW = 2.1;            % width of one bin
    hDiv = 1.2;            % bin divider height
    qBall = 1;             % unit ball charge
    qPlate = 2.2*fieldStrength;  % charge of one electrode pellet

    % Figure setup
    figure(1);
    [viewer, ax] = phx.extra.Viewer("clear", "DefaultCameraTarget", [0 0 5], ...
        "DefaultCameraPosition", [2 -26 9]);

    % Static floor and bin dividers (negative | neutral | positive)
    phx.Body(ax, "Type", "static", "Position", [0 0 -0.5], ...
        "Shape", {"Box", "Size", [16 6 1], "Color", [1 1 1]}, "Friction", [0.4 0.01 0]);
    for x = [-1.5 -0.5 0.5 1.5]*binW
        phx.Body(ax, "Type", "static", "Position", [x 0 hDiv/2], ...
            "Shape", {"Box", "Size", [0.15 3 hDiv], "Color", [0.55 0.55 0.6]});
    end
    for sgn = [-1 1]   % rails closing the open ends of the bins
        phx.Body(ax, "Type", "static", "Position", [0, sgn*1.6, hDiv/2], ...
            "Shape", {"Box", "Size", [16 0.2 hDiv], "Color", [0.55 0.55 0.6]});
    end

    % Electrode plates with a column of charge-carrying pellets each;
    % the left plate is negative (blue), the right one positive (red)
    hPlate = zPlate(2) - zPlate(1);
    zMid = mean(zPlate);
    phx.Body(ax, "Type", "static", "Position", [-xGap - 0.25, 0, zMid], ...
        "Shape", {"Box", "Size", [0.5 2.5 hPlate], "Color", [0.55 0.6 0.9]});
    phx.Body(ax, "Type", "static", "Position", [xGap + 0.25, 0, zMid], ...
        "Shape", {"Box", "Size", [0.5 2.5 hPlate], "Color", [0.9 0.55 0.5]});
    pellets = phx.Body.empty;
    for z = zPlate(1) + 0.5:1:zPlate(2) - 0.5
        for sgn = [-1 1]
            pellets(end + 1) = phx.Body(ax, "Type", "static", ...
                "Position", [sgn*xGap, 0, z], "Shape", {"Sphere", "Diameter", 0.35, ...
                "Division", 2, "Color", [0.7 0.7 0.75], "Material", "metal"}); %#ok<AGROW> small fixed size
        end
    end
    qPellets = repmat([-1 1]*qPlate, 1, numel(pellets)/2);

    % Field-line visualization between the plates (electrodes only - the
    % static bodies do not move, so this group adds no dynamics)
    phx.Monopole(pellets, "Charge", qPellets', "Attractivity", -1, ...
        "VectorFieldCenter", [0 0 zMid], "VectorFieldSize", [2*xGap - 0.8, 0, hPlate], ...
        "VectorFieldStep", 0.8, "VectorLength", 1, "VectorSegments", 5, "Color", [1 1 1]);

    % Balanced random sequence of ball charge classes (-1, 0, +1)
    nBalls = 3*ceil(nBalls/3);
    cls = repmat([-1 0 1], 1, nBalls/3);
    cls = cls(randperm(nBalls));
    clsClr = [0.45 0.55 1; 0.7 0.7 0.7; 1 0.45 0.4];   % blue, gray, red

    % On-screen readout
    viewer.displayText("Released: 0 / " + nBalls);

    % Simulation of the static scene; balls are added later, on the fly
    sim = phx.Simulation(ax);

    % Phase 1 - drop the balls one by one through the capacitor gap; each
    % ball is spawned into the running simulation with its own monopole
    % group, so it feels the plates but not the other balls
    releasePeriod = 0.4;
    dt = 0.05;
    released = 0;
    charges = zeros(1, nBalls);
    traced = false;
    while released < nBalls
        if sim.Time >= released*releasePeriod
            i = released + 1;
            % The charge magnitude varies, the deflection scales with it
            charges(i) = cls(i)*qBall*(0.8 + 0.4*rand);
            balls(i) = phx.Body(ax, "Position", [(rand - 0.5)*0.4, 0, zDrop], ...
                "Shape", {"Sphere", "Diameter", d, "Division", 2, "Density", 8, ...
                "Color", clsClr(cls(i) + 2, :)}, "Friction", [0.3 0.01 0]); %#ok<AGROW> unknown rate
            phx.Monopole([pellets balls(i)], "Charge", [qPellets charges(i)]', ...
                "Attractivity", -1, "VectorFieldSize", [0 0 0], "Visible", false);
            if ~traced && cls(i) ~= 0
                phx.Trace(balls(i), "TracePoints", 300, "Overlay", true);
                traced = true;
            end
            sim.addObjects(balls(i));
            released = i;
            viewer.displayText(sprintf("Released: %d / %d", released, nBalls));
        end
        sim.step(dt, 5, 5);
        pause(0);
    end

    % Phase 2 - let the last balls land and everything settle
    settleTimeout = sim.Time + 10;
    while sim.Time < settleTimeout
        sim.step(0.1, 10, 10);
        pause(0);
        vMax = 0;
        for i = 1:nBalls
            vMax = max(vMax, norm(balls(i).LinearVelocity));
        end
        viewer.displayText(sprintf("Settling...   max speed: %.2f", vMax));
        if vMax < 0.1
            break
        end
    end
    delete(sim);

    % Classify every ball by its landing bin and compare with the charge
    xFinal = zeros(1, nBalls);
    for i = 1:nBalls
        p = balls(i).Position;
        xFinal(i) = p(1);
    end
    % A positive ball is pulled toward the negative plate (left), so the
    % expected bin index is the negative of the charge class
    binCls = max(-1, min(1, round(xFinal/binW)));
    accuracy = mean(binCls == -cls);

    fprintf("Sorting accuracy: %.1f %% of %d balls (field strength %.2f).\n", ...
        100*accuracy, nBalls, fieldStrength);
    for c = [-1 0 1]
        fprintf("  class %+d: %2d balls, landing x = %6.2f +- %.2f\n", c, ...
            nnz(cls == c), mean(xFinal(cls == c)), std(xFinal(cls == c)));
    end
    viewer.displayText(sprintf("Done: sorting accuracy %.1f %%", 100*accuracy));

    % The money plot: landing position is proportional to the charge
    figure(2);
    hold on
    for c = [-1 0 1]
        id = cls == c;
        scatter(charges(id), xFinal(id), 36, clsClr(c + 2, :), "filled");
    end
    yline([-0.5 0.5]*binW, "--", ["bin edge" "bin edge"]);
    grid on; xlabel("ball charge q"); ylabel("landing position x");
    title(sprintf("Charge sorting: %.1f %% correct (%d balls)", 100*accuracy, nBalls));
    legend("negative", "neutral", "positive", "Location", "northwest");

end
