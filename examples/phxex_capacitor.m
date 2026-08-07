function phxex_capacitor(nBalls, rampTime, seed)
% PHXEX_CAPACITOR Charged balls sorting themselves between capacitor plates
%
% An arena is filled with a random mixture of balls carrying a positive,
% negative or zero charge, and its two opposite walls act as the plates of
% a parallel-plate capacitor - each of them holds a row of charge-carrying
% pellets, the left plate negative (blue), the right one positive (red).
% Everything belongs to a single phx.Monopole group, so the balls also
% attract and repel each other; the structure that forms is the result of
% the competition between these mutual bonds and the external field.
%
% The plate charge is ramped up from zero, so the whole transition can be
% watched: at a weak field the mixed clusters formed by the mutual
% attraction survive, and only once the plate pull grows to about the
% strength of one contact bond does the mixture unmix itself. Positive
% balls then migrate to the negative plate and negative balls to the
% positive one, where like charges spread out along the plate into a
% closely packed layer. The neutral balls feel no force at all - they only
% get stirred by collisions, so they stay scattered over the whole gap
% instead of forming a band of their own.
%
% Once the layers are formed the polarity of the plates is reversed, and
% the two counter-flowing streams swap sides in a few seconds - the same
% trick that releases a collected layer from an electrode in practice.
%
% The pellet rows sit at the height of the centres of the resting balls, so
% the pull stays horizontal and the balls do not climb over each other.
% Separation is measured throughout the run as the mean position of each
% charge class and the share of charged balls on the side they are pulled
% to at that moment.
%
% Input Arguments:
%     nBalls   - number of balls, a multiple of 3 (default 90)
%     rampTime - duration of the field ramp in seconds (default 12)
%     seed     - random seed for the layout and charges (default 0)
%
% Example:
%     phxex_capacitor            % default run
%     phxex_capacitor(90, 2)     % fast ramp -> clusters are torn apart

%   Copyright 2026 HUMUSOFT s.r.o.

    arguments
        nBalls (1, 1) double {mustBeInteger, mustBePositive} = 90
        rampTime (1, 1) double {mustBePositive} = 12
        seed (1, 1) double = 0
    end

    rng(seed); % Random seed for reproducible layout and charges

    % Particles, plates and the field
    d = 0.3;               % ball diameter
    rho = 100;             % ball density -> mass ~1.4 kg
    qBall = 0.85;          % charge magnitude of a charged ball
    qPlate = 7.3;          % charge of one electrode pellet at full field
    pitch = 0.5;           % pellet spacing along the plate
    mu = [0.05 0.01 0];    % low friction lets the balls rearrange
    arena = [5 5 0.8];     % inner arena size

    % Figure setup
    figure(1);
    [viewer, ax] = phx.extra.Viewer("clear", "DefaultCameraTarget", [0 0 0.3], ...
        "DefaultCameraPosition", [0 -7 6]);

    % Static arena; the two x walls are the capacitor plates
    parts = phx.assembly.arena(ax, "Size", arena, "Thickness", 0.3, ...
        "Color", [1 1 1], "Friction", mu);
    parts.walls(1).Color = [0.55 0.6 0.9];   % -x plate, negative
    parts.walls(2).Color = [0.9 0.55 0.5];   % +x plate, positive

    % Charge carriers: a row of pellets on the inner face of each plate, at
    % the height of the resting ball centres, so that the force on a ball
    % lying on the floor is horizontal and does not lift it onto its
    % neighbours
    yPellets = -(arena(2) - pitch)/2:pitch:(arena(2) - pitch)/2;
    pellets = phx.Body.empty;
    for sgn = [-1 1]
        for y = yPellets
            pellets(end + 1) = phx.Body(ax, "Type", "static", ...
                "Position", [sgn*arena(1)/2, y, d/2], ...
                "Shape", {"Sphere", "Diameter", 0.4, "Division", 2, ...
                "Color", [0.7 0.7 0.75], "Material", "metal"}); %#ok<AGROW> small fixed size
        end
    end
    nPellets = numel(yPellets);
    qUnit = [-ones(1, nPellets), ones(1, nPellets)];   % plate polarity

    % Random mixture of charge classes (-1, 0, +1), one third each
    nBalls = 3*ceil(nBalls/3);
    cls = repmat([-1 0 1], 1, nBalls/3);
    cls = cls(randperm(nBalls));
    clsClr = [0.4 0.55 1; 0.7 0.7 0.7; 1 0.4 0.35];   % blue, gray, red

    % Balls dropped at random non-overlapping positions over the whole gap
    balls = phx.assembly.scatter(ax, {"Sphere", "Diameter", d, "Division", 2, ...
        "Density", rho}, nBalls, "Region", [arena(1) - 0.6, arena(2) - 0.6, 1], ...
        "Spacing", 0.45, "Position", [0 0 0.3], "Friction", mu, ...
        "Color", clsClr(cls + 2, :));

    % One interaction group for the whole scene: the balls feel the plates
    % and each other, the static pellets feel nothing. The plate charge
    % starts at zero and is ramped up during the simulation
    field = phx.Monopole([pellets balls], "Charge", [0*qUnit, cls*qBall]', ...
        "Attractivity", -1, "VectorFieldSize", [0 0 0], "Visible", false);

    % Field lines between the plates (the pellets alone - static bodies add
    % no dynamics, so this group is a pure visualization)
    phx.Monopole(pellets, "Charge", qUnit'*qPlate, "Attractivity", -1, ...
        "VectorFieldCenter", [0 0 d/2], "VectorFieldSize", [arena(1) - 0.7, arena(2) - 1.2, 0], ...
        "VectorFieldStep", 0.95, "VectorLength", 0.6, "VectorSegments", 4, "Color", [1 1 1]);

    % Sleeping must stay disabled, otherwise slowly drifting balls would
    % stop responding to the field
    sim = phx.Simulation(ax);

    % Phase 1 - let the balls land and clump with no field applied
    sim.step(1.5, 300, 15);

    % Phase 2 - ramp the plate charge from zero to full and hold it, then
    % reverse the polarity and let the layers swap sides
    flipTime = 8;
    tFlip = rampTime + 3;
    hist = struct("t", [], "level", [], "meanX", [], "share", []);
    tStart = sim.Time;
    while sim.Time - tStart < tFlip + flipTime
        t = sim.Time - tStart;
        if t < tFlip
            level = min(1, t/rampTime);       % ramping up
            pol = 1;
        else
            level = -1;                       % plates swapped over
            pol = -1;
        end
        field.Charge = [qUnit*qPlate*level, cls*qBall]';
        sim.step(0.05, 10, 10);
        [meanX, share] = separation(balls, cls, pol);
        hist.t(end + 1) = sim.Time - tStart;
        hist.level(end + 1) = level;
        hist.meanX(:, end + 1) = meanX;
        hist.share(end + 1) = share;
        viewer.displayText(sprintf("field %+4.0f %%    charged balls on the correct side: %3.0f %%", ...
            100*level, 100*share));
    end
    delete(sim);

    % Force scales: the bond between two touching balls, and the pull of
    % both plates on a ball in the middle of the gap at full field
    p = reshape([pellets.Position], 3, [])';
    r = vecnorm(p, 2, 2);
    fBond = qBall^2/d^2;
    fPull = qBall*qPlate*sum(abs(p(:, 1))./r.^3);

    % The field level at which the charged balls got half way to their
    % plate - the point where the field starts to win over the bonds
    iFlip = find(hist.level < 0, 1);
    xCharged = mean(abs(hist.meanX([1 3], :)), 1);
    iCrit = find(xCharged > 0.5*xCharged(iFlip - 1), 1);

    % Time the swap took: from the reversal to the first moment the balls
    % are back on the side they are now pulled to
    iBack = find(hist.share(iFlip:end) > 0.9, 1) + iFlip - 1;

    [~, share] = separation(balls, cls, -1);
    nWrong = round((1 - share)*nnz(cls ~= 0));
    fprintf("Separation at full field: %.0f %% of the charged balls on the correct side.\n", ...
        100*hist.share(iFlip - 1));
    fprintf("Half of the separation is reached at %.0f %% field, where the plate pull " + ...
        "(%.1f N) is about the strength of one contact bond (%.1f N).\n", ...
        100*hist.level(iCrit), fPull*hist.level(iCrit), fBond);
    fprintf("After the polarity is reversed the layers swap sides in %.1f s, " + ...
        "ending at %.0f %% correct (%d wrong).\n", ...
        hist.t(iBack) - hist.t(iFlip), 100*share, nWrong);
    names = ["negative" "neutral " "positive"];
    for c = [-1 0 1]
        x = ballsX(balls(cls == c));
        fprintf("  %s: %2d balls, x = %+5.2f +- %.2f\n", names(c + 2), nnz(cls == c), ...
            mean(x), std(x));
    end
    viewer.displayText(sprintf("Done: layers swapped, %.0f %% of the charged balls on the correct side", ...
        100*share));

    % Separation curves and the field level at which the mixture breaks up
    clf(figure(2));
    subplot(2, 1, 1);
    yyaxis left
    hold on
    for c = [-1 0 1]
        plot(hist.t, hist.meanX(c + 2, :), "LineStyle", "-", "LineWidth", 1.5, ...
            "Color", clsClr(c + 2, :));
    end
    xline(hist.t(iFlip), ":", "polarity reversed", "HandleVisibility", "off", ...
        "LabelVerticalAlignment", "bottom");
    ylabel("mean position x"); ylim([-1 1]*arena(1)/2);
    set(gca, "YColor", [0.15 0.15 0.15]);
    yyaxis right
    plot(hist.t, 100*hist.level, "--", "LineWidth", 1.5); ylabel("field [%]");
    grid on; xlabel("time [s]");
    legend([names "field"], "Location", "west");
    title(sprintf("Charge separation and swap: %.0f %% of the charged balls on their plate", ...
        100*share));
    subplot(2, 1, 2);
    ramp = 1:find(hist.level == 1, 1);
    plot(100*hist.level(ramp), 100*hist.share(ramp), "LineWidth", 1.5);
    xline(100*hist.level(iCrit), "--", "bonds give way");
    grid on; xlabel("field [%]"); ylabel("correct side [%]");
    title("The mixture unmixes only above a threshold field");

    function [meanX, share] = separation(balls, cls, pol)
    % Mean position of each charge class and the share of the charged balls
    % in the half of the gap they are pulled to at the given plate polarity
        x = ballsX(balls);
        meanX = [mean(x(cls == -1)); mean(x(cls == 0)); mean(x(cls == 1))];
        charged = cls ~= 0;
        share = mean(sign(x(charged)) == -pol*cls(charged));
    end

    function x = ballsX(balls)
    % Positions of the given balls along the field direction
        x = zeros(1, numel(balls));
        for i = 1:numel(balls)
            q = balls(i).Position;
            x(i) = q(1);
        end
    end

end
