function out = phxex_drum(Options)
% PHXEX_DRUM Size segregation of a granular charge in a rotating drum
%
% A drum is filled with a well mixed charge of balls of three sizes and turned
% slowly. Small balls percolate through the gaps of the flowing surface layer
% and collect in a core, while the large ones are left on the outside: the
% muesli effect, produced by nothing but contacts between a few hundred
% spheres.
%
% The plotted index is the mean orbit radius of each class, normalised by the
% largest orbit that class can reach. Equal values mean a mixed charge, values
% rising with ball size mean the charge has sorted itself.
%
% Options:
%     Diameters   - ball diameter of each class, smallest first
%     Counts      - number of balls in each class, same length as Diameters
%     DrumRadius  - inner radius of the drum; with the charge it sets the bed
%                   fill, printed at startup, which decides how strongly the
%                   charge sorts
%     Froude      - rotation speed as omega^2*R/g; the slower the drum turns,
%                   the stronger the sorting
%     Revolutions - how long to turn the drum
%     Seed        - random seed of the initial mixed charge
%
% Example:
%     phxex_drum(Revolutions = 12, Froude = 0.06)
%
% See also phxex_galton, phxex_soil

%   Copyright 2026 HUMUSOFT s.r.o.

    arguments
        Options.Diameters (1, :) double {mustBePositive} = [0.27 0.405 0.6075]
        Options.Counts (1, :) double {mustBePositive, mustBeInteger} = [360 120 40]
        Options.DrumRadius (1, 1) double {mustBePositive} = 3.4
        Options.Froude (1, 1) double {mustBePositive} = 0.10
        Options.Revolutions (1, 1) double = 6
        Options.Seed (1, 1) double = 0
    end

    d = Options.Diameters;
    counts = Options.Counts;
    if numel(d) ~= numel(counts)
        error("Diameters and Counts must have the same number of entries.");
    end
    nClass = numel(d);

    R = Options.DrumRadius;
    L = 1.4142*max(d);          % inner length, a nearly two-dimensional drum
    froude = Options.Froude;

    % Report the fill, since it decides whether the charge sorts at all
    volume = counts.*(pi/6).*d.^3;
    fill = 100*sum(volume)/0.6/(pi*R^2*L);
    fprintf("%d balls in %d classes, class volumes %s m3, bed fill ~%.0f%%\n", ...
        sum(counts), nClass, mat2str(round(volume, 2)), fill);

    rng(Options.Seed);

    % Figure setup
    figure(1);
    [~, ax] = phx.extra.Viewer("clear", "Texture", "tiles", "DefaultCameraPosition", [0 4.0*R 0]);

    drum = makeDrum(ax, R, L);
    [balls, class] = fillDrum(ax, R, L, d, counts);
    reach = R - d(class)'/2;

    % Settle the mixed charge into a bed, then turn at a constant speed. The
    % drum angle is driven by a script, so it is the time integral of the
    % speed - two straight segments are enough for a constant rotation.
    dt = 0.01;
    settle = 3;
    omega = sqrt(froude*9.81/R);
    period = 2*pi/omega;
    turning = Options.Revolutions*period;
    phx.Script(drum, {"EulerAngles", [0 settle settle+turning]', ...
        [0 0 0; 0 0 0; 0 omega*turning 0]});

    sim = phx.Simulation([drum balls]);
    redraw = 10;
    sim.step(settle, settle/dt, redraw);

    % Sample the mean radius of every class every tenth of a second
    interval = 0.1;
    nSamples = round(turning/interval);
    out.Revolutions = zeros(nSamples + 1, 1);
    out.Radius = zeros(nSamples + 1, nClass);
    out.Radius(1, :) = orbits(balls, class, reach, nClass);

    for k = 1:nSamples
        sim.step(interval, interval/dt, redraw);
        out.Revolutions(k + 1) = k*interval/period;
        out.Radius(k + 1, :) = orbits(balls, class, reach, nClass);
    end

    out.Diameters = d;
    out.Spread = out.Radius(:, end) - out.Radius(:, 1);
    delete(sim);

    last = out.Revolutions > out.Revolutions(end) - 2;
    fprintf("mean radius, mixed:  %s\n", num2str(out.Radius(1, :), "%8.3f"));
    fprintf("mean radius, sorted: %s\n", num2str(mean(out.Radius(last, :), 1), "%8.3f"));
    fprintf("spread coarsest - finest: %+.3f mixed -> %+.3f sorted\n", ...
        out.Spread(1), mean(out.Spread(last)));

    % Plot the history of the class radii
    figure;
    colors = classColors(nClass);
    hold on
    for c = 1:nClass
        plot(out.Revolutions, out.Radius(:, c), "LineWidth", 1.5, "Color", colors(c, :), ...
            "DisplayName", sprintf("d = %.2f", d(c)));
    end
    grid on; xlabel("drum revolutions"); ylabel("mean orbit radius / reachable radius");
    legend("Location", "east"); title("Size segregation in a rotating drum");
end

function parts = makeDrum(ax, R, L)
% Barrel with a closed back wall, plus a separate cover over the front. The
% cover is invisible, so it still stops the balls but leaves the inside of the
% drum in plain view - a body wears one shape, so this needs two bodies.
    a = L/2;
    t = 0.25;

    % Closed profile [axial radius], traced from the axis outwards along the
    % back wall, around the outer barrel and back along the inside. The
    % direction matters for shading: this way the surfaces facing into the
    % cavity are the lit ones, which is what the camera looks at.
    profile = [-a 0; -a-t 0; -a-t R+t; a R+t; a R; -a R; -a 0];

    barrel = phx.Body(ax, "Type", "kinematic", "Friction", [0.9 0 0], ...
        "Shape", phx.shape.Revolution("Axis", "y", "Profile", profile, ...
            "Segments", 72, "Envelope", "concave", ...
            "Color", 0.3, "Style", "flat", "Texture", "checker", "TextureBlend", 0.2));

    cover = phx.Body(ax, "Type", "kinematic", "Position", [0 a+t/2 0], ...
        "Friction", [0.9 0 0], "Visible", false, ...
        "Shape", {"Cylinder", "Axis", "y", "Diameter", 2*(R+t), "Height", t});

    parts = [barrel cover];
end

function [balls, class] = fillDrum(ax, R, L, d, counts)
% Place the charge by rejection sampling over the whole cavity, so it starts
% well mixed - the precondition for claiming that the sorting came from the
% rotation and not from the way the drum was filled.
    class = repelem(1:numel(d), counts);
    class = class(randperm(numel(class)))';
    dBall = d(class)';

    P = zeros(numel(dBall), 3);
    for i = 1:numel(dBall)
        for attempt = 1:4000
            radius = (R - dBall(i)/2 - 0.02)*sqrt(rand);
            phi = rand*2*pi;
            P(i, :) = [radius*cos(phi), (rand - 0.5)*(L - dBall(i) - 0.02), radius*sin(phi)];
            if i == 1 || all(vecnorm(P(1:i-1, :) - P(i, :), 2, 2) - (dBall(1:i-1) + dBall(i))/2 > 0.01)
                break
            end
        end
    end

    colors = classColors(numel(d));
    balls = phx.Body.empty;
    for i = 1:numel(dBall)
        balls(i) = phx.Body(ax, "Position", P(i, :), "Friction", [0.3 0.01 0.005], ...
            "Color", colors(class(i), :), "Shape", {"Sphere", "Diameter", dBall(i)});
    end
end

function colors = classColors(nClass)
% Fine to coarse along a fixed orange - green - blue ramp
    stops = [0.90 0.35 0.15; 0.25 0.70 0.30; 0.20 0.45 0.80];
    if nClass == 1
        colors = stops(1, :);
    else
        colors = interp1(linspace(0, 1, 3), stops, linspace(0, 1, nClass));
    end
end

function r = orbits(balls, class, reach, nClass)
    P = reshape([balls.Position], 3, [])';
    normalised = hypot(P(:, 1), P(:, 3))./reach;
    r = arrayfun(@(c) mean(normalised(class == c)), 1:nClass);
end
