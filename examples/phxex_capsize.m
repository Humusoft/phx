function phxex_capsize(duration, maxAmplitude)
% PHXEX_CAPSIZE Deck cargo in growing seas - when do the crates go overboard?
%
% A small boat carries four crates on an open deck. The hull is a solid
% phx.shape.Extrusion: a chined boat cross-section swept along the length,
% with the per-point Scale tapering the beam toward a pointed bow and a
% narrower stern. The crates are not lashed down - they are held only by
% friction, and each crate has a different friction coefficient,
% representing a different quality of securing (red = slippery, green =
% grippy). The sea state slowly builds up: the phx.Buoyancy LevelFunction
% is a small directional wave spectrum (a dominant beam sea, a long swell
% and two short chop components, all deep-water dispersed) whose overall
% amplitude ramps from calm to maxAmplitude over the run.
%
% The buoyancy element gives the hull a physical righting moment (the
% center of buoyancy shifts as the hull heels), so the boat rolls with
% the wave slope. Once the roll exceeds the friction angle of a crate, it
% starts to slide, which shifts the load and heels the barge further -
% the crates depart one by one, in the order of their friction. The demo
% reports the time and wave amplitude at which each crate went overboard,
% i.e. the sea state each level of securing can survive. This coupled
% roll <-> sliding-cargo feedback needs contacts, friction and buoyancy
% in one simulation - none of it can be read from a static formula.
%
% Note the hull is modeled as a solid shape: the buoyancy sampling sees
% only the outer mesh, so a hollow hull would not displace any extra
% water anyway (see the phx.Buoyancy limitations).
%
% Input Arguments:
%     duration     - simulated time in seconds (default 60)
%     maxAmplitude - wave amplitude reached at the end of the run (default 0.35)
%
% Example:
%     phxex_capsize            % default run
%     phxex_capsize(40, 0.5)   % faster, steeper ramp

%   Copyright 2026 HUMUSOFT s.r.o.

    arguments
        duration (1, 1) double {mustBePositive} = 60
        maxAmplitude (1, 1) double {mustBePositive} = 0.35
    end

    % Hull: a chined boat cross-section (closed counterclockwise profile
    % in the beam/depth plane) extruded along the x axis; the per-point
    % Scale narrows the beam toward the pointed bow and the stern
    beam = 1.6;
    depth = 0.7;
    deckTop = depth/2;
    profile = [0 -1; 0.68 -0.71; 1 0.29; 1 1; -1 1; -1 0.29; -0.68 -0.71; 0 -1] ...
        .*[beam/2, depth/2];
    % Long parallel midbody so that all crates stand on equally wide deck
    spineX = [-2.7 -2.3 -1.4 1.2 2.1 2.6 2.9]';   % stern ... bow
    beamScale = [0.5 0.85 1 1 0.8 0.45 0.12]';
    hullShape = phx.shape.Extrusion("Spine", [spineX zeros(numel(spineX), 2)], ...
        "Profile", profile, "Scale", [beamScale ones(size(beamScale))], ...
        "Density", 500, "Style", "flat", "Color", [0.45 0.5 0.6]);

    % Crates: same size and weight, only the deck friction differs
    crateSize = 0.5;
    crateMu = [0.15 0.3 0.45 0.6];
    crateX = [-1.15 -0.5 0.15 0.8];               % on the full-beam deck part
    crateColor = [1 0.3 0.25; 1 0.65 0.2; 0.85 0.85 0.3; 0.4 0.8 0.35];

    % Directional sea: a dominant beam-sea wave plus a long swell and two
    % shorter chop components from other directions, each obeying the
    % deep-water dispersion w = sqrt(g*k). The weights sum to one, so the
    % linearly ramping amplitude is the maximum possible elevation; fixed
    % weights and phases keep the run deterministic.
    comp = [0.65   6.0    90   0.0;   % weight, wavelength, direction (deg from +x), phase
            0.15  12.0    70   1.3;
            0.12   3.5   115   4.0;
            0.08   2.2    55   2.1];
    kk = 2*pi./comp(:, 2);
    kx = kk.*cosd(comp(:, 3));
    ky = kk.*sind(comp(:, 3));
    ww = sqrt(9.81*kk);
    ph = comp(:, 4);
    wt = comp(:, 1);
    rampRate = maxAmplitude/duration;
    amp = @(t) min(rampRate*t, maxAmplitude);
    wave = @(x, y, t) amp(t).*reshape( ...
        sum(wt.*sin(kx.*x(:)' + ky.*y(:)' - ww*t + ph), 1), size(x));

    % Resources directory
    resdir = fullfile(fileparts(mfilename("fullpath")), "res", " ");

    % Figure setup
    figure(1);
    [viewer, ax] = phx.extra.Viewer("clear", "DefaultCameraTarget", [0 0 0], ...
        "DefaultCameraPosition", [6 -7 3]);

    hull = phx.Body(ax, "Position", [0 0 0], "Shape", hullShape, ...
        "Friction", [1 0 0]);

    crates = phx.Body.empty;
    for i = 1:4
        crates(i) = phx.Body(ax, "Position", [crateX(i) 0 deckTop + crateSize/2], ...
            "Shape", {"Box", "Size", crateSize*[1 1 1], "Density", 400, ...
            "Style", "edged", "Color", crateColor(i, :), "Texture", resdir+"woodtile.jpg", "TextureBlend", 0.5}, ...
            "Friction", [crateMu(i) 0 0]);
    end

    % One sea for everything: the hull rolls on it and fallen crates float
    phx.Buoyancy([hull crates], "LevelFunction", wave, ...
        "LinearDamping", 400, "AngularDamping", 200, ...
        "SurfaceSize", [14 14], "SurfaceStep", 0.25);

    % Simulation - dt = 5 ms for the crate/deck contacts
    sim = phx.Simulation(ax);
    dt = 0.005;
    subSteps = 10;

    fallen = false(1, 4);
    fallTime = nan(1, 4);
    hist = struct("t", [], "roll", [], "amp", []);

    while sim.Time < duration && ~all(fallen)
        sim.step(dt*subSteps, subSteps, subSteps);
        t = sim.Time;

        % Roll angle of the hull (rotation about its longitudinal axis)
        eul = hull.EulerAngles;
        hist.t(end + 1) = t;
        hist.roll(end + 1) = eul(1);
        hist.amp(end + 1) = amp(t);

        % A crate has departed once it leaves the deck area in the hull
        % frame (slid over the side or dropped below the deck)
        for i = find(~fallen)
            rel = hull.Transform\[crates(i).Position 1]';
            if abs(rel(2)) > beam/2 + crateSize || rel(3) < 0
                fallen(i) = true;
                fallTime(i) = t;
                fprintf("Crate with friction %.2f went overboard at t = %.1f s (wave amplitude %.2f m).\n", ...
                    crateMu(i), t, amp(t));
            end
        end

        viewer.displayText(sprintf("t = %4.1f s   wave amplitude %.2f m   roll %5.1f deg   crates lost %d/4", ...
            t, amp(t), rad2deg(hist.roll(end)), nnz(fallen)));
        pause(0);
    end
    delete(sim);

    % Summary: the sea state each level of securing survived
    fprintf("\nSecuring quality vs survivable sea state:\n");
    for i = 1:4
        if fallen(i)
            fprintf("  friction %.2f: lost at amplitude %.2f m\n", crateMu(i), amp(fallTime(i)));
        else
            fprintf("  friction %.2f: survived up to amplitude %.2f m\n", crateMu(i), amp(sim.Time));
        end
    end

    % Roll history with the growing sea and the overboard events
    figure(2);
    yyaxis left
    plot(hist.t, rad2deg(hist.roll), "LineWidth", 1.2);
    ylabel("hull roll [deg]");
    yyaxis right
    plot(hist.t, hist.amp, "--", "LineWidth", 1.2);
    ylabel("wave amplitude [m]");
    hold on
    for i = find(fallen)
        xline(fallTime(i), "-k", sprintf("\\mu = %.2f", crateMu(i)), ...
            "LabelVerticalAlignment", "bottom");
    end
    grid on; xlabel("time [s]");
    title("Crates go overboard one by one as the sea builds up");

end
