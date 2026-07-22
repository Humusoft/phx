function phxex_tackle(mCrate)
% PHXEX_TACKLE Block and tackle: lifting a heavy crate with a 4:1 hoist
%
% A heavy crate hangs under a gantry on a single phx.Rope routed as a
% block and tackle: anchored to the beam, woven twice through the moving
% block on the crate and over the sheaves of the fixed block, and led
% to a winch drum. The routing points are spread sideways so that all
% four load-bearing rope parts are clearly visible.
%
% The pulleys are ideal, so one tension acts along the whole rope - and
% the mechanical advantage emerges by itself: four rope parts carry the
% crate, the rope tension is a quarter of its weight, and the winch has
% to reel in four meters of rope for every meter of lift. The winch is
% driven by the Displacement property of the rope (negative = reeling
% in); the rope is colored by its tension.
%
% The demo hoists the crate, holds it, and lowers it back. The report
% compares the measured tension and winching ratio with the theoretical
% 4:1, and the plots show the crate height versus the reeled rope and
% the tension history.
%
% Input Arguments:
%     mCrate - crate mass in kg (default 230)
%
% Example:
%     phxex_tackle          % default crate
%     phxex_tackle(500)     % heavier crate, same quarter-of-weight tension

%   Copyright 2026 HUMUSOFT s.r.o.

    arguments
        mCrate (1, 1) double {mustBePositive} = 230
    end

    grav = 9.81;
    kRope = 20000;         % rope stiffness (N/m)
    cRope = 500;           % rope damping (N*s/m)
    rDrum = 0.18;          % winch drum radius (for the drum rotation)

    % Resources directory
    resdir = fullfile(fileparts(mfilename("fullpath")), "res", " ");

    % Figure setup
    figure(1);
    [viewer, ax] = phx.extra.Viewer("clear", "DefaultCameraTarget", [0.5 0 1.8], ...
        "DefaultCameraPosition", [4 -7 3]);

    % Floor, gantry posts and the crossbeam
    phx.Body(ax, "Type", "static", "Position", [0.5 0 -0.5], ...
        "Shape", {"Box", "Size", [10 8 1], "Color", [1 1 1]});
    for x = [-1.6 1.6]
        phx.Body(ax, "Type", "static", "Position", [x 0 1.95], ...
            "Shape", {"Box", "Size", [0.25 0.25 3.9], "Color", [0.55 0.55 0.6]});
    end
    beam = phx.Body(ax, "Type", "static", "Position", [0 0 4.0], ...
        "Shape", {"Box", "Size", [3.5 0.3 0.3], "Color", [0.55 0.55 0.6]});

    % The crate with the moving block on top (two routing points)
    crate = phx.Body(ax, "Position", [-1 0 0.3], ...
        "Shape", {"Box", "Size", [0.8 0.6 0.6], "Color", [0.7 0.5 0.3]}, ...
        "Friction", [0.4 0.05 0.05]);
    crate.Mass = mCrate;
    crate.Inertia = mCrate*[0.06 0.08 0.08];

    % The winch drum (kinematic, so it can visually rotate while reeling)
    drum = phx.Body(ax, "Type", "kinematic", "Position", [2.6 0 0.8], ...
        "Shape", {"Cylinder", "Diameter", 2*rDrum, "Height", 0.4, "Axis", "y", ...
        "Color", [0.4 0.4 0.45], "Material", "metal", "Texture", resdir+"checker4.png", "TextureBlend", 0.5});

    % The rope: beam anchor -> moving block -> fixed sheave -> moving
    % block -> fixed sheave -> winch; the crate appears twice in the
    % chain, the beam three times, each with its own routing point
    rope = phx.Rope([beam crate beam crate beam drum], ...
        "Points", [-0.3 0 -0.2; -0.15 0 0.35; 0 0 -0.25; 0.15 0 0.35; 0.3 0 -0.25; 0 0 0], ...
        "Stiffness", kRope, "Damping", cRope, ...
        "Colormap", "heat", "ColorRange", [0 0.5*mCrate*grav]);

    viewer.displayText("Settling...");

    sim = phx.Simulation(ax);
    dt = 0.005;
    subSteps = 10;

    % Winch profile: settle, hoist 4 m of rope, hold, lower 3 m back
    profile = @(t) -4.0*min(max((t - 2)/10, 0), 1) + 3.0*min(max((t - 13.5)/6, 0), 1);
    tEnd = 21;
    log = struct("t", [], "z", [], "disp", [], "F", []);
    t = 0;
    while t < tEnd
        for s = 1:subSteps
            rope.Displacement = profile(t);
            sim.step(dt, 1, 1);
            t = t + dt;
        end

        % Rotate the winch drum according to the reeled rope
        drum.EulerAngles = [0, -rope.Displacement/rDrum, 0];

        log.t(end + 1) = t;
        log.z(end + 1) = crate.Position(3);
        log.disp(end + 1) = rope.Displacement;
        log.F(end + 1) = rope.Force;
        viewer.displayText(sprintf("Crate %.2f m   rope reeled %.2f m   tension %4.0f N (weight %.0f N)", ...
            crate.Position(3), -rope.Displacement, rope.Force, mCrate*grav));
    end
    delete(sim);

    % Measured mechanical advantage during the hoist phase
    id1 = find(log.t > 3, 1);
    id2 = find(log.t > 11.5, 1);
    ratio = (log.disp(id1) - log.disp(id2))/(log.z(id2) - log.z(id1));
    idHold = log.t > 12.2 & log.t < 13.4;
    fprintf("Rope tension while holding: %.0f N = weight/%.2f (weight %.0f N).\n", ...
        mean(log.F(idHold)), mCrate*grav/mean(log.F(idHold)), mCrate*grav);
    fprintf("Winched %.2f m of rope per meter of lift (theory 4).\n", ratio);

    % Hoist kinematics and rope tension plots
    figure(2);
    subplot(2, 1, 1);
    yyaxis left
    plot(log.t, log.z, "LineWidth", 1.5); ylabel("crate height [m]");
    yyaxis right
    plot(log.t, -log.disp, "LineWidth", 1.5); ylabel("rope reeled in [m]");
    grid on; xlabel("time [s]");
    title(sprintf("4:1 hoist: %.2f m of rope per meter of lift", ratio));
    subplot(2, 1, 2);
    plot(log.t, log.F, "LineWidth", 1.5); hold on
    yline(mCrate*grav, "--", "crate weight");
    yline(mCrate*grav/4, ":", "weight / 4");
    grid on; xlabel("time [s]"); ylabel("rope tension [N]");

end