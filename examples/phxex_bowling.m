function out = phxex_bowling(Options)
% PHXEX_BOWLING One visible bowling throw, then a headless map of throws
%
% A regulation lane, a 6.8 kg ball and ten pins. The demo runs the same scene
% twice over, in two different roles:
%
%   1. One throw is simulated with rendering, so the roll, the hook and the
%      pin action can be watched.
%   2. The pins are stood up again and the scene becomes a pure solver: a
%      grid of releases (aim across the lane x ball revolutions) is stepped
%      without rendering, and every path is drawn into the scene as it
%      finishes, coloured by the number of pins it knocked down. The result is
%      a fan of trajectories over the lane: a map of what the release
%      parameters are worth.
%
% The hook is not scripted. The lane is built from segments with decreasing
% friction (an oil pattern: slick at the front, dry at the back) and the ball
% is released with side roll, so the sideways friction force bends the path
% on its own.
%
% Input Arguments:
%     Throws  - number of headless throws in the map (default 100)
%     Aim     - release positions across the lane, [min max] (default [-0.1 0.55])
%     Revs    - side roll of the ball in rad/s, [min max] (default [0 14])
%     Speed   - release speed in m/s (default 8)
%
% Example:
%     phxex_bowling
%     phxex_bowling(Throws = 400)
%
% See also phxex_optimize, phxex_determinism, phxex_galton

%   Copyright 2026 HUMUSOFT s.r.o.

    arguments
        Options.Throws (1, 1) double {mustBeInteger, mustBePositive} = 100
        Options.Aim (1, 2) double = [-0.1 0.55]
        Options.Revs (1, 2) double = [0 14]
        Options.Speed (1, 1) double = 8
    end

    % Lane and pin geometry (regulation: 60 ft to the head pin, 12 in pin spacing)
    L = struct("xHead", 18.29, "width", 1.0566, "deck", 1.2, ...
        "ballR", 0.1085, "ballM", 6.8, "pinH", 0.381, "pinM", 1.531, ...
        "pinSpace", 0.3048);

    % Oil pattern: friction of the lane segments from the foul line back
    L.edges = [0 13 15 L.xHead + L.deck];
    L.mu = [0.03 0.10 0.28];

    figure(1);
    [viewer, ax] = phx.extra.Viewer("clear", "DefaultCameraPosition", [21 0 1], "DefaultCameraTarget", [-100 0 -42], "Texture", "tiles");

    scene = makeScene(ax, L);
    sim = phx.Simulation(ax, "EngineSettings", phx.engine.BulletSettings("Margin", 0.001));

    cmap = turbo(11);

    % --- Part 1: one throw, rendered ------------------------------------
    % Plenty of side roll, so the path bends visibly - but it hooks straight
    % across the pocket, clips the corner and leaves eight pins standing
    renderedThrow(sim, scene, L, ax, cmap, 0.18, 14, Options.Speed, viewer, ...
        "One throw, with side roll...");

    % --- Part 2: the same scene as a solver -----------------------------
    % A grid of releases stepped without rendering. The bodies themselves are
    % never redrawn; the only thing that appears is one line per finished
    % throw, coloured by its result.
    % Move the camera behind the foul line and zoom in: from here the lane
    % runs away to the pins and the growing fan of paths fills the frame
    % ax.CameraPosition = [-2.2 -0.8 2.9];
    % ax.CameraTarget = [15 0 -0.9];
    % ax.CameraViewAngle = 20;

    n = round(sqrt(Options.Throws));
    aims = linspace(Options.Aim(1), Options.Aim(2), n);
    revs = linspace(Options.Revs(1), Options.Revs(2), n);

    out.Aim = aims;
    out.Revs = revs;
    out.Pins = zeros(n, n);
    out.Gutter = false(n, n);
    out.Paths = cell(n, n);

    done = 0;
    tStart = tic;
    for i = 1:n
        for j = 1:n
            r = throwOnce(sim, scene, L, aims(i), revs(j), Options.Speed, false);
            out.Pins(i, j) = r.pins;
            out.Gutter(i, j) = r.gutter;
            out.Paths{i, j} = r.path;

            % Draw the finished path over the lane, coloured by the result.
            line(ax, r.path(:, 1), r.path(:, 2), repmat(0.01, size(r.path, 1), 1), ...
                "Color", cmap(r.pins + 1, :), "LineWidth", 1.5 + 1.8*(r.pins == 10));

            done = done + 1;
            viewer.displayText(sprintf("Headless throws: %d / %d   (%.0f ms each)", ...
                done, n*n, 1000*toc(tStart)/done));
            drawnow;
        end
    end
    fprintf("%d headless throws in %.1f s (%.0f ms each), %d strikes.\n", ...
        n*n, toc(tStart), 1000*toc(tStart)/(n*n), nnz(out.Pins == 10));

    % Stand the pins up again and refresh the view under the fan of paths
    resetScene(scene, L, mean(aims), 0, Options.Speed);
    sim.step(0.05, 5, 1);
    pause(1);

    % --- Part 3: the release the map found, thrown for real --------------
    best = renderedThrow(sim, scene, L, ax, cmap, 0.435, 10.2, Options.Speed, ...
        viewer, "And the line the map found...");
    viewer.displayText(sprintf("%d throws, %d pins", n*n, best.pins));
    delete(sim);

    % --- The map as a plain plot ----------------------------------------
    figure(2); clf;
    tiledlayout(2, 1, "TileSpacing", "compact");

    % Top tile: the paths themselves. The lane is 18 m long and 1 m wide, so
    % the across-lane axis has to be stretched for anything to be visible -
    % which is exactly how the charts bowlers use are drawn.
    nexttile;
    hold on
    for i = 1:n
        for j = 1:n
            p = out.Paths{i, j};
            plot(p(:, 1), p(:, 2), "Color", cmap(out.Pins(i, j) + 1, :), "LineWidth", 0.8 + 1.2*(out.Pins(i, j) == 10));
        end
    end
    plot(L.xHead + [0 0], L.width/2*[-1 1], "k:");
    plot([0 L.xHead + L.deck], L.width/2*[1 1], "k-", "LineWidth", 1);
    plot([0 L.xHead + L.deck], L.width/2*[-1 -1], "k-", "LineWidth", 1);
    xlim([0 L.xHead + L.deck]); ylim(0.62*[-1 1]);
    xlabel("distance down the lane [m]"); ylabel("across [m]");
    title(sprintf("%d headless throws, coloured by pins down (grey = off the lane)", n*n));

    % Bottom tile: the same results as a map over the release parameters
    nexttile;
    shown = out.Pins;
    shown(out.Gutter) = nan;
    imagesc(revs, aims, shown, "AlphaData", ~isnan(shown));
    set(gca, "Color", cmap(1, :));
    axis xy
    colormap(gca, cmap); clim([-0.5 10.5]);
    xlabel("side roll at release [rad/s]"); ylabel("release position across the lane [m]");
    title("pins down over the release parameters");
    colorbar("Ticks", 0:2:10);

end

% ========================================================================
function scene = makeScene(ax, L)
% Lane segments, ball and pins. The pin profile is shifted so that the body
% origin (which is the centre of mass) sits low, as it does in a real pin.

    lane = phx.Body.empty;
    for k = 1:numel(L.mu)
        len = L.edges(k+1) - L.edges(k);
        lane(k) = phx.Body(ax, "Type", "static", "Position", [L.edges(k) + len/2, 0, -0.05], ...
            "Shape", {"Box", "Size", [len L.width 0.1], "Color", [0.78 0.62 0.38] + 0.06*(k - 2)}, ...
            "Friction", [L.mu(k) 0 0], "Restitution", 0.1); %#ok<AGROW>
    end

    % Gutters, so a missed throw leaves the lane instead of sliding along it
    for s = [-1 1]
        phx.Body(ax, "Type", "static", "Position", [(L.xHead + L.deck)/2, s*(L.width/2 + 0.12), -0.16], ...
            "Shape", {"Box", "Size", [L.xHead + L.deck, 0.24, 0.12], "Color", 0.35});
    end

    ball = phx.Body(ax, "Position", [0.3 0 L.ballR + 0.001], ...
        "Shape", {"Globe", "Radius", L.ballR, "Color", [0.15 0.2 0.45], "Texture", "checker", "TextureBlend", 0.3}, ...
        "Friction", [1.0 0.002 0.001], "Restitution", 0.6);
    ball.Mass = L.ballM;
    ball.Inertia = 2/5*L.ballM*L.ballR^2*[1 1 1];

    cg = 0.14;                                  % centre of mass above the base
    prof = [-cg 0; -cg 0.0254; -cg+0.02 0.0290; -cg+0.09 0.0605; ...
            -cg+0.15 0.0530; -cg+0.24 0.0290; -cg+0.30 0.0330; ...
            -cg+0.355 0.0230; L.pinH-cg 0.011; L.pinH-cg 0];
    dx = L.pinSpace*cosd(30); dy = L.pinSpace/2;
    layout = [0 0; dx -dy; dx dy; 2*dx -2*dy; 2*dx 0; 2*dx 2*dy; ...
              3*dx -3*dy; 3*dx -dy; 3*dx dy; 3*dx 3*dy];

    pins = phx.Body.empty;
    shp = phx.shape.Revolution("Profile", prof, "Segments", 24, "Envelope", "convex", "Color", 1);
    for k = 1:10
        pins(k) = phx.Body(ax, "Position", [L.xHead + layout(k, 1), layout(k, 2), cg + 0.002], ...
            "Shape", shp, "Friction", [0.08 0.001 0.001], "Restitution", 0.7, "Mass", L.pinM, "Inertia", [0.0095 0.0095 0.0018]); %#ok<AGROW>
    end

    % The full rack: every throw starts by restoring this state
    storeState([pins ball], "rack");

    scene = struct("lane", lane, "ball", ball, "pins", pins, "pin0", vertcat(pins.Position));
end

% ========================================================================
function r = renderedThrow(sim, scene, L, ax, cmap, aim, sideRoll, speed, viewer, caption)
% One throw with rendering, its path left behind as a thick line so that the
% two watched throws stand out among the hundred computed ones

    viewer.displayText(caption);
    trace = phx.Trace(scene.ball, "TracePoints", 400, "Overlay", true, "Color", [1 0.85 0.2]);
    r = throwOnce(sim, scene, L, aim, sideRoll, speed, true);
    delete(trace);

    line(ax, r.path(:, 1), r.path(:, 2), repmat(0.01, size(r.path, 1), 1), ...
        "Color", cmap(r.pins + 1, :), "LineWidth", 4);

    viewer.displayText(sprintf("Pins down: %d", r.pins));
    fprintf("Rendered throw (aim %.3f m, side roll %.1f rad/s): %d pins down.\n", aim, sideRoll, r.pins);
    pause(1);
end

% ========================================================================
function r = throwOnce(sim, scene, L, aim, sideRoll, speed, render)
% One throw on the existing scene. With render = false the scene is stepped
% without any redraw, which is what makes a hundred throws affordable.

    resetScene(scene, L, aim, sideRoll, speed);

    dt = 0.02; nSub = 10;                       % 2 ms substeps
    nStep = round(3.5/dt);
    path = nan(nStep, 2);
    redraw = -1;
    if render
        redraw = 1;
    end

    gutter = false;
    for k = 1:nStep
        sim.step(dt, nSub, redraw);
        p = scene.ball.Position;
        path(k, :) = p(1:2);
        if abs(p(2)) > L.width/2
            gutter = true;                      % the ball has left the lane
            break
        end
        if p(1) > L.xHead + L.deck
            break                               % off the back of the deck
        end
    end
    path = path(~isnan(path(:, 1)), :);

    down = 0;
    for k = 1:10
        pos = scene.pins(k).Position;
        up = scene.pins(k).Orientation(:, 3);
        if acosd(min(1, max(-1, up(3)))) > 35 || pos(3) < 0.10 || ...
                norm(pos(1:2) - scene.pin0(k, 1:2)) > 0.20
            down = down + 1;
        end
    end

    r = struct("pins", down, "path", path, "gutter", gutter);
end

% ========================================================================
function resetScene(scene, L, aim, sideRoll, speed)
% Stand the pins up on their spots and put the ball back on the foul line.

    restoreState([scene.pins scene.ball], "rack");

    scene.ball.Position = [0.3 aim L.ballR + 0.001];
    scene.ball.LinearVelocity = [speed 0 0];
    % Forward roll about +y (70 % of the rolling rate, the ball still skids)
    % plus side roll about the direction of travel, which is what hooks it
    scene.ball.AngularVelocity = [sideRoll, 0.7*speed/L.ballR, 0];
end