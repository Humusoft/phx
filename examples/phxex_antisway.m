function phxex_antisway
% PHXEX_ANTISWAY Anti-sway crane control: input shaping vs a direct move
%
% Two identical overhead cranes carry a pile of rocks on a flat platform.
% The platform hangs by its corners on four sling ropes from a common
% hook (a separate body), and the hook hangs on a single rope under a
% kinematic trolley. Both trolleys travel the same 4 m along the runway:
% the left one follows the raw trapezoidal motion profile, the right one
% follows the same profile passed through a ZV input shaper - the average
% of two copies of the command, the second delayed by half the pendulum
% swing period, so the swing excited by the first copy is cancelled by
% the second.
%
% The direct move sets the whole hanging assembly swinging; the platform
% tilts with the swing and sheds its cargo. The shaped move arrives only
% half a swing period later, yet the load barely stirs. The report
% prints the residual sway and the number of rocks still on board, and
% the plots compare the sway and platform tilt histories.
%
% See also phx.Rope, phx.Script, phx.Simulation

%   Copyright 2026 HUMUSOFT s.r.o.

    grav = 9.81;
    dist = 4;                    % trolley travel (m)
    vMax = 2.4;                  % profile cruise speed (m/s)
    aMax = 6;                    % profile acceleration (m/s^2)
    zTrolley = 6.35;             % trolley height (m)
    zPlate = 2.5;                % platform centre height (m)
    zHook = zPlate + 1.35;       % hook height (m)

    % Half the swing period of the hanging assembly (trolley to load) -
    % the only model knowledge the ZV shaper needs
    tHalf = pi*sqrt((zTrolley - (zPlate + 0.2))/grav);

    % Trapezoidal profile timing and the two trolley commands
    tSettle = 2.5;               % let the pile and the ropes settle first
    tAcc = vMax/aMax;
    tMove = 2*tAcc + (dist - vMax^2/aMax)/vMax;
    xRef = @(t) trapezoid(t - tSettle, dist, vMax, aMax);
    xDirect = @(t) xRef(t);
    xShaped = @(t) (xRef(t) + xRef(t - tHalf))/2;
    tEnd = tSettle + tMove + tHalf + 4;

    % Two identical crane scenes side by side
    figure(1); clf;
    tl = tiledlayout(1, 2, "TileSpacing", "compact", "Padding", "compact");
    axA = nexttile(tl); title(axA, "Direct move");
    axB = nexttile(tl); title(axB, "Input-shaped move");
    A = buildCrane(axA);
    B = buildCrane(axB);

    simA = phx.Simulation(axA, "EngineSettings", ...
        phx.engine.BulletSettings("AutoActivated", false));
    simB = phx.Simulation(axB, "EngineSettings", ...
        phx.engine.BulletSettings("AutoActivated", false));

    dt = 0.005;
    subSteps = 8;
    log = struct("t", [], "offA", [], "offB", [], "tiltA", [], "tiltB", []);
    nStart = NaN;
    t = 0;
    while t < tEnd
        for s = 1:subSteps
            A.trolley.Position = [xDirect(t) 0 zTrolley];
            B.trolley.Position = [xShaped(t) 0 zTrolley];
            redraw = mod(s, 5) == 0;
            simA.step(dt, 1, redraw);
            simB.step(dt, 1, redraw);
            t = t + dt;
        end

        % Rocks on board at the end of the settle phase (the baseline)
        if isnan(nStart) && t >= tSettle
            nStart = min(countRocks(A), countRocks(B));
        end

        log.t(end + 1) = t;
        log.offA(end + 1) = A.hook.Position(1) - A.trolley.Position(1);
        log.offB(end + 1) = B.hook.Position(1) - B.trolley.Position(1);
        log.tiltA(end + 1) = tilt(A.plate);
        log.tiltB(end + 1) = tilt(B.plate);
        title(axA, sprintf("Direct move   sway %5.2f m", log.offA(end)));
        title(axB, sprintf("Input-shaped move   sway %5.2f m", log.offB(end)));
    end

    % Residual sway after both trolleys have stopped
    idRes = log.t > tSettle + tMove + tHalf + 1;
    nA = countRocks(A);
    nB = countRocks(B);
    delete(simA);
    delete(simB);
    fprintf("Direct move:       residual sway %.3f m, kept %d of %d rocks.\n", ...
        max(abs(log.offA(idRes))), nA, nStart);
    fprintf("Input-shaped move: residual sway %.3f m, kept %d of %d rocks.\n", ...
        max(abs(log.offB(idRes))), nB, nStart);

    % Sway and platform tilt histories
    figure(2);
    subplot(2, 1, 1);
    plot(log.t, log.offA, log.t, log.offB, "LineWidth", 1.5);
    grid on; ylabel("hook sway [m]");
    legend("direct", "input-shaped");
    title(sprintf("ZV input shaping: the same move, %.1f s later, without the swing", tHalf));
    subplot(2, 1, 2);
    plot(log.t, log.tiltA, log.t, log.tiltB, "LineWidth", 1.5);
    grid on; xlabel("time [s]"); ylabel("platform tilt [deg]");
    legend("direct", "input-shaped");

    function S = buildCrane(ax)
    % Build one crane scene into the given axes and return its bodies

        view(ax, -15, 8); axis(ax, "equal"); grid(ax, "on");
        camlight(ax, "headlight");
        axis(ax, [-4 dist + 4, -3 3, -0.5 7.5]);

        % Floor and the runway (two posts and a rail beam)
        phx.Body(ax, "Type", "static", "Position", [dist/2 0 -0.2], ...
            "Shape", {"Box", "Size", [dist + 8 6 0.4], "Color", [1 1 1]});
        for x = dist/2 + (dist/2 + 3)*[-1 1]
            phx.Body(ax, "Type", "static", "Position", [x 0 3.3], ...
                "Shape", {"Box", "Size", [0.3 0.3 6.6], "Color", [0.55 0.55 0.6]});
        end
        phx.Body(ax, "Type", "static", "Position", [dist/2 0 6.7], ...
            "Shape", {"Box", "Size", [dist + 6.3 0.3 0.2], "Color", [0.55 0.55 0.6]});

        % The kinematic trolley riding under the rail
        S.trolley = phx.Body(ax, "Type", "kinematic", "Position", [0 0 zTrolley], ...
            "Shape", {"Box", "Size", [0.7 0.5 0.3], "Color", [0.75 0.25 0.2]});

        % The hook - a separate heavy body all five ropes meet at
        S.hook = phx.Body(ax, "Position", [0 0 zHook], ...
            "Shape", {"Sphere", "Radius", 0.12, "Density", 7800, ...
            "Color", 0.3, "Material", "metal"});

        % The platform hanging by its corners
        S.plate = phx.Body(ax, "Position", [0 0 zPlate], ...
            "Shape", {"Box", "Size", [1.8 1.8 0.08], "Density", 700, ...
            "Style", "edged", "Color", [0.72 0.55 0.35]}, ...
            "Friction", [0.7 0 0]);

        % Main rope trolley -> hook and four corner slings hook -> platform
        phx.Rope([S.trolley S.hook], "Points", [0 0 -0.15; 0 0 0.1], ...
            "Stiffness", 2e5, "Damping", 2e3, ...
            "Colormap", "heat", "ColorRange", [0 10000]);
        for c = [-1 -1; -1 1; 1 1; 1 -1]'
            phx.Rope([S.hook S.plate], "Points", [0 0 -0.05; 0.85*c' 0.04], ...
                "Stiffness", 5e4, "Damping", 500, ...
                "Colormap", "heat", "ColorRange", [0 4000]);
        end

        % The cargo: a two-layer pile of rocks dropped onto the platform
        rng(0);   % reproducible rock shapes
        pile = [allpairs([-0.5 0 0.5], [-0.5 0 0.5]), zeros(9, 1) + 0.26; ...
                allpairs([-0.25 0.25], [-0.25 0.25]), zeros(4, 1) + 0.64];
        S.rocks = phx.Body.empty;
        for k = 1:size(pile, 1)
            S.rocks(k) = phx.Body(ax, "Position", pile(k, :) + [0 0 zPlate], ...
                "Shape", {"Rock", "Radius", 0.2, "Color", (rand + [1 0.5 0])/2}, ...
                "Friction", [0.4 0.1 0.1]);
        end
    end

    function n = countRocks(S)
    % Number of rocks still resting on the platform
        n = 0;
        pc = S.plate.Position;
        for r = S.rocks
            p = r.Position;
            n = n + (p(3) > pc(3) && all(abs(p(1:2) - pc(1:2)) < 1.1));
        end
    end
end

function x = trapezoid(t, dist, vMax, aMax)
% Position on a trapezoidal velocity profile (rest-to-rest move by dist)
    tAcc = vMax/aMax;
    tCruise = (dist - vMax^2/aMax)/vMax;
    tTotal = 2*tAcc + tCruise;
    t = min(max(t, 0), tTotal);
    if t < tAcc
        x = aMax*t^2/2;
    elseif t < tAcc + tCruise
        x = vMax^2/(2*aMax) + vMax*(t - tAcc);
    else
        x = dist - aMax*(tTotal - t)^2/2;
    end
end

function d = tilt(body)
% Tilt of a body from the horizontal (deg)
    R = body.Orientation;
    d = acosd(min(R(3, 3), 1));
end

function p = allpairs(x, y)
% All [x y] combinations of the two vectors, one pair per row
    [X, Y] = meshgrid(x, y);
    p = [X(:) Y(:)];
end
