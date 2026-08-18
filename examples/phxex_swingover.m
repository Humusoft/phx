function phxex_swingover
% PHXEX_SWINGOVER Swing a load over a row of walls, chamber by chamber
%
% A trolley rides a rail on a phx.PrismaticJoint and carries a rigid arm with
% a heavy ball on a phx.RevoluteJoint, driven only by a horizontal force. Four
% walls of dry-stacked bricks stand in its way, all taller than the hanging
% ball, and the load starts walled into the first chamber. To reach the last
% one it has to be thrown over the two walls in between: wind the swing up in
% the room that is available, throw the load over, catch it on the other side.
%
% Build walls stands a wall on each of the four foundation blocks, which you
% can drag and turn with the mouse first (the drag tools of phx.extra.Viewer,
% no code needed) - tight chambers are pointed out but built anyway. Simulate
% hops over the inner walls one after another, retrying a hop that fails.
% Return then drives plainly home through whatever is in the way.
%
% See also phx.extra.Viewer, phx.assembly.wall, phx.PrismaticJoint,
% phx.RevoluteJoint

%   Copyright 2026 HUMUSOFT s.r.o.

    % --- Mechanism ------------------------------------------------------
    zPivot = 1.75;               % hinge height above the ground (m)
    armLen = 1.2;                % hinge to ball centre (m)
    rBall = 0.2;                 % ball radius (m)
    mBall = 25;                  % ball mass (kg)
    mTrolley = 8;                % trolley mass (kg)
    xStart = -2.8;               % home position of the trolley (m), chamber one
    xPost = [-4.8 4.8];          % the two posts the rail rests on (m)

    % --- Walls and their foundations ------------------------------------
    wallSize = [1.5 0.1 1.0];    % length across the travel, thickness, height
    zPlinth = 0.06;              % foundation block thickness (m)
    wallTop = zPlinth + wallSize(3);
    xPlinth = [-4.2 -1.4 1.4 4.2];   % the four blocks (2.7 m chambers)
    nWalls = numel(xPlinth);

    % --- Travel and control ---------------------------------------------
    % The wind up is one stroke, measured from the wall to be crossed: run
    % forward over dWind, brake hard, snap back over dBack to the launch point
    dLaunch = 1.4;               % launch point short of the wall (m)
    dWind = 0.9;                 % length of the wind up run (m)
    dBack = 0.35;                % backward snap stroke (m)
    thTrig = -20*pi/180;         % swing angle at which the snap starts
    vWind = 3.5; aWind = 10;     % wind up profile (m/s, m/s^2)
    vBack = 2.5; aBack = 20;     % backward snap profile - this is the whip
    fMax = 1000;                 % actuator force limit (N)
    fDash = 500;                 % thrust while crossing (N)
    vCruise = 4.0;               % speed cap while crossing (m/s)
    kTrack = [1500 800];         % tracking gains for a plain scheduled move
    margin = 0.15;               % how far past a wall the ball must be
    aim = -0.3;                  % catch target, short of the middle of a chamber
    tCatch = 8;                  % how long a catch is given (s)
    maxHops = 4;                 % two walls to cross, so two failed hops over
    dt = 0.01;                  % small step - the stacked walls need it

    % Room a chamber needs: the run up to wind up in, and about 2.5 m to land
    % in, because below the top of the walls the load sweeps its whole swing
    dRoom = dLaunch - dBack + dWind;
    dLand = 2.5;

    % Catch gains for the linearized trolley-pendulum (state
    % [x-xTarget; dx/dt; sin(angle); rate*cos(angle)]), weighted hard on
    % position so that the trolley and its swinging load stay inside a chamber
    K = [150 120 -90.3 -10.9];

    % Angle at which the ball just clears the top of a wall, and half the
    % period of the free swing (the shaping delay of a scheduled move)
    thClear = acos((zPivot - wallTop - rBall)/armLen);
    tShape = pi*sqrt(armLen/9.81);

    % --- Window ---------------------------------------------------------
    fig = clf(figure(1));
    set(fig, "Name", "PHX - swing the load through the walls", "CloseRequestFcn", @onClose);
    layout = uigridlayout(fig, [2 1], "RowHeight", {50, "1x"}, "Padding", 0);
    bar = uigridlayout(layout, [1 5], "ColumnWidth", {110, 110, 110, "1x", 110});
    btnBuild = uibutton(bar, "Text", "Build walls", "ButtonPushedFcn", @onBuild);
    btnRun = uibutton(bar, "Text", "Simulate", "ButtonPushedFcn", @onSimulate, "Enable", "off");
    btnBack = uibutton(bar, "Text", "Return", "ButtonPushedFcn", @onReturn, "Enable", "off");
    hint = uilabel(bar, "HorizontalAlignment", "center");
    uibutton(bar, "Text", "Reset", "ButtonPushedFcn", "phxex_swingover", "Enable", "on");

    ax = uiaxes(layout);
    % Wide angle from the side - a straight-on view of an 8 m row of walls
    % would leave them tiny
    viewer = phx.extra.Viewer(ax, "RestrictedNavigation", true, ...
        "DefaultCameraPosition", [-4.5 -8 3.2], "DefaultCameraTarget", [0 0 0.8]);

    % --- Scene ----------------------------------------------------------
    % Ground
    phx.Body(ax, "Type", "static", "Position", [0 0 -0.15], ...
        "Shape", {"Box", "Size", [14 8 0.3], "Color", 0.85, "Texture", "tiles"});

    % Rail beam on two posts, with the trolley hanging under the beam. Starting
    % and finishing inside the row keeps it short - the trolley never has to
    % reach beyond the outer walls.
    zRail = zPivot + 0.23;
    for xp = xPost
        phx.Body(ax, "Type", "static", "Position", [xp 0 (zRail - 0.07)/2], ...
            "Shape", {"Box", "Size", [0.14 0.14 zRail - 0.07], "Color", [0.45 0.47 0.5]});
    end
    rail = phx.Body(ax, "Type", "static", "Position", [mean(xPost) 0 zRail], ...
        "Shape", {"Box", "Size", [diff(xPost) + 0.3 0.14 0.14], "Color", [0.45 0.47 0.5]});
    trolley = phx.Body(ax, "Position", [xStart 0 zPivot + 0.08], "Mass", mTrolley, ...
        "Shape", {"Box", "Size", [0.3 0.2 0.16], "Color", [0.85 0.5 0.15]});
    phx.PrismaticJoint(rail, trolley, "PointA", [0 0 -0.15], ...
        "AxisA", [1 0 0], "AxisB", [1 0 0], "Visible", false);

    % Arm and ball, hanging still - the wind up needs no nudge to start from
    pivot = [xStart 0 zPivot];
    arm = phx.Body(ax, "Position", pivot - [0 0 armLen/2], ...
        "Shape", {"Cylinder", "Axis", "z", "Radius", 0.025, "Height", armLen, ...
                  "Color", [0.35 0.37 0.4], "Material", "metal"});
    ball = phx.Body(ax, "Position", pivot - [0 0 armLen], ...
        "Shape", {"Sphere", "Radius", rBall, "Color", 0.3, "Material", "metal"}, ...
        "Mass", mBall, "Inertia", 2/5*mBall*rBall^2);
    phx.Trace(ball, "TracePoints", 2400, "Color", [1 0.8 0.2], "Overlay", true);
    phx.RevoluteJoint(trolley, arm, "PointA", [0 0 -0.08], "PointB", [0 0 armLen/2], ...
        "AxisA", [0 1 0], "AxisB", [0 1 0], "Visible", false);
    phx.FixedJoint(arm, ball, "PointA", [0 0 -armLen/2], "Visible", false);

    % Only the foundations are meant to be picked up, so the mechanism ignores
    % double clicks instead of being selected (and made kinematic) by them
    set([trolley arm ball], "OnDoubleClickFcn", @(~, ~) []);

    % The foundation blocks stay static, and the viewer makes a selected body
    % kinematic only for the drag, so nothing moves until the walls are built
    plinth = phx.Body.empty;
    for b = 1:nWalls
        plinth(b) = phx.Body(ax, "Type", "static", "Name", "foundation" + b, ...
            "Position", [xPlinth(b) 0 zPlinth/2], "Shape", {"Box", ...
            "Size", [0.32 1.7 zPlinth], "Color", 1});
    end

    sim = [];                    % created when the walls are built
    bricks = phx.Body.empty;
    nBrick = 0;                  % bricks per wall
    xWall = [];                  % where the walls were built, left to right
    halfX = [];                  % half their thickness measured along x
    reach = [];                  % how close the ball centre may come to each
    wallR = zeros(3, 3, nWalls); % how each wall stands, and the centre of
    wallC = zeros(nWalls, 3);    % the box it fills (for measuring clearances)
    inner = 2:nWalls - 1;        % the walls that are crossed
    step = 0;
    running = false;
    closing = false;
    hint.Text = "Place the four foundation blocks (drag to move, middle button to turn), then press Build walls.";

    % =====================================================================
    function onBuild(~, ~)
        viewer.deselect;                          % restores the static type
        [xWall, order] = sort(arrayfun(@(p) p.Position(1), plinth));
        plinth = plinth(order);                   % left to right

        % A wall is built exactly as its block was left, turned blocks
        % included. The load passes over y = 0, so what it has to clear is the
        % thickness along x, which a turned wall stretches by 1/cos.
        turn = arrayfun(@(p) p.EulerAngles(3), plinth);
        halfX = min(wallSize(2)/2./max(abs(cos(turn)), 0.2), wallSize(1)/2);
        reach = rBall + halfX;                    % how close the ball may come
        chamber = diff(xWall) - halfX(1:end - 1) - halfX(2:end);
        if any(chamber <= 0)
            uialert(fig, "Two of the foundations overlap - the walls would be " + ...
                "built into each other. Move them apart.", "Build walls");
            return
        end

        for j = 1:nWalls
            plinth(j).OnDoubleClickFcn = @(~, ~) [];   % no moving them now
            % The wall stands on the top face of its block, its length running
            % across it (hence the quarter turn on top of the block pose)
            R = plinth(j).Orientation;
            base = plinth(j).Position + (R*[0; 0; zPlinth/2])';
            wallR(:, :, j) = R*[0 -1 0; 1 0 0; 0 0 1];
            wallC(j, :) = base + (wallR(:, :, j)*[0; 0; wallSize(3)/2])';
            bricks = [bricks phx.assembly.wall(ax, "Size", wallSize, ...
                "Rows", 6, "Columns", 5, "Color", [0.72 0.4 0.32], ...
                "Position", base, "Orientation", wallR(:, :, j))]; %#ok<AGROW>
        end
        nBrick = numel(bricks)/nWalls;

        sim = phx.Simulation(ax);
        busy(true);
        sim.step(0.5, 125, 5);                    % let the walls settle
        busy(false);
        if bailOut, return, end

        % The first chamber is only wound up in, the last only landed in, the
        % ones between do both. Chambers too tight for their job are named, but
        % nothing is refused - ending in the bricks is a fair answer.
        need = repmat(max(dLand, dRoom + 2*rBall), 1, nWalls - 1);
        need([1 end]) = [dRoom + 2*rBall, dLand];
        short = find(chamber < need);
        txt = "Chambers " + join(compose("%.2f", chamber), " / ") + " m. ";
        if isempty(short)
            hint.Text = txt + "Press Simulate.";
        else
            hint.Text = txt + sprintf("Chamber %s under the %s m it needs to be " + ...
                "wound up or landed in, so expect bricks. Press Simulate anyway.", ...
                join(string(short), ", "), join(compose("%.1f", need(short)), "/"));
        end
    end

    % =====================================================================
    function onSimulate(~, ~)
        busy(true);
        p0 = brickPositions;
        clear0 = inf(1, nWalls);                  % closest the ball ever came
        crossed = false(1, nWalls);
        hops = 0;
        while ~closing && hops < maxHops
            % the first wall still ahead of the load, outer ones excluded
            k = inner(find(xWall(inner) - reach(inner) > ball.Position(1), 1));
            if isempty(k)
                break                             % in the last chamber
            end
            hops = hops + 1;
            clear0 = min(clear0, hop(k, mean(xWall(k:k + 1)) + aim));
            crossed(k) = ball.Position(1) > xWall(k) + reach(k);
        end
        if bailOut, return, end

        sim.step(0.5, 125, 2);
        report(crossed, clear0, p0, hops);
        busy(false);
    end

    % ------------------------------------------------------------ one hop
    function c = hop(k, xTarget)
        % Wind the swing up in front of wall k, throw the load over it and
        % catch the swing on the far side. All of it is measured from that
        % wall, so a hop is the same maneuver wherever the walls stand.
        xBrake = xWall(k) - dLaunch + dBack;
        xWind = xBrake - dWind;
        x0 = trolley.Position(1);
        phase = 0;                    % 0 up, 1 wind up, 2 snap, 3 over, 4 catch
        tPhase = 0; t = 0;
        c = inf(1, nWalls);
        names = ["taking position", "winding up", "snapping back", ...
            "going over", "catching the swing"];

        while ~closing
            % --- sensing: trolley state and pendulum angle from vertical --
            x = trolley.Position(1);
            v = trolley.LinearVelocity(1);
            d = ball.Position - (trolley.Position - [0 0 0.08]);
            th = atan2(d(1), -d(3));
            w = -arm.AngularVelocity(2);

            % --- control law ---------------------------------------------
            switch phase
                case 0   % ZV input shaping: two half moves half a swing period
                         % apart, which leaves the load hanging still, so the
                         % wind up always starts from the same state
                    [x1, v1] = trapezoid(t, xWind - x0, 1.5, 1.5);
                    [x2, v2] = trapezoid(t - tShape, xWind - x0, 1.5, 1.5);
                    F = kTrack(1)*(x0 + (x1 + x2)/2 - x) + kTrack(2)*((v1 + v2)/2 - v);
                    if abs(x - xWind) < 0.01 && abs(v) < 0.05 && t > tShape
                        phase = 1; tPhase = t;
                    end
                case 1   % run at the wall and brake hard: the load falls behind
                         % on the way and the braking throws it forward
                    [xr, vr] = trapezoid(t - tPhase, xBrake - xWind, vWind, aWind);
                    F = kTrack(1)*(xWind + xr - x) + kTrack(2)*(vr - v);
                    if th > thTrig && w > 0 && t - tPhase > 0.5
                        phase = 2; tPhase = t;
                    end
                case 2   % snap back under the rising load: a pivot moving away
                         % from its load pumps it, and this one stroke is what
                         % lifts the swing over the height of the wall
                    [xr, vr] = trapezoid(t - tPhase, -dBack, vBack, aBack);
                    F = kTrack(1)*(xBrake + xr - x) + kTrack(2)*(vr - v);
                    if w < 0 && th > 0.3          % the load is at its peak
                        phase = 3; tPhase = t;
                    end
                case 3   % thrust out from under the load, which comes down on
                         % the far side. The trolley position is the fallback
                         % for a hop that fails to get the load over at all.
                    F = min(fDash, 400*(vCruise - v));
                    if x + armLen*sin(th) > xWall(k) + reach(k) + margin || x > xTarget
                        phase = 4; tPhase = t;
                    end
                case 4   % catch the swing and park where the load should end up
                    F = -K*[x - xTarget; v; sin(th); w*cos(th)];
            end
            advance(F);
            t = t + dt;
            c = min(c, clearance);

            if mod(step, 25) == 0
                viewer.displayText(sprintf("wall %d of %d - %s   |   swing %5.1f deg" + ...
                    "   |   force %6.1f N", k, nWalls, names(phase + 1), th*180/pi, F));
            end
            % done when the load hangs still, or when the catch has had its
            % time (a hop that fails simply ends swinging and is tried again)
            if phase == 4 && (t - tPhase > tCatch || (t - tPhase > 2 && ...
                    abs(th) < 0.05 && abs(w) < 0.3 && abs(v) < 0.1)) || t > 40
                break
            end
        end
    end

    % =====================================================================
    function onReturn(~, ~)
        busy(true);
        p0 = brickPositions;
        x0 = trolley.Position(1);
        t = 0;
        while ~closing
            x = trolley.Position(1);
            v = trolley.LinearVelocity(1);
            [xr, vr] = trapezoid(t, xStart - x0, 2.5, 3);
            F = kTrack(1)*(x0 + xr - x) + kTrack(2)*(vr - v);
            advance(F);
            t = t + dt;

            if mod(step, 25) == 0
                viewer.displayText(sprintf("driving home   |   x %5.2f m   |   force %6.1f N", ...
                    x, F));
            end
            if (abs(x - xStart) < 0.05 && abs(v) < 0.05) || t > 30
                break
            end
        end
        if bailOut, return, end

        sim.step(1, 250, 2);                      % let the rubble come to rest
        viewer.displayText("");
        hint.Text = sprintf("Home again - the drive back displaced %d of %d bricks.", ...
            sum(displaced(p0, brickPositions)), numel(bricks));
        busy(false);
    end

    % ------------------------------------------------------------ helpers
    function advance(F)
        % One step of the simulation under a clamped drive force
        trolley.applyForce([max(min(F, fMax), -fMax) 0 0], [], false);
        step = step + 1;
        sim.step(dt, 1, mod(step, 5) == 0);       % 60 redraws a second is plenty
    end

    function report(crossed, clear0, p0, hops)
        moved = displaced(p0, brickPositions);
        perWall = sum(reshape(moved, nBrick, nWalls), 1);
        done = all(crossed(inner));
        lines = strings(4, 1);
        lines(1) = sprintf("%d of %d walls crossed in %d hops (a wall is " + ...
            "cleared at %.0f deg)", sum(crossed), numel(inner), hops, thClear*180/pi);
        lines(2) = "closest to each wall: " + join(compose("%+.2f", clear0), " / ") + " m";
        lines(3) = "bricks displaced: " + join(compose("%d", perWall), " + ") + ...
            sprintf(" of %d", numel(bricks));
        if done && ~any(moved)
            lines(4) = "the load is in the last chamber, every wall untouched";
        elseif done
            lines(4) = "the load got there, but it did not leave the walls alone";
        else
            lines(4) = "the load did not make it to the last chamber";
        end
        viewer.displayText(lines);
        hint.Text = "Press Return to drive home through the walls.";
    end

    function c = clearance
        % Distance from the ball surface to each wall, taking the wall as the
        % box it fills, so a turned or offset one is measured as it stands.
        % Negative means the ball is inside the wall.
        p = ball.Position;
        c = zeros(1, nWalls);
        for j = 1:nWalls
            q = abs((p - wallC(j, :))*wallR(:, :, j));   % into the wall frame
            c(j) = norm(max(q - wallSize/2, 0)) - rBall;
        end
    end

    function p = brickPositions
        p = cell2mat(arrayfun(@(br) br.Position, bricks, "UniformOutput", false)');
    end

    function tf = displaced(p0, p)
        tf = vecnorm(p - p0, 2, 2)' > 0.03;
    end

    function busy(tf)
        % Nothing may be pressed while a run is in progress: the redraw inside
        % step lets button callbacks through
        running = tf;
        btnBuild.Enable = ~tf && isempty(sim);
        btnRun.Enable = ~tf && ~isempty(sim);
        btnBack.Enable = ~tf && ~isempty(sim);
        drawnow;
    end

    function tf = bailOut
        % The window was closed while the simulation was stepping (the redraw
        % inside step lets callbacks through), so tear down now, when no step
        % is in flight
        tf = closing;
        if tf
            delete(sim);
            delete(fig);
        end
    end

    function onClose(~, ~)
        closing = true;
        if running
            return          % let the running loop finish and tear down
        end
        delete(sim);
        delete(fig);
    end
end

function [x, v] = trapezoid(t, dist, vMax, aMax)
% Position and speed on a trapezoidal velocity profile (rest-to-rest by dist)
    s = sign(dist); dist = abs(dist);
    if dist == 0
        x = 0; v = 0;
        return
    end
    if dist < vMax^2/aMax
        vMax = sqrt(dist*aMax);
    end
    tAcc = vMax/aMax;
    tTotal = tAcc + dist/vMax;
    t = min(max(t, 0), tTotal);
    if t < tAcc
        x = aMax*t^2/2;
        v = aMax*t;
    elseif t < tTotal - tAcc
        x = vMax^2/(2*aMax) + vMax*(t - tAcc);
        v = vMax;
    else
        x = dist - aMax*(tTotal - t)^2/2;
        v = aMax*(tTotal - t);
    end
    x = s*x;
    v = s*v;
end