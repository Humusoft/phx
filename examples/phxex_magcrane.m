function out = phxex_magcrane(Options)
% PHXEX_MAGCRANE Overhead crane moving a car body on a lifting magnet
%
% A bridge crane picks a car body shell off the floor with a lifting
% magnet hanging on its hoist rope and sets it down on a pad across the
% hall. The magnet is a phx.Monopole charge on the magnet body, so the
% shell is held by nothing but the attraction and the contact under it:
% the grip is a force, not a joint, and the load can slide on the pole
% face, tilt on it, or be left behind if the pull is too weak.
%
% The charge is sized from the wanted holding force at contact, where
% the collision keeps the pole distance fixed; HoldFactor says how many
% load weights that force is. The whole cycle - lower, energize, hoist,
% travel, lower, release - is played from the rope payout (a winch,
% through phx.Rope.Displacement) and the kinematic bridge and trolley.
%
% The report gives the moment the shell broke free of the floor, the sway
% it carried into the set-down and the resulting placement error.
%
% Options:
%     HoldFactor - holding force at contact, in load weights; the magnet
%                  is energized across an air gap, so a factor of 1 is
%                  not enough to break the shell free of the floor
%     TravelTime - duration of the diagonal travel; short times swing the
%                  load and slide it off the pole face
%     Settle     - pause between the travel and the set-down, for the sway
%                  to decay
%
% Example:
%     phxex_magcrane(HoldFactor = 1)   % too weak, the shell stays down
%     phxex_magcrane(TravelTime = 2)   % the shell slides off in mid-air
%
% See also phx.Monopole, phx.Rope, phxex_antisway, phxex_maglev

%   Copyright 2026 HUMUSOFT s.r.o.

    arguments
        Options.HoldFactor (1, 1) double {mustBePositive} = 3
        Options.TravelTime (1, 1) double {mustBePositive} = 6
        Options.Settle (1, 1) double {mustBeNonnegative} = 3
    end

    grav = 9.81;
    pick = [-4 -2.5];            % where the shell lies
    drop = [4 2.5];              % where it is to be set down
    zBridge = 8;                 % runway and bridge beams
    zTrolley = 7.6;              % underside of the trolley
    zHoist = 5.5;                % magnet height with the rope wound in
    zTravel = 4.2;               % magnet height during the travel
    padHeight = 0.25;            % set-down pad

    dMag = 1.0;  hMag = 0.3;     % lifting magnet

    % The shell: an STL model with a convex collision envelope, drawn
    % decimated and scaled from millimetres, its origin at the centre of
    % its bounding box
    shellShape = phx.shape.Mesh("Source", "res/BuggyBody.stl", "Details", 0.2, ...
        "Scale", 0.001, "Envelope", "convex", "Density", 80, ...
        "Color", [0.45 0.5 0.6], "Material", "metal");
    zRoof = 0.641;               % bounding-box half height of the shell

    % Figure setup
    figure(1);
    [viewer, ax] = phx.extra.Viewer("clear", "DefaultCameraTarget", [0 0 2.5], ...
        "DefaultCameraPosition", [-3 -22 11]);

    % Hall floor, the runway on four posts and the set-down pad
    parts = phx.Body(ax, "Type", "static", "Position", [0 0 -0.5], ...
        "Shape", {"Box", "Size", [20 14 1], "Texture", "tiles"});
    for c = [-1 -1; -1 1; 1 1; 1 -1]'
        parts(end + 1) = phx.Body(ax, "Type", "static", "Position", [7*c(1) 5*c(2) 3.9], ...
            "Shape", {"Box", "Size", [0.4 0.4 7.8], "Color", [0.55 0.55 0.6], "Texture", "metal"}); %#ok<AGROW>
    end
    for y = [-5 5]
        parts(end + 1) = phx.Body(ax, "Type", "static", "Position", [0 y zBridge], ...
            "Shape", {"Box", "Size", [14.4 0.35 0.6], "Color", [0.55 0.55 0.6], "Texture", "metal"}); %#ok<AGROW>
    end
    parts(end + 1) = phx.Body(ax, "Type", "static", "Position", [drop padHeight/2], ...
        "Shape", {"Box", "Size", [3.4 2.2 padHeight], "Color", [0.85 0.7 0.2], "Texture", "wood", "TextureBlend", 0.3});

    % The crane: a bridge travelling along the runway, a trolley along the
    % bridge, both kinematic, and the magnet hanging on the hoist rope
    bridge = phx.Body(ax, "Type", "kinematic", "Position", [pick(1) 0 zBridge], ...
        "Shape", {"Box", "Size", [0.7 10.6 0.5], "Color", [0.9 0.6 0.15]});
    trolley = phx.Body(ax, "Type", "kinematic", "Position", [pick zTrolley], ...
        "Shape", {"Box", "Size", [1.1 1 0.4], "Color", [0.75 0.25 0.2]});
    magnet = phx.Body(ax, "Position", [pick zHoist], ...
        "Shape", {"Cylinder", "Diameter", dMag, "Height", hMag, "Density", 800, ...
        "Color", 0.3, "Material", "metal"});
    shell = phx.Body(ax, "Position", [pick zRoof], "Shape", shellShape);

    rope = phx.Rope([trolley magnet], "Points", [0 0 -0.2; 0 0 hMag/2], ...
        "Stiffness", 6e5, "Damping", 6e3, ...
        "Colormap", "heat", "ColorRange", [0 12000]);

    % Tracking camera
    phx.Camera(parts(1), shell, "PointA", [-3 -22 11], "TrackingLag", 1);

    % The lifting magnet: a charge pair switched on and off by writing
    % Charge while the simulation runs, sized for the contact distance
    mLoad = shell.Mass;
    rHold = hMag/2 + zRoof;
    qMag = Options.HoldFactor*mLoad*grav*rHold^2;
    % Its field arrows fill a small window that travels with the magnet
    coil = phx.Monopole([magnet shell], "Charge", [0 -1]', "Attractivity", -1, ...
        "VectorFieldCenter", [pick zHoist], "VectorFieldSize", [3 0 2.5], ...
        "VectorFieldStep", 0.5, "VectorLength", 0.4, "VectorSegments", 3, "Color", 1);
    fprintf("Shell %.0f kg, holding force %.1f kN at contact - %.1f times its weight.\n", ...
        mLoad, Options.HoldFactor*mLoad*grav/1000, Options.HoldFactor);

    % The cycle as timed segments, with the magnet height at their ends
    gap = 0.08;                                 % air gap the coil bridges
    zPick = 2*zRoof + hMag/2 + gap;             % just over the resting shell
    zDrop = padHeight + zRoof + rHold + gap;    % shell just over the pad
    tSeg = [1 2.5 1.5 3 Options.TravelTime Options.Settle 2.5 2];
    zSeg = [zHoist zHoist zPick zPick zTravel zTravel zTravel zDrop zDrop];
    label = ["settle", "lower to the shell", "magnet on", "hoist", ...
        "travel", "let the sway decay", "lower onto the pad", "magnet off"];
    tEnd = cumsum(tSeg);
    pay = zHoist - zSeg;                        % the same heights as payout

    sim = phx.Simulation([parts bridge trolley magnet shell]);

    % Hidden only now: an object that is invisible when the pipelines are
    % built stays out of the redraw (ExcludeInvisible) for the whole run
    coil.Visible = false;

    dt = 0.004;
    subSteps = 5;
    log = struct("t", [], "z", [], "sway", [], "force", []);
    t = 0;
    zRest = NaN;
    tLift = NaN;
    while t < tEnd(end)
        for s = 1:subSteps
            % The winch interpolates the payout over the running segment
            k = min(sum(t >= tEnd) + 1, numel(tSeg));
            u = smooth((t - tEnd(k) + tSeg(k))/tSeg(k));
            rope.Displacement = pay(k) + (pay(k + 1) - pay(k))*u;

            % The magnet is live from the grip segment to the release
            live = t >= tEnd(2) && t < tEnd(7);
            coil.Charge = [qMag*live -1]';

            % The bridge and the trolley travel diagonally; smooth clamps
            % the progress, so they wait at pick and stop at drop
            travel = pick + (drop - pick)*smooth((t - tEnd(4))/tSeg(5));
            bridge.Position = [travel(1) 0 zBridge];
            trolley.Position = [travel zTrolley];

            % The field window follows the magnet, and shows only while live
            redraw = mod(s, subSteps) == 0;
            if redraw
                coil.VectorFieldCenter = magnet.Position - [0 0 0.3];
                coil.Visible = live;
            end

            sim.step(dt, 1, redraw);
            t = t + dt;
        end

        % Where the shell rests, and when the magnet breaks it free
        if t >= tEnd(1) && isnan(zRest)
            zRest = shell.Position(3);
        elseif isnan(tLift) && shell.Position(3) > zRest + 0.1
            tLift = t;
        end

        log.t(end + 1) = t;
        log.z(end + 1) = shell.Position(3);
        log.sway(end + 1) = norm(shell.Position(1:2) - trolley.Position(1:2));
        log.force(end + 1) = rope.Force;
        viewer.displayText(sprintf("%s   hoist %.2f m   sway %.2f m   rope %.1f kN", ...
            label(k), shell.Position(3), log.sway(end), rope.Force/1000));
    end

    % The pick-up, the sway carried into the set-down and the result
    p = shell.Position;
    idSway = log.t > tEnd(4) & log.t < tEnd(6);
    out = struct("Lifted", ~isnan(tLift), "Position", p, ...
        "Error", norm(p(1:2) - drop), "Tilt", acosd(min(shell.Orientation(3, 3), 1)), ...
        "MaxSway", max(log.sway(idSway)));
    if out.Lifted
        fprintf("Broke free at t = %.1f s, peak rope tension %.1f kN.\n", ...
            tLift, max(log.force)/1000);
        fprintf("Sway up to %.2f m, set down %.2f m off the mark, tilted %.1f deg.\n", ...
            out.MaxSway, out.Error, out.Tilt);
    else
        fprintf("The shell never left the floor - the grip is too weak.\n");
    end
    delete(sim);

    % Hoist height, sway and rope tension over the cycle
    clf(figure(2));
    subplot(2, 1, 1);
    plot(log.t, log.z, log.t, log.sway, "LineWidth", 1.5);
    grid on; ylabel("[m]");
    legend("shell height", "sway behind the trolley");
    title(sprintf("Lifting magnet at %.1f load weights", Options.HoldFactor));
    subplot(2, 1, 2);
    plot(log.t, log.force/1000, "LineWidth", 1.5); hold on
    yline(mLoad*grav/1000, "--", "load weight");
    xline(tEnd([2 7]), ":", ["magnet on", "magnet off"]);
    grid on; xlabel("time [s]"); ylabel("rope tension [kN]");
end

function y = smooth(x)
% Smooth step from 0 to 1 over the unit interval
    x = min(max(x, 0), 1);
    y = x*x*(3 - 2*x);
end
