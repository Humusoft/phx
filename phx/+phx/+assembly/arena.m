function parts = arena(varargin)
%phx.assembly.arena Build a static arena - a floor plate enclosed by four walls
%
%   parts = phx.assembly.arena() creates a static floor with four walls around
%   it in the current axes, ready to keep dynamic bodies from escaping the
%   scene. Like with phx.Body, a target axes to draw into may be passed
%   as an optional first argument: phx.assembly.arena(ax, ___). An empty
%   target ([]) creates the parts without graphics for headless
%   simulations. The arena origin lies at the middle of the floor surface, so by
%   default the inner space spans -x/2..x/2 and -y/2..y/2 in the ground
%   plane and 0..z upwards around the world origin. The parts are returned
%   in a struct with the fields:
%   - floor: the floor body (phx.Body)
%   - walls: the wall bodies (1x4 phx.Body, in the order -x, +x, -y, +y)
%
%   parts = phx.assembly.arena(Name, Value, ...) specifies options as
%   name-value pairs:
%   - Size: inner dimensions [x y z] of the arena - the usable floor area
%     x-by-y and the inner height z of the walls. Default [2 2 0.5].
%   - Thickness: thickness of the floor and wall plates, default 0.1.
%   - Color: common color of all parts, default [0.75 0.73 0.7].
%   - Friction: friction coefficients of all parts, default [0.5 0 0].
%   - Position: world position of the arena origin (the middle of the
%     floor surface), default [0 0 0].
%   - Orientation: world rotation of the arena as a 3x3 rotation matrix,
%     default eye(3). Same convention as phx.Body.Orientation.
%   - EulerAngles: world rotation of the arena as Euler angles for the
%     z->y->x order, an alternative to Orientation (do not combine the
%     two). Same convention as phx.Body.EulerAngles.
%
%   The individual parts are ordinary static bodies, so they can be
%   restyled or retextured afterwards through the returned struct:
%       parts = phx.assembly.arena("Size", [4 3 0.3]);
%       set(parts.walls, "Color", [0.4 0.55 0.7]);
%       sim = phx.Simulation(gca);
%
%   See also phx.assembly.import, phx.Body, phx.shape.Box, phx.Simulation

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

    [ax, args] = axesTarget(varargin);
    parts = build(ax, args{:});
end

function parts = build(ax, Options)
    arguments
        ax
        Options.Size (1, 3) double {mustBePositive} = [2 2 0.5]
        Options.Thickness (1, 1) double {mustBePositive} = 0.1
        Options.Color (1, 3) double = [1 1 1]
        Options.Friction (1, 3) double {mustBeGreaterThanOrEqual(Options.Friction, 0)} = [0.5 0 0]
        Options.Position (1, 3) double = [0 0 0]
        Options.Orientation (3, 3) double = eye(3)
        Options.EulerAngles (1, 3) double = [0 0 0]
    end

    TBase = basePose(Options, "arena");
    s = Options.Size;
    t = Options.Thickness;

    % The floor plate extends under the walls and its top surface passes
    % through the arena origin. The walls stand on the floor: the two x
    % walls fit exactly between the two y walls, which span the full outer
    % width and close the corners, so the inner space is exactly s(1) by
    % s(2) by s(3).
    sizes = {[s(1) + 2*t, s(2) + 2*t, t]
             [t, s(2), s(3)]
             [t, s(2), s(3)]
             [s(1) + 2*t, t, s(3)]
             [s(1) + 2*t, t, s(3)]};
    centres = {[0, 0, -t/2]
               [-(s(1) + t)/2, 0, s(3)/2]
               [(s(1) + t)/2, 0, s(3)/2]
               [0, -(s(2) + t)/2, s(3)/2]
               [0, (s(2) + t)/2, s(3)/2]};
    names = ["floor" "wall" "wall" "wall" "wall"];

    if isa(ax, "missing")
        ax = gca;
    end
    bodies = phx.Body.empty;
    for i = 1:numel(sizes)
        bodies(i) = phx.Body(ax, "Type", "static", "Name", names(i), ...
            "Shape", {"Box", "Size", sizes{i}, "Color", Options.Color}, ...
            "Friction", Options.Friction);
        T = eye(4);
        T(1:3, 4) = centres{i};
        bodies(i).Transform = TBase*T;
    end

    parts.floor = bodies(1);
    parts.walls = bodies(2:5);
end
