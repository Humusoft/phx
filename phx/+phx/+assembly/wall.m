function bricks = wall(varargin)
%phx.assembly.wall Build a wall of bricks laid in a running bond
%
%   bricks = phx.assembly.wall() creates a wall of loose brick bodies in the
%   current axes and returns them as a 1-by-n phx.Body array. Like with
%   phx.Body, a target axes to draw into may be passed as an optional first
%   argument: phx.assembly.wall(ax, ___). An empty target ([]) creates the
%   bricks without graphics for headless simulations. The wall origin lies
%   at the middle of its base, so by default the wall spans -x/2..x/2 in
%   length, -y/2..y/2 in thickness and 0..z upwards around the world
%   origin.
%
%   The bricks are ordinary dynamic bodies that only rest on each other -
%   there is no mortar - so the wall stands by friction and gravity alone
%   and can be knocked down. Every row is laid in a running bond: the even
%   rows are shifted by half a brick against the odd ones. The bricks come
%   back row by row, the bottom row first and left to right (in the +x
%   direction) within a row; their names ("brick<row>_<column>") carry the
%   same indices.
%
%   bricks = phx.assembly.wall(Name, Value, ...) specifies options as
%   name-value pairs:
%   - Size: overall dimensions [x y z] of the wall - its length x, its
%     thickness y and its height z. Default [2 0.2 1]. The brick size
%     follows from the row and column counts: a brick is x/Columns long,
%     as thick as the wall and z/Rows high.
%   - Rows: number of brick rows stacked on top of each other, default 8.
%   - Columns: number of bricks in an odd (unshifted) row, default 8.
%   - HalfBricks: how the shifted even rows end at the sides of the wall,
%     default true. With true the even rows are closed by a half brick at
%     each side, so both ends of the wall are flush and an even row holds
%     Columns+1 bricks. With false the even rows are left open, i.e. they
%     hold Columns-1 full bricks and are inset by half a brick at both
%     ends, which leaves the toothed edge a wall continues from.
%   - Density: density of the brick material (kg/m^3), default 2000; the
%     brick masses and inertias follow from the geometry.
%   - Color: common color of all bricks, default [1 1 1].
%   - RandomTint: random shade variation between the bricks, default 0.1.
%     Every brick is darkened by its own random factor,
%     brickColor = Color*(1 - rand*RandomTint), so the shades differ while
%     the hue stays; 0 leaves all bricks in exactly the same color. The
%     random numbers are drawn from the global generator (like rand), so a
%     wall is reproduced by seeding the generator first with rng.
%   - Friction: friction coefficients of all bricks, default [0.5 0 0].
%   - Position: world position of the wall origin (the middle of its base),
%     default [0 0 0].
%   - Orientation: world rotation of the wall as a 3x3 rotation matrix,
%     default eye(3). Same convention as phx.Body.Orientation.
%   - EulerAngles: world rotation of the wall as Euler angles for the
%     z->y->x order, an alternative to Orientation (do not combine the
%     two). Same convention as phx.Body.EulerAngles.
%
%   The bricks are returned as plain bodies, so they can be restyled or
%   picked apart afterwards (note that setting Color on the returned bodies
%   overwrites the random tint):
%       phx.assembly.arena("Size", [6 4 0.2]);
%       bricks = phx.assembly.wall("Size", [3 0.25 1.2], "Rows", 10, ...
%           "Columns", 6, "EulerAngles", [0 0 pi/2], ...
%           "Color", [0.7 0.35 0.25], "RandomTint", 0.3);
%       ball = phx.Body("Shape", {"Sphere", "Diameter", 0.4, "Density", 4000}, ...
%           "Position", [-2.5 0 0.6], "LinearVelocity", [10 0 1]);
%       sim = phx.Simulation(gca);
%       sim.step(2, 1000, 20);
%
%   See also phx.assembly.arena, phx.assembly.scatter, phx.assembly.chain,
%   phx.Body, phx.shape.Box

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

    [ax, args] = axesTarget(varargin);
    bricks = build(ax, args{:});
end

function bricks = build(ax, Options)
    arguments
        ax
        Options.Size (1, 3) double {mustBePositive} = [2 0.2 1]
        Options.Rows (1, 1) double {mustBeInteger, mustBePositive} = 8
        Options.Columns (1, 1) double {mustBeInteger, mustBePositive} = 8
        Options.HalfBricks (1, 1) logical = true
        Options.Density (1, 1) double {mustBePositive} = 2000
        Options.Color (1, 3) double = [1 1 1]
        Options.RandomTint (1, 1) double {mustBeInRange(Options.RandomTint, 0, 1)} = 0.1
        Options.Friction (1, 3) double {mustBeGreaterThanOrEqual(Options.Friction, 0)} = [0.5 0 0]
        Options.Position (1, 3) double = [0 0 0]
        Options.Orientation (3, 3) double = eye(3)
        Options.EulerAngles (1, 3) double = [0 0 0]
    end

    TBase = basePose(Options, "wall");

    nRows = Options.Rows;
    nCols = Options.Columns;
    if ~Options.HalfBricks && nRows > 1 && nCols < 2
        error("phx:wall:invalidColumns", "Open even rows hold one brick less than the odd ones, so a wall with HalfBricks set to false needs at least two columns (got %d).", nCols);
    end

    % A brick fills its share of the wall volume: the rows split the height,
    % the columns split the length of an odd row and the brick is as thick
    % as the wall itself
    len = Options.Size(1)/nCols;
    thickness = Options.Size(2);
    height = Options.Size(3)/nRows;

    if isa(ax, "missing")
        ax = gca;
    end
    bricks = phx.Body.empty;
    for row = 1:nRows
        % Brick lengths of the row and the x coordinate its left side starts
        % at; the even rows are shifted by half a brick and either closed by
        % half bricks or left open at both ends
        if mod(row, 2) == 1
            lengths = repmat(len, 1, nCols);
            left = -Options.Size(1)/2;
        elseif Options.HalfBricks
            lengths = [len/2, repmat(len, 1, nCols - 1), len/2];
            left = -Options.Size(1)/2;
        else
            lengths = repmat(len, 1, nCols - 1);
            left = -Options.Size(1)/2 + len/2;
        end

        for col = 1:numel(lengths)
            % Each brick gets its own shade; with no tint the generator is
            % left alone, so the wall does not disturb other random layouts
            color = Options.Color;
            if Options.RandomTint > 0
                color = color*(1 - rand*Options.RandomTint);
            end

            brick = phx.Body(ax, "Name", sprintf("brick%d_%d", row, col), ...
                "Shape", {"Box", "Size", [lengths(col) thickness height], ...
                    "Color", color, "Density", Options.Density}, ...
                "Friction", Options.Friction);
            T = eye(4);
            T(1:3, 4) = [left + sum(lengths(1:col - 1)) + lengths(col)/2
                         0
                         (row - 0.5)*height];
            brick.Transform = TBase*T;
            bricks(end + 1) = brick; %#ok<AGROW> the count is data-dependent
        end
    end
end
