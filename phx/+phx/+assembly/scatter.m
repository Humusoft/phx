function bodies = scatter(varargin)
%phx.assembly.scatter Scatter bodies randomly inside a box region
%
%   bodies = phx.assembly.scatter(shape, n) creates n dynamic bodies with
%   the given shape at uniformly random positions in the current axes and
%   returns them as a 1-by-n phx.Body array. Like with phx.Body, a target
%   axes to draw into may be passed as an optional first argument:
%   phx.assembly.scatter(ax, shape, n, ___), where an empty target ([])
%   creates the bodies without graphics. The shape is either a cell
%   array of a phx.shape.* class name and constructor arguments - a new
%   shape is then created for every body, so shapes with random geometry
%   such as phx.shape.Rock give natural variety - or a phx.shape.* object
%   shared by all bodies.
%
%   The body centres are drawn from a box region spanning -x/2..x/2 and
%   -y/2..y/2 in the ground plane and 0..z upwards (the same convention as
%   the inner space of phx.assembly.arena), placed by default around the
%   world origin.
%
%   The random numbers are drawn from the global generator (like rand),
%   so a layout is reproduced by seeding the generator first with rng.
%
%   bodies = phx.assembly.scatter(___, Name, Value, ...) specifies options
%   as name-value pairs:
%   - Region: dimensions [x y z] of the box that the body centres are
%     drawn from, default [2 2 1].
%   - Spacing: minimum distance between body centres, default 0 (no
%     limit). With a positive spacing the positions are sampled by
%     rejection, so bodies whose bounding sphere diameter is below the
%     spacing are guaranteed not to overlap at the start. When the
%     requested count does not fit, the error phx:scatter:regionFull is
%     raised - reduce Spacing, lower n or enlarge Region.
%   - RandomOrientation: give every body a uniformly random orientation,
%     default false.
%   - Color: body colors, either one common 1x3 RGB color or an n-by-3
%     matrix with one color per body. Applied over the shape color.
%   - Friction: friction coefficients of all bodies, default [0.5 0 0].
%   - Position, Orientation, EulerAngles: pose of the region frame in the
%     world, default at the world origin. Same conventions as phx.Body.
%
%   Example - a pile of rocks dropped into an arena:
%       phx.assembly.arena("Size", [8 8 1]);
%       rocks = phx.assembly.scatter({"Rock", "Radius", 0.4}, 30, ...
%           "Region", [7 7 4], "Spacing", 0.8, "Position", [0 0 1]);
%       sim = phx.Simulation(gca);
%       sim.step(3, 600, 6);
%
%   See also phx.assembly.arena, phx.assembly.chain, phx.Body, rng

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

    [ax, args] = axesTarget(varargin);
    bodies = build(ax, args{:});
end

function bodies = build(ax, shape, n, Options)
    arguments
        ax
        shape
        n (1, 1) double {mustBeInteger, mustBePositive}
        Options.Region (1, 3) double {mustBeNonnegative} = [2 2 1]
        Options.Spacing (1, 1) double {mustBeNonnegative} = 0
        Options.RandomOrientation (1, 1) logical = false
        Options.Color double = double.empty
        Options.Friction (1, 3) double {mustBeGreaterThanOrEqual(Options.Friction, 0)} = [0.5 0 0]
        Options.Position (1, 3) double = [0 0 0]
        Options.Orientation (3, 3) double = eye(3)
        Options.EulerAngles (1, 3) double = [0 0 0]
    end

    TBase = basePose(Options, "scatter");

    if ~iscell(shape) && ~isa(shape, "phx.base.Shape")
        error("phx:scatter:invalidShape", "The shape must be a phx.shape object or a cell array of a shape class name and constructor arguments.");
    end

    colors = Options.Color;
    if ~isempty(colors)
        if isequal(size(colors), [1 3])
            colors = repmat(colors, n, 1);
        elseif ~isequal(size(colors), [n 3])
            error("phx:scatter:invalidColor", "Color must be a 1x3 RGB color or an n-by-3 matrix matching the %d bodies (got %dx%d).", n, size(colors, 1), size(colors, 2));
        end
    end

    % Rejection-sample the body centres in the region frame
    region = Options.Region;
    pos = zeros(n, 3);
    placed = 0;
    budget = max(1000, 200*n);
    while placed < n && budget > 0
        budget = budget - 1;
        c = [(rand(1, 2) - 0.5).*region(1:2), rand*region(3)];
        if Options.Spacing > 0 && placed > 0 && ...
                any(sum((pos(1:placed, :) - c).^2, 2) < Options.Spacing^2)
            continue
        end
        placed = placed + 1;
        pos(placed, :) = c;
    end
    if placed < n
        error("phx:scatter:regionFull", "Only %d of %d bodies fit into the region with the spacing %g; reduce Spacing, lower the count or enlarge Region.", placed, n, Options.Spacing);
    end

    % Create the bodies (a cell shape spec builds a new shape per body)
    if isa(ax, "missing")
        ax = gca;
    end
    bodies = phx.Body.empty;
    for i = 1:n
        bodies(i) = phx.Body(ax, "Shape", shape, "Friction", Options.Friction);
        T = eye(4);
        if Options.RandomOrientation
            T(1:3, 1:3) = randomRotation;
        end
        T(1:3, 4) = pos(i, :)';
        bodies(i).Transform = TBase*T;
        if ~isempty(colors)
            bodies(i).Color = colors(i, :);
        end
    end
end

function R = randomRotation
% Uniformly random rotation matrix (QR of a Gaussian matrix, with the
% sign fixed so that the result is a proper rotation)
    [R, ~] = qr(randn(3));
    R = R*diag([det(R) 1 1]);
end
