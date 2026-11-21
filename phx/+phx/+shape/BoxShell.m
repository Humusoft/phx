classdef BoxShell < phx.base.Shape
%phx.shape.BoxShell Box shell shape with a wall of given thickness
%
%   A hollow box: a wall of WallThickness inside the outer Size, solid
%   from both sides, like a vessel of sheet metal. A body outside the shell
%   touches its outer face, a body inside it (in the cavity) its inner
%   face; particles and a phx.Liquid inside stay in. The edges and corners
%   of the cavity are rounded by InnerEdgeRadius (0 keeps them sharp), and
%   those outside by InnerEdgeRadius + WallThickness, as a bent sheet is:
%   the wall has the same thickness everywhere. Visual appearance is the
%   outer and the inner face, the rounded edges drawn with Segments
%   segments per full turn; a texture is projected onto each side. Alpha is
%   the opacity of the wall as a whole: a line of sight through the wall
%   crosses both faces, so each face is drawn with the opacity
%   1 - sqrt(1 - Alpha).
%
%   Collision shape and mass properties are modeled as an ideal box shell
%   with those rounded edges, independent of the visual details. The shell
%   collides with all other shapes, meshes, terrains and shells included;
%   a body in a corner of the cavity touches every wall it meets. As any
%   thin wall, it lets through a body that moves farther in one step than
%   the wall thickness plus its own size; shorten the step for fast
%   bodies.
%
%   phx.shape.BoxShell() creates a shape with default parameters.
%
%   phx.shape.BoxShell(Name, Value, ...) creates a shape and sets
%   properties values according to given name-value pairs:
%
%       tank = phx.Body(gca, "Type", "static", "Shape", ...
%           phx.shape.BoxShell("Size", [1.2 0.6 0.8], "WallThickness", 0.02));
%
%   See also phx.Body, phx.shape.Box, phx.shape.SphereShell,
%   phx.shape.CapsuleShell, phx.Liquid

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

%#ok<*INUSD> OK to see the full list of arguments for callbacks

    properties
        % Outer size of the shell (X Y Z)
        Size (1, 3) double {mustBePositive, mustBeFinite} = [1 1 1]

        % Thickness of the wall, inwards from the outer size
        WallThickness (1, 1) double {mustBePositive, mustBeFinite} = 0.05

        % Radius of the rounded edges and corners of the cavity; 0 keeps
        % them sharp. The outer edges are rounded by this radius plus the
        % wall thickness.
        InnerEdgeRadius (1, 1) double {mustBeNonnegative, mustBeFinite} = 0

        % Number of segments of the rounded edges per full turn
        Segments (1, 1) double = 24

        % Volumetric density of the wall (kg/m^3)
        Density (1, 1) double = 1000
    end

    properties (Dependent, SetAccess = private)
        % Size of the cavity, Size - 2*WallThickness
        InnerSize (1, 3) double
    end

    methods
        function obj = BoxShell(Options)
            arguments
                Options.?phx.shape.BoxShell
            end

            % Process input arguments
            obj = phx.internal.applyArguments(Options, obj);
            if isnan(obj.Color(1))
                obj.Color = phx.base.Shape.newColor;
            end
        end

        function s = get.InnerSize(obj)
            s = obj.Size - 2*obj.WallThickness;
        end

        function drawTo(obj, target)
            obj.checkWall;

            % The outer face, and the inner face turned inside out: its
            % normals and its winding face the cavity
            n = max(1, round(obj.Segments/4));
            core = obj.core;
            [V, N, F, T] = roundedBox(core, obj.InnerEdgeRadius + obj.WallThickness, n);
            [Vi, Ni, Fi, Ti] = roundedBox(core, obj.InnerEdgeRadius, n);
            V = [V; Vi];
            N = [N; -Ni];
            F = [F; fliplr(Fi) + size(V, 1) - size(Vi, 1)];   % V holds both faces now
            T = [T; Ti];

            % Alpha is the opacity of the wall, which a line of sight
            % crosses at two faces: each takes 1 - sqrt(1 - Alpha)
            faces = obj;
            faces.Alpha = 1 - sqrt(1 - obj.Alpha);
            primitive = faces.drawMesh(target, V, N, F, T, obj.getTexture);
            setappdata(primitive, "phxShape", obj);
        end

        function [volume, inertia, center] = computeVolume(obj)
            % Integrated over a fine mesh of both faces, the inner one turned
            % inside out, so that it encloses the wall alone (the flat sides
            % exact, the rounded edges to well under 0.1 %)
            obj.checkWall;
            n = 16;
            core = obj.core;
            [V, ~, F] = roundedBox(core, obj.InnerEdgeRadius + obj.WallThickness, n);
            [Vi, ~, Fi] = roundedBox(core, obj.InnerEdgeRadius, n);
            [volume, ~, Icm] = phx.internal.Geometry.meshMass([V; Vi], [F; fliplr(Fi) + size(V, 1)], 1);
            inertia = Icm([1 5 9]);
            center = [0 0 0];
        end
    end

    methods (Access = {?phx.base.Shape, ?phx.base.Object})
        function eh = createBody(obj, body, primitive)
            obj.checkWall;
            eh = phxEngine('add', body.WorldHandle, 'boxshell', body.TypeID, obj.Size/2, obj.WallThickness, ...
                obj.InnerEdgeRadius, body.Transform, body.Mass, body.Inertia, body.CenterOfMass);
        end
    end

    methods (Access = private)
        function checkWall(obj)
            % The wall and the rounding must leave a cavity in every
            % direction; checked where the shape is used, so that the sizes
            % may be set in any order
            if any(obj.core <= 0)
                error("phx:BoxShell:badWallThickness", ...
                    "Half the Size (%s) must exceed WallThickness + InnerEdgeRadius (%g) in every direction; a solid box is phx.shape.Box.", ...
                    mat2str(obj.Size/2, 4), obj.WallThickness + obj.InnerEdgeRadius);
            end
        end

        function c = core(obj)
            % Half extents of the box whose rounding by the radii gives the
            % faces
            c = obj.Size/2 - obj.WallThickness - obj.InnerEdgeRadius;
        end
    end

end

function [V, N, F, T] = roundedBox(core, r, n)
%roundedBox Mesh of the box of half extents core grown by r.
%   A latitude-longitude grid of n segments per quarter turn whose rings at
%   the octant boundaries are doubled, each vertex moved out by the core in
%   its octant: the doubled rings open into the flat sides and the rounded
%   edges, and two caps close the top and the bottom. Vertex normals are
%   those of the rounding; texture coordinates are projected onto the side
%   each normal faces most. r = 0 gives the sharp box.
    if r == 0
        % triBox winds its triangles clockwise seen from outside
        [V, N, F, T] = phx.internal.Geometry.triBox(2*core);
        F = fliplr(F);
        return
    end
    q = (0:n)'/n*pi/2;
    phi = [q; q + pi/2; q + pi; q + 3*pi/2];
    sx = repelem([1; -1; -1; 1], n + 1);
    sy = repelem([1; 1; -1; -1], n + 1);
    theta = [q - pi/2; q];
    sz = repelem([-1; 1], n + 1);
    [P, TH] = meshgrid(phi, theta);           % rows latitude, columns longitude
    [SX, SZ] = meshgrid(sx, sz);
    SY = repmat(sy', numel(theta), 1);
    N = [cos(TH(:)).*cos(P(:)), cos(TH(:)).*sin(P(:)), sin(TH(:))];
    V = r*N + [SX(:)*core(1), SY(:)*core(2), SZ(:)*core(3)];

    % Quads between neighbouring rows and columns, the last column wrapping
    % to the first: east, then north, faces outwards
    rows = numel(theta);
    cols = numel(phi);
    id = @(i, j) (mod(j - 1, cols))*rows + i;
    [I, J] = ndgrid(1:rows - 1, 1:cols);
    a = id(I(:), J(:)); b = id(I(:), J(:) + 1); c = id(I(:) + 1, J(:) + 1); d = id(I(:) + 1, J(:));
    F = [a b c; a c d];

    % Caps over the corners at the poles, counter-clockwise seen from outside
    first = (0:3)*(n + 1) + 1;
    top = id(rows, first);
    bottom = id(1, first);
    F = [F; top([1 2 3]); top([1 3 4]); bottom([1 3 2]); bottom([1 4 3])];

    % Drop the triangles of no area (at the poles)
    e = cross(V(F(:, 2), :) - V(F(:, 1), :), V(F(:, 3), :) - V(F(:, 1), :), 2);
    F = F(vecnorm(e, 2, 2) > 1e-12*max(core + r)^2, :);

    % Texture coordinates on the side each vertex faces most
    H = core + r;
    [~, k] = max(abs(N), [], 2);
    T = zeros(size(V, 1), 2);
    others = [2 3; 1 3; 1 2];
    for axis = 1:3
        m = k == axis;
        T(m, :) = (V(m, others(axis, :)) + H(others(axis, :)))./(2*H(others(axis, :)));
    end
end
