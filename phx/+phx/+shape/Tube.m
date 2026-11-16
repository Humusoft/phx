classdef Tube < phx.base.Shape
%phx.shape.Tube Tube, pipe or funnel shape
%
%   Visual appearance is a straight or tapered tube with an open bore.
%   InnerDiameter gives the bore at the negative and the positive end of
%   Axis; a single value is a straight tube, two values taper the bore while
%   the wall keeps its thickness, so a single shape covers a pipe, a funnel,
%   a hopper and a closed cup.
%
%   Collision shape is selected by the Envelope property: the exact concave
%   triangle mesh, which is the only one that leaves the bore open, the convex
%   hull of the mesh, or a bounding cylinder about the tube axis, which keeps
%   a rolling body smooth where a faceted hull would not. Mass properties are
%   always calculated from the actual triangular mesh.
%
%   As the Shape of a phx.FlexibleBody, the tube wall is filled with a mesh
%   of tetrahedra (generateVolumetricMesh); as the Shape of a phx.SoftBody,
%   it is covered with a mesh of evenly spaced vertices
%   (generateSurfaceMesh). The ElementResolution sets how fine they are.
%
%   phx.shape.Tube() creates a shape with default parameters.
%
%   phx.shape.Tube(Name, Value, ...) creates a shape and sets properties
%   values according to given name-value pairs.
%
%   See also phx.Body, phx.FlexibleBody, phx.SoftBody, phx.shape.Cylinder,
%   phx.shape.Revolution

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

%#ok<*MCSUP> OK to access other properties in setters
%#ok<*INUSD> OK to see the full list of arguments for callbacks

    properties
        % Inner diameters at the negative and the positive end of Axis
        % A single value d is [d d], a straight tube; two values taper the
        % bore while the wall keeps its thickness. A zero closes the bore at
        % that end, leaving a disc as wide as the wall is thick.
        InnerDiameter (1, :) double {mustBeBoreSize} = [0.9 0.9]

        % Thickness of the tube wall
        WallThickness (1, 1) double {mustBePositive, mustBeFinite} = 0.05

        % Tube height along the modeling axis
        Height (1, 1) double = 1

        % Modeling axis of the tube
        Axis {mustBeMember(Axis, ["x", "y", "z"])} = "z"

        % Number of tube segments
        Segments (1, 1) double = 24

        % Volumetric density (kg/m^3)
        Density (1, 1) double = 1000

        % Collision envelope: "concave" triangle mesh, "convex" hull or
        % bounding "cylinder" about the tube axis
        Envelope {mustBeMember(Envelope, ["concave", "convex", "cylinder"])} = "concave"

        % Resolution of the meshes of flexible and soft bodies: the number
        % of cells across the smaller of the wall thickness and the height,
        % the wall cut around and along into cells as close to cubes as
        % possible, or the numbers of cells across the wall, around it and
        % along the height (1x3). The default cuts a thin wall into cells
        % longer than it is thick, far fewer than cubes would take.
        ElementResolution (1, :) double {mustBeResolution} = [1 24 8]
    end

    properties (Dependent)
        % Inner radii at the negative and the positive end of Axis (as
        % InnerDiameter)
        InnerRadius (1, :) double {mustBeBoreSize}

        % Outer diameters at the negative and the positive end of Axis
        % The bore plus twice the wall; setting it keeps WallThickness and
        % moves the bore.
        Diameter (1, :) double

        % Outer radii at the negative and the positive end of Axis (as
        % Diameter)
        Radius (1, :) double
    end

    methods
        function obj = Tube(Options)
            arguments
                Options.?phx.shape.Tube
            end

            % Process input arguments
            obj = phx.internal.applyArguments(Options, obj);
            if isnan(obj.Color(1))
                obj.Color = phx.base.Shape.newColor;
            end
        end

        function obj = set.InnerDiameter(obj, diameter)
            if isscalar(diameter)
                diameter = [diameter diameter];
            end
            obj.InnerDiameter = diameter;
        end

        function diameter = get.InnerDiameter(obj)
            % A tube saved by PHX 1.0 kept a single diameter
            diameter = obj.InnerDiameter;
            if isscalar(diameter)
                diameter = [diameter diameter];
            end
        end

        function obj = set.InnerRadius (obj, radius)
            obj.InnerDiameter = radius*2;
        end

        function radius = get.InnerRadius (obj)
            radius = obj.InnerDiameter/2;
        end

        function obj = set.Diameter (obj, diameter)
            obj.InnerDiameter = diameter - obj.WallThickness*2;
        end

        function diameter = get.Diameter (obj)
            diameter = obj.InnerDiameter + obj.WallThickness*2;
        end

        function obj = set.Radius (obj, radius)
            obj.Diameter = radius*2;
        end

        function radius = get.Radius (obj)
            radius = obj.Diameter/2;
        end

        function drawTo(obj, target)
            [V, N, F, T] = phx.internal.Geometry.revolution(obj.wallProfile, obj.Segments, false, false);
            [V, N] = phx.internal.Geometry.switchZAxis(obj.Axis, V, N);

            primitive = obj.drawMesh(target, V, N, F, T, obj.getTexture);
            setappdata(primitive, "phxShape", obj);
        end

        function [mass, inertia, center] = computeMass(obj)
            [V, ~, F] = phx.internal.Geometry.revolution(obj.wallProfile, obj.Segments, false, false);
            V = phx.internal.Geometry.switchZAxis(obj.Axis, V);

            [mass, ~, Icm, center] = phx.internal.Geometry.meshMass(V, F, obj.Density);
            inertia = Icm([1 5 9]);
        end

        function [V, T] = generateVolumetricMesh(obj)
        %generateVolumetricMesh Returns a mesh of tetrahedra that fills the tube wall.
        %
        %   [V, T] = generateVolumetricMesh(tube) returns a mesh of
        %   tetrahedra that fills the tube wall, as phx.FlexibleBody takes
        %   it: its vertices V (Mx3, in the shape frame) and its quadratic
        %   tetrahedra T (Kx10 vertex indices: the corners, of positive
        %   volume, then the vertices in the middles of the edges 12 23 13
        %   14 24 34). The wall is cut into a polar grid of cells across
        %   it, around it and along it, as many as ElementResolution sets;
        %   each cell is split into six tetrahedra. A bore closed at an end
        %   closes onto the axis there. The vertices in the middles of the
        %   edges on the outer surface and on the bore are moved onto them.
        %
        %   See also phx.FlexibleBody, phx.shape.Tube.ElementResolution,
        %   phx.shape.Tube.generateSurfaceMesh

            bore = obj.InnerRadius;
            t = obj.WallThickness;
            h = obj.Height;
            n = obj.divisions;
            [V, T] = phx.internal.Geometry.polarTube(n(1), n(2), n(3), bore, t, h);

            % The edges on the surface with both ends on the outer surface,
            % or both on the bore, have their middles moved radially onto it
            inner = @(z) bore(1) + (bore(2) - bore(1))*(z/h + 0.5);
            F = sort([T(:, [1 2 3]); T(:, [1 2 4]); T(:, [1 3 4]); T(:, [2 3 4])], 2);
            [faces, ~, k] = unique(F, "rows");
            outer = faces(accumarray(k, 1) == 1, :);
            surface = sort([outer(:, [1 2]); outer(:, [2 3]); outer(:, [1 3])], 2);
            rho = sqrt(sum(V(:, 1:2).^2, 2));
            onBore = abs(rho - inner(V(:, 3))) < 1e-12*(max(bore) + t);
            onOutside = abs(rho - inner(V(:, 3)) - t) < 1e-12*(max(bore) + t);
            bores = surface(all(onBore(surface), 2), :);
            outsides = surface(all(onOutside(surface), 2), :);
            corners = size(V, 1);
            [V, T, ends] = phx.internal.Geometry.quadraticTetrahedra(V, T);
            for part = {bores, 0; outsides, t}'
                on = corners + find(ismember(ends, part{1}, "rows"));
                rho = sqrt(sum(V(on, 1:2).^2, 2));
                on = on(rho > 0);
                V(on, 1:2) = (inner(V(on, 3)) + part{2}).*V(on, 1:2)./sqrt(sum(V(on, 1:2).^2, 2));
            end
            V = phx.internal.Geometry.switchZAxis(obj.Axis, V);
        end

        function [V, F] = generateSurfaceMesh(obj)
        %generateSurfaceMesh Returns a triangle mesh of the surface of the tube wall.
        %
        %   [V, F] = generateSurfaceMesh(tube) returns a triangle mesh of
        %   the surface of the tube wall, as phx.SoftBody takes it: its
        %   vertices V (Nx3, in the shape frame) and its triangles F (Mx3
        %   vertex indices, counter-clockwise seen from outside). The outer
        %   surface, the ends and the bore are covered by rings of vertices,
        %   each with as many as its circumference takes, so that the
        %   vertices are spaced about evenly as ElementResolution sets.
        %
        %   See also phx.SoftBody, phx.shape.Tube.ElementResolution,
        %   phx.shape.Tube.generateVolumetricMesh

            bore = obj.InnerRadius;
            t = obj.WallThickness;
            h = obj.Height;
            n = obj.divisions;
            spacing = [h/n(3) 2*pi*(mean(bore) + t/2)/n(2)];

            % The closed profile of the wall: up the outside, in across the
            % top, down the bore and out across the bottom
            profile = [bore(1) + t -h/2; bore(2) + t h/2; bore(2) h/2; bore(1) -h/2; bore(1) + t -h/2];
            [V, F] = phx.internal.Geometry.ringSurface(profile, spacing);
            V = phx.internal.Geometry.switchZAxis(obj.Axis, V);
        end
    end

    methods (Access = private)
        function ZX = wallProfile(obj)
            % Closed [z x] loop of the tube wall. It is traversed outer wall
            % first so that the outer surface, both end faces and the bore all
            % come out facing away from the material.
            rb = obj.InnerDiameter/2;
            t = obj.WallThickness;
            h = obj.Height/2;
            ZX = [-h rb(1) + t; h rb(2) + t; h rb(2); -h rb(1); -h rb(1) + t];
        end

        function n = divisions(obj)
            % Cells across the wall, around it and along it: a scalar
            % ElementResolution across the thinner of the wall and the
            % height, cells about cubes around the middle of the wall
            n = obj.ElementResolution;
            if isscalar(n)
                t = obj.WallThickness;
                h = obj.Height;
                step = min(t, h)/n;
                around = 2*pi*(mean(obj.InnerRadius) + t/2);
                n = [max(1, round(t/step)) max(3, round(around/step)) max(1, round(h/step))];
            end
        end
    end

    methods (Access = {?phx.base.Shape, ?phx.base.Object})
        function eh = createBody(obj, body, primitive)
            switch obj.Envelope
                case "cylinder"
                    % Bounding cylinder (centered on the shape origin)
                    eh = phxEngine('add', body.WorldHandle, 'cylinder', body.TypeID, max(obj.Radius), obj.Height, char(obj.Axis), body.Transform, body.Mass, body.Inertia, body.CenterOfMass);
                case "convex"
                    ph = phx.internal.PrimitiveHelper(primitive);
                    vertices = ph.Vertices';
                    eh = obj.createHullBody(body, vertices);
                case "concave"
                    ph = phx.internal.PrimitiveHelper(primitive);
                    vertices = ph.Vertices';
                    eh = obj.createTrimeshBody(body, vertices, ph.LinearizedFaces);
            end
        end
    end

end

function mustBeBoreSize(value)
%mustBeBoreSize Validates InnerDiameter and InnerRadius: one or two finite, nonnegative values
    if ~any(numel(value) == [1 2]) || any(~isfinite(value)) || any(value < 0)
        error("phx:Tube:badSize", ...
            "The bore must be one or two finite, nonnegative values (Diameter at least twice WallThickness).");
    end
end

function mustBeResolution(value)
%mustBeResolution Validates ElementResolution: one or three positive whole numbers
    if ~any(numel(value) == [1 3]) || any(value ~= round(value)) || any(value < 1) || any(~isfinite(value))
        error("phx:Tube:badResolution", ...
            "ElementResolution must be a number of cells across the thinner of the wall and the height, or the numbers of cells across the wall, around it and along the height (1x3), whole numbers of at least 1.");
    end
end
