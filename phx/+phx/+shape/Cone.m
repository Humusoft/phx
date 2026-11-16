classdef Cone < phx.base.Shape
%phx.shape.Cone Cone or truncated cone shape
%
%   Visual appearance is based on a geometry of revolved profile with
%   an adjustable level of details.
%
%   Collision shape and mass properties are modeled as an ideal cone
%   independent on visual details. Diameter gives the diameters at the
%   negative and the positive end of Axis; a single value is a sharp cone
%   with its apex on the positive axis, two values a truncated cone (or a
%   cylinder when they are equal).
%
%   As the Shape of a phx.FlexibleBody, the cone is filled with a mesh of
%   tetrahedra (generateVolumetricMesh); as the Shape of a phx.SoftBody, it
%   is covered with the surface of that mesh (generateSurfaceMesh). The
%   ElementResolution sets how fine they are.
%
%   phx.shape.Cone() creates a shape with default parameters.
%
%   phx.shape.Cone(Name, Value, ...) creates a shape and sets properties
%   values according to given name-value pairs.
%
%   See also phx.Body, phx.FlexibleBody, phx.SoftBody, phx.shape.Cylinder

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

%#ok<*INUSD> OK to see the full list of arguments for callbacks

    properties
        % Diameters at the negative and the positive end of Axis
        % A single value d is [d 0], a sharp cone with its apex on the
        % positive axis; two values make a truncated cone.
        Diameter (1, :) double {mustBeConeSize} = [1 0]

        % Cone height
        Height (1, 1) double = 1

        % Modeling axis of the cone
        Axis {mustBeMember(Axis, ["x", "y", "z"])} = "z"

        % Number of cone segments
        Segments (1, 1) double = 24

        % Volumetric density (kg/m^3)
        Density (1, 1) double = 1000

        % Resolution of the meshes of flexible and soft bodies: the number
        % of cells across the smaller of the larger diameter and the height,
        % the other one cut into cells as close to cubes as possible, or the
        % numbers of cells across the larger diameter and along the height
        % (1x2)
        ElementResolution (1, :) double {mustBeResolution} = 2
    end

    properties (Dependent)
        % Radii at the negative and the positive end of Axis (as Diameter)
        Radius (1, :) double {mustBeConeSize}
    end

    methods
        function obj = Cone(Options)
            arguments
                Options.?phx.shape.Cone
            end

            % Process input arguments
            obj = phx.internal.applyArguments(Options, obj);
            if isnan(obj.Color(1))
                obj.Color = phx.base.Shape.newColor;
            end
        end

        function obj = set.Diameter(obj, diameter)
            if isscalar(diameter)
                diameter = [diameter 0];
            end
            obj.Diameter = diameter;
        end

        function diameter = get.Diameter(obj)
            % A cone saved by PHX 1.0 kept a single diameter
            diameter = obj.Diameter;
            if isscalar(diameter)
                diameter = [diameter 0];
            end
        end

        function obj = set.Radius (obj, radius)
            obj.Diameter = radius*2;
        end

        function radius = get.Radius (obj)
            radius = obj.Diameter/2;
        end

        function drawTo(obj, target)
            % The profile from the negative to the positive end, each end
            % capped unless it is an apex
            d = obj.Diameter;
            [V, N, F, T] = phx.internal.Geometry.revolution([-0.5 d(1)/2; 0.5 d(2)/2], obj.Segments, d(1) > 0, d(2) > 0);
            V = V.*[1 1 obj.Height];
            [V, N] = phx.internal.Geometry.switchZAxis(obj.Axis, V, N);

            primitive = obj.drawMesh(target, V, N, F, T, obj.getTexture);
            setappdata(primitive, "phxShape", obj);
        end

        function [mass, inertia, center] = computeMass(obj)
            % A solid of revolution of radius r(z), linear in z, integrated
            % by the 3-point Gauss rule, exact for the polynomials of up to
            % the fifth degree here: mass, centre of mass zc, the moment
            % about the axis and the transverse one about zc
            r = obj.Radius;
            h = obj.Height;
            s = [-sqrt(3/5) 0 sqrt(3/5)];
            g = [5 8 5]/18;
            z = s*h/2;
            rz = r(1) + (r(2) - r(1))*(z/h + 0.5);
            dm = obj.Density*pi*rz.^2.*g*h;
            mass = sum(dm);
            zc = sum(dm.*z)/mass;
            ia = sum(dm.*rz.^2/2);
            io = sum(dm.*(rz.^2/4 + (z - zc).^2));
            inertia = phx.internal.Geometry.switchZAxis(obj.Axis, [io io ia]);
            center = phx.internal.Geometry.switchZAxis(obj.Axis, [0 0 zc]);
        end

        function [V, T] = generateVolumetricMesh(obj)
        %generateVolumetricMesh Returns a mesh of tetrahedra that fills the cone.
        %
        %   [V, T] = generateVolumetricMesh(cone) returns a mesh of
        %   tetrahedra that fills the cone, as phx.FlexibleBody takes it: its
        %   vertices V (Mx3, in the shape frame) and its quadratic
        %   tetrahedra T (Kx10 vertex indices: the corners, of positive
        %   volume, then the vertices in the middles of the edges 12 23 13
        %   14 24 34). Its cross-section is a square core surrounded by four
        %   blocks of cells reaching the circle, stacked in layers along the
        %   axis and narrowing with the cone, about as many cells across the
        %   larger diameter and along the height as ElementResolution sets;
        %   each cell is split into six tetrahedra. At the apex of a sharp
        %   cone the last layer closes into pyramids, whose tetrahedra are
        %   as slender as the cone itself. The vertices in the middles of
        %   the edges on the curved surface are moved onto it.
        %
        %   See also phx.FlexibleBody, phx.shape.Cone.ElementResolution,
        %   phx.base.Shape.generateSurfaceMesh

            radii = obj.Radius;
            r = max(radii);
            h = obj.Height;
            n = obj.ElementResolution;
            if isscalar(n)
                n = max(1, round([2*r h]*n/min(2*r, h)));
            end

            % Half the cells across the diameter in the core, a quarter in
            % each side block; a core of 0.4 of the radius, as in a cylinder
            core = max(1, round(n(1)/2));
            side = max(1, round((n(1) - core)/2));
            [V, T] = phx.internal.Geometry.cubedCylinder(core, side, n(2), 0.4, radii/r);
            V = V.*[r r h];

            % The edges on the surface with both ends on the curved side
            % have their middles moved radially onto it
            radius = @(z) radii(1) + (radii(2) - radii(1))*(z/h + 0.5);
            F = sort([T(:, [1 2 3]); T(:, [1 2 4]); T(:, [1 3 4]); T(:, [2 3 4])], 2);
            [faces, ~, k] = unique(F, "rows");
            outer = faces(accumarray(k, 1) == 1, :);
            surface = sort([outer(:, [1 2]); outer(:, [2 3]); outer(:, [1 3])], 2);
            onSide = abs(sqrt(sum(V(:, 1:2).^2, 2)) - radius(V(:, 3))) < 1e-12*r;
            surface = surface(all(onSide(surface), 2), :);
            corners = size(V, 1);
            [V, T, ends] = phx.internal.Geometry.quadraticTetrahedra(V, T);
            on = corners + find(ismember(ends, surface, "rows"));
            rho = sqrt(sum(V(on, 1:2).^2, 2));
            on = on(rho > 0);
            V(on, 1:2) = radius(V(on, 3)).*V(on, 1:2)./sqrt(sum(V(on, 1:2).^2, 2));
            V = phx.internal.Geometry.switchZAxis(obj.Axis, V);
        end

        function [V, F] = generateSurfaceMesh(obj)
        %generateSurfaceMesh Returns a triangle mesh of the surface of the cone.
        %
        %   [V, F] = generateSurfaceMesh(cone) returns a triangle mesh of the
        %   surface of the cone, as phx.SoftBody takes it: its vertices V
        %   (Nx3, in the shape frame) and its triangles F (Mx3 vertex
        %   indices, counter-clockwise seen from outside). The side and the
        %   flat ends are covered by rings of vertices, each with as many as
        %   its circumference takes, so that the vertices are spaced about
        %   evenly as ElementResolution sets and a soft body spreads its
        %   mass evenly over the surface.
        %
        %   See also phx.SoftBody, phx.shape.Cone.ElementResolution,
        %   phx.shape.Cone.generateVolumetricMesh

            radii = obj.Radius;
            r = max(radii);
            h = obj.Height;
            n = obj.ElementResolution;
            if isscalar(n)
                spacing = min(2*r, h)/n;
            else
                spacing = [h/n(2) 2*r/n(1)];
            end

            % The profile from the axis at the bottom, out and up the side,
            % and back to the axis at the top; an apex appears only once
            profile = unique([0 -h/2; radii(1) -h/2; radii(2) h/2; 0 h/2], "rows", "stable");
            [V, F] = phx.internal.Geometry.ringSurface(profile, spacing);
            V = phx.internal.Geometry.switchZAxis(obj.Axis, V);
        end
    end

    methods (Access = {?phx.base.Shape, ?phx.base.Object})
        function eh = createBody(obj, body, primitive)
            eh = phxEngine('add', body.WorldHandle, 'cone', body.TypeID, obj.Radius, obj.Height, char(obj.Axis), body.Transform, body.Mass, body.Inertia, body.CenterOfMass);
        end
    end

end

function mustBeConeSize(value)
%mustBeConeSize Validates Diameter and Radius: one or two values, not negative, not both zero
    if ~any(numel(value) == [1 2]) || any(~isfinite(value)) || any(value < 0) || ~any(value > 0)
        error("phx:Cone:badSize", ...
            "Diameter (Radius) must be one or two finite, nonnegative values, not all zero.");
    end
end

function mustBeResolution(value)
%mustBeResolution Validates ElementResolution: one or two positive whole numbers
    if ~any(numel(value) == [1 2]) || any(value ~= round(value)) || any(value < 1) || any(~isfinite(value))
        error("phx:Cone:badResolution", ...
            "ElementResolution must be a number of cells across the smaller of the larger diameter and the height, or the numbers of cells across the larger diameter and along the height (1x2), whole numbers of at least 1.");
    end
end