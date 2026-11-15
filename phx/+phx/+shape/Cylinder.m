classdef Cylinder < phx.base.Shape
%phx.shape.Cylinder Cylinder shape
%
%   Visual appearance is based on a geometry of revolved profile with 
%   an adjustable level of details.
%
%   Collision shape and mass properties are modeled as an ideal cylinder
%   independent on visual details.
%
%   As the Shape of a phx.FlexibleBody, the cylinder is filled with a mesh
%   of tetrahedra (generateVolumetricMesh); as the Shape of a phx.SoftBody,
%   it is covered with the surface of that mesh (generateSurfaceMesh). The
%   ElementResolution sets how fine they are.
%
%   phx.shape.Cylinder() creates a shape with default parameters.
%
%   phx.shape.Cylinder(Name, Value, ...) creates a shape and sets properties
%   values according to given name-value pairs.
%
%   See also phx.Body, phx.FlexibleBody, phx.SoftBody, phx.shape.Cone

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

%#ok<*INUSD> OK to see the full list of arguments for callbacks

    properties
        % Cylinder diameter
        Diameter (1, 1) double = 1

        % Cylinder height
        Height (1, 1) double = 1

        % Modeling axis of the cylinder
        Axis {mustBeMember(Axis, ["x", "y", "z"])} = "z"

        % Number of cylinder segments
        Segments (1, 1) double = 24

        % Volumetric density (kg/m^3)
        Density (1, 1) double = 1000

        % Resolution of the meshes of flexible and soft bodies: the number
        % of cells across the smaller of the diameter and the height, the
        % other one cut into cells as close to cubes as possible, or the
        % numbers of cells across the diameter and along the height (1x2)
        ElementResolution (1, :) double {mustBeResolution} = 2
    end

    properties (Dependent)
        % Cylinder radius
        Radius (1, 1) double
    end

    methods
        function obj = Cylinder(Options)
            arguments
                Options.?phx.shape.Cylinder
            end

            % Process input arguments
            obj = phx.internal.applyArguments(Options, obj);
            if isnan(obj.Color(1))
                obj.Color = phx.base.Shape.newColor;
            end
        end

        function obj = set.Radius (obj, radius)
            obj.Diameter = radius*2;
        end

        function radius = get.Radius (obj)
            radius = obj.Diameter/2;
        end

        function drawTo(obj, target)
            [V, N, F, T] = phx.internal.Geometry.revolution([-0.5 0.5; 0.5 0.5], obj.Segments, true, true);
            V = V.*[obj.Diameter obj.Diameter obj.Height];
            [V, N] = phx.internal.Geometry.switchZAxis(obj.Axis, V, N);
            
            primitive = obj.drawMesh(target, V, N, F, T, obj.getTexture);
            setappdata(primitive, "phxShape", obj);
        end

        function [mass, inertia, center] = computeMass(obj)
            rq = (obj.Diameter/2)^2;
            h = obj.Height;
            mass = obj.Density*pi*rq*h;
            ia = mass*rq/2;
            io = mass*(h^2 + 3*rq)/12;
            inertia = phx.internal.Geometry.switchZAxis(obj.Axis, [io io ia]);
            center = [0 0 0];
        end

        function [V, T] = generateVolumetricMesh(obj)
        %generateVolumetricMesh Returns a mesh of tetrahedra that fills the cylinder.
        %
        %   [V, T] = generateVolumetricMesh(cylinder) returns a mesh of
        %   tetrahedra that fills the cylinder, as phx.FlexibleBody takes
        %   it: its vertices V (Mx3, in the shape frame) and its quadratic
        %   tetrahedra T (Kx10 vertex indices: the corners, of positive
        %   volume, then the vertices in the middles of the edges 12 23 13
        %   14 24 34). Its cross-section is a square core surrounded by four
        %   blocks of cells reaching the circle, stacked in layers along the
        %   axis, about as many cells across the diameter and along the
        %   height as ElementResolution sets; each cell is split into six
        %   tetrahedra. The vertices in the middles of the edges on the
        %   curved surface are moved onto it, so that the curved tetrahedra
        %   follow it even when they are coarse.
        %
        %   See also phx.FlexibleBody, phx.shape.Cylinder.ElementResolution,
        %   phx.base.Shape.generateSurfaceMesh

            r = obj.Diameter/2;
            h = obj.Height;
            n = obj.ElementResolution;
            if isscalar(n)
                n = max(1, round([2*r h]*n/min(2*r, h)));
            end

            % Half the cells across the diameter in the core, a quarter in
            % each side block; a core of 0.4 of the radius, as in a sphere
            core = max(1, round(n(1)/2));
            side = max(1, round((n(1) - core)/2));
            [V, T] = phx.internal.Geometry.cubedCylinder(core, side, n(2), 0.4);
            V = V.*[r r h];

            % The edges on the surface with both ends on the curved side
            % have their middles moved radially onto it
            F = sort([T(:, [1 2 3]); T(:, [1 2 4]); T(:, [1 3 4]); T(:, [2 3 4])], 2);
            [faces, ~, k] = unique(F, "rows");
            outer = faces(accumarray(k, 1) == 1, :);
            surface = sort([outer(:, [1 2]); outer(:, [2 3]); outer(:, [1 3])], 2);
            onSide = abs(sqrt(sum(V(:, 1:2).^2, 2)) - r) < 1e-12*r;
            surface = surface(all(onSide(surface), 2), :);
            corners = size(V, 1);
            [V, T, ends] = phx.internal.Geometry.quadraticTetrahedra(V, T);
            on = corners + find(ismember(ends, surface, "rows"));
            V(on, 1:2) = r*V(on, 1:2)./sqrt(sum(V(on, 1:2).^2, 2));
            V = phx.internal.Geometry.switchZAxis(obj.Axis, V);
        end
    end

    methods (Access = {?phx.base.Shape, ?phx.base.Object})
        function eh = createBody(obj, body, primitive)
            eh = phxEngine('add', body.WorldHandle, 'cylinder', body.TypeID, obj.Diameter/2, obj.Height, char(obj.Axis), body.Transform, body.Mass, body.Inertia, body.CenterOfMass);
        end
    end

end

function mustBeResolution(value)
%mustBeResolution Validates ElementResolution: one or two positive whole numbers
    if ~any(numel(value) == [1 2]) || any(value ~= round(value)) || any(value < 1) || any(~isfinite(value))
        error("phx:Cylinder:badResolution", ...
            "ElementResolution must be a number of cells across the smaller of the diameter and the height, or the numbers of cells across the diameter and along the height (1x2), whole numbers of at least 1.");
    end
end