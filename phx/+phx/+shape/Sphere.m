classdef Sphere < phx.base.Shape
%phx.shape.Sphere Sphere shape
%
%   Visual appearance is based on a latitude-longitude grid with an
%   adjustable level of details. A texture is mapped as a standard 2:1
%   equirectangular image: its top row at the +z pole, its left and right
%   edges on the +x meridian, longitude growing anticlockwise about +z.
%
%   Collision shape and mass properties are modeled as an ideal sphere
%   independent on visual details.
%
%   As the Shape of a phx.FlexibleBody, the sphere is filled with a mesh of
%   tetrahedra (generateVolumetricMesh); as the Shape of a phx.SoftBody, it
%   is covered with a geodesic mesh (generateSurfaceMesh). The
%   ElementResolution sets how fine they are.
%
%   phx.shape.Sphere() creates a shape with default parameters.
%
%   phx.shape.Sphere(Name, Value, ...) creates a shape and sets properties
%   values according to given name-value pairs.
%
%   See also phx.Body, phx.SoftBody, phx.shape.Rock

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

%#ok<*INUSD> OK to see the full list of arguments for callbacks

    properties
        % Sphere diameter
        Diameter (1, 1) double = 1

        % Number of meridian segments; the parallels follow at the same
        % angular step
        Segments (1, 1) double = 28

        % Resolution of the meshes of flexible and soft bodies: about the
        % number of mesh edges across the diameter
        ElementResolution (1, :) double {mustBeResolution} = 5
    end

    properties (Dependent)
        % Sphere radius
        Radius (1, 1) double
    end

    methods
        function obj = Sphere(Options)
            arguments
                Options.?phx.shape.Sphere
            end

            % Process input arguments
            obj = phx.internal.applyArguments(Options, obj);
            if isnan(obj.Color(1))
                obj.Color = phx.base.Shape.newColor;
            end
        end

        function obj = set.Radius(obj, radius)
            obj.Diameter = radius*2;
        end

        function radius = get.Radius(obj)
            radius = obj.Diameter/2;
        end

        function drawTo(obj, target)
            [V, N, F, T] = phx.internal.Geometry.sphere(obj.Diameter, obj.Segments);
            primitive = obj.drawMesh(target, V, N, F, T, obj.getTexture);
            setappdata(primitive, "phxShape", obj);
        end

        function [volume, inertia, center] = computeVolume(obj)
            r = obj.Diameter/2;
            volume = pi*r^3*4/3;
            inertia = [1 1 1]*r^2*volume*2/5;
            center = [0 0 0];
        end

        function [V, T] = generateVolumetricMesh(obj)
        %generateVolumetricMesh Returns a mesh of tetrahedra that fills the sphere.
        %
        %   [V, T] = generateVolumetricMesh(sphere) returns a mesh of
        %   tetrahedra that fills the sphere, as phx.FlexibleBody takes it:
        %   its vertices V (Mx3, in the shape frame) and its quadratic
        %   tetrahedra T (Kx10 vertex indices: the corners, of positive
        %   volume, then the vertices in the middles of the edges 12 23 13
        %   14 24 34). A cube core is surrounded by six blocks of cells
        %   reaching the sphere, about ElementResolution cells across the
        %   diameter; each cell is split into six tetrahedra. The vertices
        %   in the middles of the edges on the surface are moved onto the
        %   sphere, so that the curved tetrahedra follow it even when they
        %   are coarse.
        %
        %   See also phx.FlexibleBody, phx.shape.Sphere.ElementResolution,
        %   phx.shape.Sphere.generateSurfaceMesh

            % Half the cells along the diameter in the core, a quarter in
            % each shell; a core of 0.4 of the radius keeps the tetrahedra
            % at the corners of the core least distorted
            N = obj.ElementResolution;
            core = max(1, round(N/2));
            shell = max(1, round((N - core)/2));
            [V, T] = phx.internal.Geometry.cubedBall(core, shell, 0.4);
            r = obj.Diameter/2;
            V = V*r;

            % The edges of the faces that only one tetrahedron has lie on
            % the surface; their middles go onto the sphere
            F = sort([T(:, [1 2 3]); T(:, [1 2 4]); T(:, [1 3 4]); T(:, [2 3 4])], 2);
            [faces, ~, k] = unique(F, "rows");
            outer = faces(accumarray(k, 1) == 1, :);
            surface = sort([outer(:, [1 2]); outer(:, [2 3]); outer(:, [1 3])], 2);
            corners = size(V, 1);
            [V, T, ends] = phx.internal.Geometry.quadraticTetrahedra(V, T);
            on = corners + find(ismember(ends, surface, "rows"));
            V(on, :) = r*V(on, :)./sqrt(sum(V(on, :).^2, 2));
        end

        function [V, F] = generateSurfaceMesh(obj)
        %generateSurfaceMesh Returns a geodesic triangle mesh of the sphere.
        %
        %   [V, F] = generateSurfaceMesh(sphere) returns a geodesic mesh of
        %   the sphere, as phx.SoftBody takes it: its vertices V (Nx3, in the
        %   shape frame) and its triangles F (Mx3 vertex indices,
        %   counter-clockwise seen from outside). Each edge of a regular
        %   icosahedron is cut into as many parts as make the edges about
        %   Diameter/ElementResolution long, each face into triangles of
        %   about the same size, and the vertices are projected on the
        %   sphere.
        %
        %   See also phx.SoftBody, phx.shape.Sphere.ElementResolution,
        %   phx.base.Shape.generateVolumetricMesh

            % The edge of the icosahedron is 1.05 of its circumradius
            edge = 4/sqrt(10 + 2*sqrt(5));
            [V, F] = phx.internal.Geometry.geodesicSphere(max(1, round(obj.ElementResolution*edge/2)));
            V = V*obj.Diameter/2;
        end
    end

    methods (Access = {?phx.base.Shape, ?phx.base.Object})
        function eh = createBody(obj, body, primitive)
            eh = phxEngine('add', body.WorldHandle, 'sphere', body.TypeID, obj.Radius, body.Transform, body.Mass, body.Inertia, body.CenterOfMass);
        end
    end

end

function mustBeResolution(value)
%mustBeResolution Validates ElementResolution: one positive whole number
    if ~isscalar(value) || value ~= round(value) || value < 1 || ~isfinite(value)
        error("phx:Sphere:badResolution", ...
            "ElementResolution must be a number of mesh edges across the diameter, a whole number of at least 1.");
    end
end
