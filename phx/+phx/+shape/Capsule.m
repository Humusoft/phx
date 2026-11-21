classdef Capsule < phx.base.Shape
%phx.shape.Capsule Capsule shape
%
%   Visual appearance is based on a geometry of revolved profile with 
%   an adjustable level of details.
%
%   Collision shape and mass properties are modeled as an ideal capsule
%   independent on visual details.
%
%   As the Shape of a phx.FlexibleBody, the capsule is filled with a mesh
%   of tetrahedra (generateVolumetricMesh); as the Shape of a phx.SoftBody,
%   it is covered with a mesh of evenly spaced vertices
%   (generateSurfaceMesh). The ElementResolution sets how fine they are.
%
%   phx.shape.Capsule() creates a shape with default parameters.
%
%   phx.shape.Capsule(Name, Value, ...) creates a shape and sets properties
%   values according to given name-value pairs.
%
%   See also phx.Body, phx.FlexibleBody, phx.SoftBody, phx.shape.Cone

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

%#ok<*INUSD> OK to see the full list of arguments for callbacks

    properties
        % Capsule diameter
        Diameter (1, 1) double = 1

        % Capsule height
        Height (1, 1) double = 1

        % Modeling axis of the capsule
        Axis {mustBeMember(Axis, ["x", "y", "z"])} = "z"

        % Number of Capsule segments
        Segments (1, 1) double = 24

        % Resolution of the meshes of flexible and soft bodies: the number
        % of cells across the diameter, the cylindrical part cut into cells
        % as close to cubes as possible, or the numbers of cells across the
        % diameter and along the Height of the cylindrical part (1x2)
        ElementResolution (1, :) double {mustBeResolution} = 2
    end

    properties (Dependent)
        % Capsule radius
        Radius (1, 1) double
    end

    methods
        function obj = Capsule(Options)
            arguments
                Options.?phx.shape.Capsule
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
            sh = sin(linspace(0, pi/2, obj.Segments/3))*obj.Radius;
            sr = cos(linspace(0, pi/2, obj.Segments/3))*obj.Radius;
            sl = fliplr(sr);
            hr = sh + obj.Height/2;
            hl = -fliplr(hr);
            [V, N, F, T] = phx.internal.Geometry.revolution([hl hr; sl sr]', obj.Segments, true, true);
            [V, N] = phx.internal.Geometry.switchZAxis(obj.Axis, V, N);
            
            primitive = obj.drawMesh(target, V, N, F, T, obj.getTexture);
            setappdata(primitive, "phxShape", obj);
        end

        function [volume, inertia, center] = computeVolume(obj)
            % The cylinder and the two halves of the sphere (at unit density,
            % their masses are their volumes)
            r = obj.Diameter/2;
            rq = r^2;
            h = obj.Height;
            mc = pi*rq*h;
            ms = pi*r^3*4/3;
            volume = mc + ms;
            is = rq*ms*2/5;
            ia = mc*rq/2 + is;
            io = mc*(h^2 + 3*rq)/12 + is + ms*(h/2)^2;
            inertia = phx.internal.Geometry.switchZAxis(obj.Axis, [io io ia]);
            center = [0 0 0];
        end

        function [V, T] = generateVolumetricMesh(obj)
        %generateVolumetricMesh Returns a mesh of tetrahedra that fills the capsule.
        %
        %   [V, T] = generateVolumetricMesh(capsule) returns a mesh of
        %   tetrahedra that fills the capsule, as phx.FlexibleBody takes it:
        %   its vertices V (Mx3, in the shape frame) and its quadratic
        %   tetrahedra T (Kx10 vertex indices: the corners, of positive
        %   volume, then the vertices in the middles of the edges 12 23 13
        %   14 24 34). The cylindrical part is meshed as phx.shape.Cylinder
        %   does and each end as half of the mesh of phx.shape.Sphere,
        %   joined face to face, with about as many cells across the
        %   diameter and along the cylindrical part as ElementResolution
        %   sets; each cell is split into six tetrahedra. The vertices in
        %   the middles of the edges on the curved surface are moved onto
        %   it.
        %
        %   See also phx.FlexibleBody, phx.shape.Capsule.ElementResolution,
        %   phx.shape.Capsule.generateSurfaceMesh

            r = obj.Diameter/2;
            h = obj.Height;
            n = obj.ElementResolution;
            if isscalar(n)
                n = [n round(h/(2*r)*n)];
            end

            % Half the cells across the diameter in the core, a quarter in
            % each side block, a core of 0.4 of the radius: the cylinder and
            % the half balls match where they join
            core = max(1, round(n(1)/2));
            side = max(1, round((n(1) - core)/2));
            [Vb, Tb] = phx.internal.Geometry.cubedHalfBall(core, max(1, round(core/2)), side, 0.4);
            Vb = r*Vb;
            V = [Vb + [0 0 h/2]; Vb.*[1 1 -1] - [0 0 h/2]];
            T = [Tb; size(Vb, 1) + Tb];
            if h > 0
                [Vc, Tc] = phx.internal.Geometry.cubedCylinder(core, side, max(1, n(2)), 0.4);
                T = [T; size(V, 1) + Tc];
                V = [V; Vc.*[r r h]];
            end

            % The points the parts share merged, the tetrahedra turned to a
            % positive volume (the lower end is mirrored)
            [V, ~, map] = uniquetol(V, 1e-12, "ByRows", true, "DataScale", r + h);
            T = map(T);
            e = V(T(:, 1), :);
            negative = dot(cross(V(T(:, 2), :) - e, V(T(:, 3), :) - e, 2), V(T(:, 4), :) - e, 2) < 0;
            T(negative, [2 3]) = T(negative, [3 2]);

            % The middles of the edges on the surface moved onto it: radially
            % along the cylindrical part, from the centre of the end beyond
            F = sort([T(:, [1 2 3]); T(:, [1 2 4]); T(:, [1 3 4]); T(:, [2 3 4])], 2);
            [faces, ~, k] = unique(F, "rows");
            outer = faces(accumarray(k, 1) == 1, :);
            surface = sort([outer(:, [1 2]); outer(:, [2 3]); outer(:, [1 3])], 2);
            axial = @(P) P - [0 0 1].*min(max(P(:, 3), -h/2), h/2);
            onSurface = abs(sqrt(sum(axial(V).^2, 2)) - r) < 1e-12*r;
            surface = surface(all(onSurface(surface), 2), :);
            corners = size(V, 1);
            [V, T, ends] = phx.internal.Geometry.quadraticTetrahedra(V, T);
            on = corners + find(ismember(ends, surface, "rows"));
            d = axial(V(on, :));
            V(on, :) = V(on, :) - d + r*d./sqrt(sum(d.^2, 2));
            V = phx.internal.Geometry.switchZAxis(obj.Axis, V);
        end

        function [V, F] = generateSurfaceMesh(obj)
        %generateSurfaceMesh Returns a triangle mesh of the surface of the capsule.
        %
        %   [V, F] = generateSurfaceMesh(capsule) returns a triangle mesh of
        %   the surface of the capsule, as phx.SoftBody takes it: its
        %   vertices V (Nx3, in the shape frame) and its triangles F (Mx3
        %   vertex indices, counter-clockwise seen from outside). The
        %   surface is covered by rings of vertices, each with as many as
        %   its circumference takes, so that the vertices are spaced about
        %   evenly as ElementResolution sets and a soft body spreads its
        %   mass evenly over the surface.
        %
        %   See also phx.SoftBody, phx.shape.Capsule.ElementResolution,
        %   phx.shape.Capsule.generateVolumetricMesh

            r = obj.Diameter/2;
            h = obj.Height;
            n = obj.ElementResolution;
            if isscalar(n)
                spacing = 2*r/n;
            else
                spacing = [h/n(2) 2*r/n(1)];
            end

            % The profile from the lower pole along the end, up the side and
            % along the upper end to its pole; the arcs cut into parts
            parts = max(2, round(pi*r/2/spacing(end)));
            t = linspace(0, pi/2, parts + 1)';
            lower = [r*sin(t) -h/2 - r*cos(t)];
            lower([1 end], :) = [0 -h/2 - r; r -h/2];   % exact, to merge at Height 0
            upper = flipud(lower).*[1 -1];
            profile = unique([lower; upper], "rows", "stable");
            [V, F] = phx.internal.Geometry.ringSurface(profile, spacing);
            V = phx.internal.Geometry.switchZAxis(obj.Axis, V);
        end
    end

    methods (Access = {?phx.base.Shape, ?phx.base.Object})
        function eh = createBody(obj, body, primitive)
            eh = phxEngine('add', body.WorldHandle, 'capsule', body.TypeID, obj.Diameter/2, obj.Height, char(obj.Axis), body.Transform, body.Mass, body.Inertia, body.CenterOfMass);
        end
    end

end

function mustBeResolution(value)
%mustBeResolution Validates ElementResolution: one or two positive whole numbers
    if ~any(numel(value) == [1 2]) || any(value ~= round(value)) || any(value < 1) || any(~isfinite(value))
        error("phx:Capsule:badResolution", ...
            "ElementResolution must be a number of cells across the diameter, or the numbers of cells across the diameter and along the Height of the cylindrical part (1x2), whole numbers of at least 1.");
    end
end