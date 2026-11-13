classdef FlexibleBody < phx.base.Object
%phx.FlexibleBody Deformable solid made of tetrahedra
%
%   FlexibleBody is a solid of an elastic material given by its Young's
%   modulus and Poisson's ratio: a block of rubber or foam, a soft gripper
%   pad, a bushing, a beam. Its mesh of tetrahedra deforms as a fixed
%   corotated material, which also holds for large rotations; the vertices
%   carry the mass of the tetrahedra around them. It collides with rigid
%   bodies (phx.Body), not with soft or flexible bodies. No joint holds it
%   directly: a body welded into it with phx.IntersectionWeld holds and
%   loads it instead.
%
%   ElementOrder selects the elements. Quadratic tetrahedra, the default,
%   have a vertex in the middle of each edge, bend without the stiffening
%   of linear ones on a coarse mesh and are solved implicitly by Newton
%   iterations until the force left unbalanced falls below Tolerance, so
%   their stiffness does not depend on Iterations, which only limits the
%   iterations per step; two are usually enough, more only in a step with a
%   sudden load such as an impact. Linear tetrahedra are cheaper but solved
%   with as many solver substeps as Iterations gives, so a stiff material on
%   a fine mesh needs more of them and still comes out softer than it is.
%
%   phx.FlexibleBody(axes, Vertices=V, Tetrahedra=T) creates a flexible body
%   of the tetrahedral mesh V (Nx3, in the body frame) and T (vertex
%   indices) and draws its surface into the given axes object; [] draws
%   nothing. T is either Kx4, as delaunayTriangulation gives it, and the
%   vertices in the middles of the edges are added to V, or Kx10, a
%   quadratic mesh (the corners, then the vertices of the edges 12 23 13 14
%   24 34). The added vertices follow V in CurrentVertices and
%   VonMisesStress. The mesh is the rest shape, placed at Position and
%   Orientation when the simulation creates the body.
%
%   phx.FlexibleBody(axes, Mesh=mesh) takes the mesh of tetrahedra that PDE
%   Toolbox generates, for example of an STL file:
%
%       gm = generateMesh(fegeometry("part.stl"), Hmax=0.01);
%       flex = phx.FlexibleBody(gca, Mesh=gm.Mesh);
%
%   phx.FlexibleBody(___, ElementOrder="linear") creates a body of linear
%   tetrahedra from a Kx4 mesh.
%
%   phx.FlexibleBody(___, Name, Value, ...) sets properties values according
%   to given name-value pairs.
%
%   See also phx.SoftBody, phx.IntersectionWeld, phx.Body

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

%#ok<*MCSUP> OK to access other properties in setters
%#ok<*INUSD> OK to see the full list of arguments for callbacks

    properties (Access = private)
        hP
    end

    % Rest shape and material, read when the simulation creates the body
    properties
        % Vertices of the mesh in the body frame (Nx3)
        Vertices (:, 3) double {mustBeFinite} = zeros(0, 3)

        % Tetrahedra as vertex indices (Kx4, or Kx10 quadratic ones)
        Tetrahedra (:, :) double {mustBeInteger, mustBePositive} = zeros(0, 4)

        % Order of the elements: "quadratic" or "linear"
        ElementOrder (1, 1) string {mustBeMember(ElementOrder, ["linear", "quadratic"])} = "quadratic"

        % Position of the rest shape when the body is created
        Position (1, 3) double {mustBeFinite} = [0 0 0]

        % Orientation of the rest shape when the body is created (3x3)
        Orientation (3, 3) double {mustBeFinite} = eye(3)

        % Density of the material (kg/m^3)
        Density (1, 1) double {mustBePositive} = 1000

        % Young's modulus of the material (Pa)
        YoungsModulus (1, 1) double {mustBePositive} = 1e6

        % Poisson's ratio of the material, 0.5 for an incompressible one
        PoissonsRatio (1, 1) double {mustBeInRange(PoissonsRatio, 0, 0.5)} = 0.45
    end

    properties (Dependent)
        % Mesh of tetrahedra as PDE Toolbox gives it: an object or struct
        % with Nodes (3xN) and Elements (4xK or 10xK), such as the Mesh of
        % fegeometry or femodel, or one of these whose Mesh is used; sets
        % Vertices and Tetrahedra, and reads them back in this form
        Mesh
    end

    % Settings that apply at any time
    properties
        % Radius (m) of the spheres the vertices collide as
        VertexRadius (1, 1) double {mustBeNonnegative} = 0

        % Solver substeps per simulation step; the most Newton iterations
        % per step for quadratic tetrahedra
        Iterations (1, 1) double {mustBeInteger, mustBePositive} = 10

        % Force left unbalanced at which a step of quadratic tetrahedra has
        % converged, relative to the weight of the body; 0 always takes
        % Iterations iterations
        Tolerance (1, 1) double {mustBeNonnegative} = 1e-4

        % Damping of the vertex velocities (1/s): dv/dt = -Damping*v
        Damping (1, 1) double {mustBeNonnegative} = 0

        % Friction coefficient; a contact combines the values of both bodies
        Friction (1, 1) double {mustBeNonnegative} = 0.5

        % Restitution (bounciness) coefficient, 0 = no bounce; linear
        % tetrahedra only, the contacts of quadratic ones are inelastic
        Restitution (1, 1) double {mustBeNonnegative} = 0

        % Colormap (Nx3, RGB in 0-1) that colors the surface by the von
        % Mises stress over ColorRange, e.g. turbo(64); empty for the
        % plain Color
        Colormap (:, 3) double {mustBeInRange(Colormap, 0, 1)} = zeros(0, 3)

        % Stress range (Pa) mapped on the whole colormap
        ColorRange (1, 2) double = [0 1e6]
    end

    properties (Dependent, SetAccess = private)
        % von Mises stress (Pa) at the vertices (Nx1, with the added edge
        % vertices of quadratic tetrahedra), averaged over the tetrahedra
        % around each; zero before the simulation starts
        VonMisesStress (:, 1) double

        % Boundary triangles of the mesh, wound outwards (Mx3); a face of a
        % quadratic tetrahedron is split into four at its edge vertices
        Faces (:, 3) double

        % Mass of the body (kg)
        Mass (1, 1) double

        % Present vertex positions in the global frame (Nx3, with the added
        % edge vertices of quadratic tetrahedra)
        CurrentVertices (:, 3) double
    end

    methods
        function obj = FlexibleBody(ParentAxes, Options)
            arguments
                ParentAxes = gca
                Options.?phx.FlexibleBody
            end

            % Set default values
            obj.SimulationOrder = "none";
            obj.RedrawOrder = "after";
            obj.Color = phx.base.Shape.newColor;

            % Process input arguments
            obj.ParentAxes = ParentAxes;
            phx.internal.applyArguments(Options, obj);
            if isempty(obj.Tetrahedra)
                error("phx:FlexibleBody:noMesh", "A flexible body needs a mesh: give its Vertices and Tetrahedra.");
            end

            obj.hP = patch(obj.Graphics, "Faces", obj.Faces, "Vertices", obj.CurrentVertices, ...
                "FaceColor", obj.Color, "EdgeColor", "none", "FaceLighting", "gouraud");
            obj.recolor;
        end

        function bbox = boundingBox(objs)
        %boundingBox Returns the axis-aligned bounding box of flexible bodies.
        %
        %   bbox = boundingBox(flexibleBodies) returns the axis-aligned
        %   bounding box, in world coordinates, that encloses the present
        %   meshes of all given flexible bodies. The result is a 2x3 matrix:
        %   - first row:  minimum corner [xmin ymin zmin]
        %   - second row: maximum corner [xmax ymax zmax]
        %
        % See also phx.Body.boundingBox, phx.FlexibleBody.CurrentVertices

            arguments
                objs (1, :) phx.FlexibleBody
            end

            bbox = [Inf Inf Inf; -Inf -Inf -Inf];
            for obj = objs
                [vmin, vmax] = bounds(obj.CurrentVertices, 1);
                bbox = [min(bbox(1, :), vmin); max(bbox(2, :), vmax)];
            end
        end

        function set.Vertices(obj, value)
            obj.Vertices = value;
            obj.redrawRestShape;
        end

        function set.Tetrahedra(obj, value)
            if ~ismember(size(value, 2), [4 10])
                error("phx:FlexibleBody:badTetrahedra", "The tetrahedra must be Kx4, or Kx10 quadratic ones.");
            end
            obj.Tetrahedra = value;
            obj.redrawRestShape;
        end

        function set.ElementOrder(obj, value)
            obj.ElementOrder = value;
            obj.redrawRestShape;
        end

        function set.Mesh(obj, mesh)
            has = @(m, name) (isobject(m) && isprop(m, name)) || (isstruct(m) && isfield(m, name));
            if has(mesh, "Mesh") && ~has(mesh, "Nodes")
                mesh = mesh.Mesh;
            end
            if ~has(mesh, "Nodes") || ~has(mesh, "Elements") || isempty(mesh.Elements)
                error("phx:FlexibleBody:badMesh", ...
                    "A mesh needs Nodes (3xN) and Elements (4xK or 10xK), as PDE Toolbox meshes have; generate it first.");
            end
            if ~ismember(size(mesh.Elements, 1), [4 10]) || size(mesh.Nodes, 1) ~= 3
                error("phx:FlexibleBody:badMesh", ...
                    "The mesh must be a solid of tetrahedra: Nodes 3xN and Elements 4xK or 10xK.");
            end
            obj.Vertices = double(mesh.Nodes)';
            obj.Tetrahedra = double(mesh.Elements)';
        end

        function mesh = get.Mesh(obj)
            mesh = struct("Nodes", obj.Vertices', "Elements", obj.Tetrahedra');
        end

        function set.Position(obj, value)
            obj.Position = value;
            obj.redrawRestShape;
        end

        function set.Orientation(obj, value)
            obj.Orientation = value;
            obj.redrawRestShape;
        end

        function set.VertexRadius(obj, value)
            obj.VertexRadius = value;
            obj.engineSet('vertexradius', value);
        end

        function set.Iterations(obj, value)
            obj.Iterations = value;
            obj.engineSet('iterations', value);
        end

        function set.Tolerance(obj, value)
            obj.Tolerance = value;
            if obj.ElementOrder == "quadratic"
                obj.engineSet('tolerance', value);
            end
        end

        function set.Damping(obj, value)
            obj.Damping = value;
            obj.engineSet('damping', value);
        end

        function set.Friction(obj, value)
            obj.Friction = value;
            obj.engineSet('friction', value);
        end

        function set.Restitution(obj, value)
            obj.Restitution = value;
            obj.engineSet('restitution', value);
        end

        function set.Colormap(obj, map)
            obj.Colormap = map;
            obj.recolor;
        end

        function set.ColorRange(obj, value)
            obj.ColorRange = value;
            obj.recolor;
        end

        function value = get.VonMisesStress(obj)
            if ~isempty(obj.ObjectHandle)
                value = phx.engine.io('get', obj.WorldHandle, obj.ObjectHandle, 'stress')';
            else
                value = zeros(size(obj.mesh, 1), 1);
            end
        end

        function value = get.Faces(obj)
            % Of the corners alone, numbered anew for the triangulation,
            % which warns about vertices it does not use
            [V, T] = obj.mesh;
            [corner, ~, k] = unique(obj.positiveTetrahedra(V, T));
            value = corner(freeBoundary(triangulation(reshape(k, [], 4), V(corner, :))));
            if size(T, 2) == 10
                % Each corner triangle split at the vertices of its edges,
                % found by the corners they join
                ends = obj.edgeEnds(T);
                middle = reshape(T(:, 5:10), [], 1);
                [~, i] = ismember(sort(value(:, [1 2]), 2), ends, "rows");
                [~, j] = ismember(sort(value(:, [2 3]), 2), ends, "rows");
                [~, k] = ismember(sort(value(:, [3 1]), 2), ends, "rows");
                six = [value middle(i) middle(j) middle(k)];
                value = [six(:, [1 4 6]); six(:, [4 2 5]); six(:, [6 5 3]); six(:, [4 5 6])];
            end
        end

        function value = get.Mass(obj)
            % Of the straight-edged tetrahedra between the corners
            [V, T] = obj.mesh;
            [~, volume] = obj.positiveTetrahedra(V, T);
            value = obj.Density*sum(volume);
        end

        function value = get.CurrentVertices(obj)
            if ~isempty(obj.ObjectHandle)
                value = phx.engine.io('get', obj.WorldHandle, obj.ObjectHandle, 'vertices')';
            else
                value = obj.mesh*obj.Orientation' + obj.Position;
            end
        end
    end

    methods (Access = ?phx.base.Object)
        function state = stateTransfer(obj, state)
            % The engine keeps no state that could be written back: the body
            % starts again from its rest shape when it is created anew. What
            % is drawn stays as it was.
            arguments
                obj
                state = []
            end

            if isempty(state)
                state.Vertices = obj.CurrentVertices;
            elseif isvalid(obj.hP)
                obj.hP.Vertices = state.Vertices;
            end
        end
    end

    methods (Access = private)
        function [V, T] = mesh(obj)
            % The vertices and tetrahedra handed to the engine: Kx4 linear
            % ones as given, or Kx10 quadratic ones, a Kx4 mesh refined by
            % the middles of its edges
            V = obj.Vertices;
            T = obj.Tetrahedra;
            if obj.ElementOrder == "linear"
                if size(T, 2) == 10
                    error("phx:FlexibleBody:badTetrahedra", ...
                        "Quadratic tetrahedra (Kx10) need the ElementOrder ""quadratic"".");
                end
            elseif size(T, 2) == 4
                [ends, ~, j] = unique(phx.FlexibleBody.edgeEnds(T), "rows");
                T = [T size(V, 1) + reshape(j, [], 6)];
                V = [V; (V(ends(:, 1), :) + V(ends(:, 2), :))/2];
            end
        end

        function recolor(obj)
            % The plain colour, or the stress mapped on the colormap per
            % vertex, interpolated over the faces
            if isempty(obj.hP) || ~isvalid(obj.hP)
                return
            end
            ph = phx.internal.PrimitiveHelper(obj.hP);
            if isempty(obj.Colormap)
                ph.releaseColor;
            else
                ph.holdColor;
                set(obj.hP, "FaceVertexCData", obj.stressColors(obj.VonMisesStress), "FaceColor", "interp");
            end
        end

        function rgb = stressColors(obj, stress)
            map = obj.Colormap;
            r = obj.ColorRange;
            id = round(1 + (stress - r(1))/(r(2) - r(1))*(size(map, 1) - 1));
            id = min(max(id, 1), size(map, 1));
            rgb = map(id, :);
        end

        function engineSet(obj, field, value)
            if ~isempty(obj.ObjectHandle)
                phx.engine.io('set', obj.WorldHandle, obj.ObjectHandle, field, value);
            end
        end

        function redrawRestShape(obj)
            % The mesh drawn before the simulation creates the body; the
            % vertices, tetrahedra and order are set one after the other,
            % so the patch takes them only once they fit together
            if isempty(obj.hP) || ~isempty(obj.ObjectHandle) || isempty(obj.Tetrahedra) ...
                    || any(obj.Tetrahedra > size(obj.Vertices, 1), "all") ...
                    || (obj.ElementOrder == "linear" && size(obj.Tetrahedra, 2) == 10)
                return
            end
            set(obj.hP, "Faces", obj.Faces, "Vertices", obj.CurrentVertices);
        end
    end

    methods (Static, Access = private)
        function ends = edgeEnds(T)
            % The corners of the edges 12 23 13 14 24 34 of all tetrahedra,
            % sorted (6Kx2, edge by edge)
            ends = sort([reshape(T(:, [1 2 1 1 2 3]), [], 1) reshape(T(:, [2 3 3 4 4 4]), [], 1)], 2);
        end

        function [T, volume] = positiveTetrahedra(V, T)
            % Corner tetrahedra turned to a positive volume, which the
            % outward winding of the boundary relies on, and their volumes
            T = T(:, 1:4);
            a = V(T(:, 1), :);
            volume = dot(cross(V(T(:, 2), :) - a, V(T(:, 3), :) - a, 2), V(T(:, 4), :) - a, 2)/6;
            T(volume < 0, [2 3]) = T(volume < 0, [3 2]);
            volume = abs(volume);
        end
    end

    methods (Access = protected)
        function valid = initObject(obj, world)
            valid = true;
            if isempty(obj.ObjectHandle)
                pose = [obj.Orientation obj.Position'; 0 0 0 1];
                [V, T] = obj.mesh;
                obj.ObjectHandle = phx.engine.io('add', world, 'flexiblebody', V', ...
                    obj.Faces', pose, obj.Mass, [obj.YoungsModulus obj.PoissonsRatio], ...
                    T');
                obj.WorldHandle = world;
                obj.VertexRadius = obj.VertexRadius;
                obj.Iterations = obj.Iterations;
                obj.Tolerance = obj.Tolerance;
                obj.Damping = obj.Damping;
                obj.Friction = obj.Friction;
                obj.Restitution = obj.Restitution;
            end
        end

        function destroyObject(obj)
            if ~isempty(obj.ObjectHandle)
                phx.engine.io('remove', obj.WorldHandle, obj.ObjectHandle);
                obj.ObjectHandle = [];
            end
        end
    end

    methods (Static, Access = protected)
        function resolveState(cellObjs, dt, time, world)
        end

        function updateView(cellObjs, dt, time, world)
            for i = 1:numel(cellObjs)
                obj = cellObjs{i};
                obj.hP.Vertices = phx.engine.io('get', world, obj.ObjectHandle, 'vertices')';
                if ~isempty(obj.Colormap)
                    obj.hP.FaceVertexCData = obj.stressColors(phx.engine.io('get', world, obj.ObjectHandle, 'stress')');
                end
            end
        end
    end

end
