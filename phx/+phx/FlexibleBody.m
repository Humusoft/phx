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
%   The tetrahedra are quadratic: each has a vertex in the middle of each
%   edge, so that it bends without the stiffening of linear tetrahedra on a
%   coarse mesh. A step is solved implicitly by Newton iterations until the
%   force left unbalanced falls below Tolerance, so the stiffness does not
%   depend on Iterations, which only limits the iterations per step; two are
%   usually enough, more only in a step with a sudden load such as an
%   impact.
%
%   phx.FlexibleBody(axes, Vertices=V, Tetrahedra=T) creates a flexible body
%   of the tetrahedral mesh V (Nx3, in the body frame) and T (vertex
%   indices) and draws its surface into the given axes object; [] draws
%   nothing. T is either Kx4, as delaunayTriangulation gives it, and the
%   vertices in the middles of the edges are added to V, or Kx10, a
%   quadratic mesh (the corners, then the vertices of the edges 12 23 13 14
%   24 34), whose edges are curved when the vertices are off their middles.
%   The added vertices follow V in CurrentVertices and VonMisesStress. The
%   mesh is the rest shape, placed at Position and Orientation when the
%   simulation creates the body; its volume, curved edges included, gives
%   the Mass.
%
%   phx.FlexibleBody(axes, Mesh=mesh) takes the mesh of tetrahedra that PDE
%   Toolbox generates, for example of an STL file:
%
%       gm = generateMesh(fegeometry("part.stl"), Hmax=0.01);
%       flex = phx.FlexibleBody(gca, Mesh=gm.Mesh);
%
%   phx.FlexibleBody(axes, Shape=shape) fills a shape with the mesh of
%   tetrahedra it generates and takes its Density; the shape is a
%   phx.shape.* object or a cell array of a shape name and its arguments,
%   as phx.Body takes it. The ElementResolution of the shape sets how fine
%   the mesh is:
%
%       flex = phx.FlexibleBody(gca, Shape={"Box", "Size", [1 0.1 0.1]});
%
%   phx.FlexibleBody(___, Name, Value, ...) sets properties values according
%   to given name-value pairs.
%
%   See also phx.SoftBody, phx.IntersectionWeld, phx.Body, phx.shape.Box

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

%#ok<*MCSUP> OK to access other properties in setters
%#ok<*INUSD> OK to see the full list of arguments for callbacks

    properties (Access = private)
        hP

        % Drawn as a TriangleStrip of the surface (in the Viewer), with its
        % vertices (indices into the mesh), its triangles (Mx3, numbered as
        % its vertices) and the map that sums the face normals into vertex
        % normals (faces x vertices); else as a patch
        Strip (1, 1) logical = false
        Surface (:, 1) double
        SurfaceFaces (:, 3) double
        NormalMap
    end

    % Declared before the mesh and the material, so that given to the
    % constructor, a Density or Mass overrides the one of the shape
    properties (Dependent, GetAccess = private)
        % Shape filled with a mesh of tetrahedra, which sets Vertices,
        % Tetrahedra and Density: a phx.shape.* object that can generate
        % the mesh, or a cell array of its name and constructor arguments
        Shape
    end

    % Rest shape and material, read when the simulation creates the body
    properties
        % Vertices of the mesh in the body frame (Nx3)
        Vertices (:, 3) double {mustBeFinite} = zeros(0, 3)

        % Tetrahedra as vertex indices (Kx4, or Kx10 quadratic ones)
        Tetrahedra (:, :) double {mustBeInteger, mustBePositive} = zeros(0, 4)

        % Position of the rest shape when the body is created
        Position (1, 3) double {mustBeFinite} = [0 0 0]

        % Orientation of the rest shape when the body is created (3x3)
        Orientation (3, 3) double {mustBeFinite} = eye(3)

        % Density of the material (kg/m^3)
        Density (1, 1) double {mustBePositive} = 1000

        % Young's modulus of the material (Pa)
        YoungsModulus (1, 1) double {mustBePositive} = 1e6

        % Poisson's ratio of the material, 0.5 for an incompressible one
        % (taken as at most 0.49995)
        PoissonsRatio (1, 1) double {mustBeInRange(PoissonsRatio, 0, 0.5)} = 0.45
    end

    % The orientation of the rest shape in other forms, views of Orientation
    properties (Dependent)
        % Axis-angle rotation [x y z angle] of the rest shape
        AxisAngle (1, 4) double {mustBeFinite}

        % Euler angles [x y z] of the rest shape for z->y->x order
        EulerAngles (1, 3) double {mustBeFinite}

        % Quaternion [w x y z] of the rest shape
        Quaternion (1, 4) double {mustBeFinite}
    end

    properties (Dependent)
        % Mesh of tetrahedra as PDE Toolbox gives it: an object or struct
        % with Nodes (3xN) and Elements (4xK or 10xK), such as the Mesh of
        % fegeometry or femodel, or one of these whose Mesh is used; sets
        % Vertices and Tetrahedra, and reads them back in this form
        Mesh

        % Mass of the body (kg), the Density times the volume of the mesh;
        % setting it sets the Density for the present mesh, which a later
        % change of the mesh keeps. Given to the constructor, it is taken
        % after the mesh and overrides a Density given there.
        Mass (1, 1) double {mustBePositive, mustBeFinite}
    end

    % Settings that apply at any time
    properties
        % Radius (m) of the spheres the vertices collide as
        VertexRadius (1, 1) double {mustBeNonnegative} = 0

        % Most Newton iterations per step, up to 4 more in a step whose
        % contacts keep changing
        Iterations (1, 1) double {mustBeInteger, mustBePositive} = 10

        % Force left unbalanced at which a step has converged, relative to
        % the weight of the body; 0 takes Iterations iterations unless the
        % force falls below 1e-10 of the weight
        Tolerance (1, 1) double {mustBeNonnegative} = 1e-4

        % Damping of the vertex velocities (1/s): dv/dt = -Damping*v
        Damping (1, 1) double {mustBeNonnegative} = 0

        % Friction coefficient; a contact combines the values of both bodies
        % as their product
        Friction (1, 1) double {mustBeNonnegative} = 0.5

        % Colormap (Nx3, RGB in 0-1) that colors the surface by the von
        % Mises stress over ColorRange, e.g. turbo(64); empty for the
        % plain Color
        Colormap (:, 3) double {mustBeInRange(Colormap, 0, 1)} = zeros(0, 3)

        % Stress range (Pa) mapped on the whole colormap
        ColorRange (1, 2) double = [0 1e6]
    end

    properties (Dependent, SetAccess = private)
        % von Mises stress (Pa) at the vertices (Nx1, with the vertices
        % added in the middles of the edges of a Kx4 mesh), averaged over
        % the tetrahedra around each; zero before the simulation starts
        VonMisesStress (:, 1) double

        % Boundary triangles of the mesh, wound outwards (Mx3); a face of a
        % tetrahedron is split into four at the vertices of its edges
        Faces (:, 3) double

        % Present vertex positions in the global frame (Nx3, with the
        % vertices added in the middles of the edges of a Kx4 mesh)
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
            if isfield(Options, "Shape") && any(isfield(Options, ["Vertices", "Tetrahedra", "Mesh"]))
                error("phx:FlexibleBody:shapeAndMesh", "Give a flexible body either a Shape or a mesh (Vertices and Tetrahedra, or Mesh), not both.");
            end
            phx.internal.applyArguments(Options, obj);
            if isempty(obj.Tetrahedra)
                error("phx:FlexibleBody:noMesh", "A flexible body needs a mesh: give its Vertices and Tetrahedra, a Mesh or a Shape.");
            end

            % In the Viewer a TriangleStrip, which redraws faster than a
            % patch; its material is the "plastic" of the shapes there
            parent = obj.Graphics.Parent;
            obj.Strip = ~isempty(parent) && isappdata(parent, "phxAxes");
            if obj.Strip
                obj.hP = matlab.graphics.primitive.world.TriangleStrip("Parent", obj.Graphics, ...
                    "ColorData", uint8([obj.Color 1]*255)', "ColorBinding", "object", ...
                    "NormalBinding", "interpolated", "HitTest", "on", ...
                    "AmbientStrength", 0.32, "DiffuseStrength", 0.68, "SpecularStrength", 0.4, ...
                    "SpecularExponent", 10, "SpecularColorReflectance", 1);
            else
                obj.hP = patch(obj.Graphics, "FaceColor", obj.Color, "EdgeColor", "none", "FaceLighting", "gouraud");
            end
            obj.redrawRestShape;
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

        function set.Shape(obj, shape)
            if iscell(shape)
                if endsWith(lower(shape{1}), [".stl", ".obj", ".ply"])
                    shape = phx.shape.Mesh("Source", shape{:});
                else
                    shape = feval("phx.shape." + shape{1}, shape{2:end});
                end
            end
            if ~isa(shape, "phx.base.Shape")
                error("phx:FlexibleBody:badShape", ...
                    "The Shape of a flexible body must be a phx.shape.* object or a cell array of its name and arguments.");
            end

            % The tetrahedra are cleared first, so that the drawing waits
            % until the new vertices and tetrahedra fit together
            [V, T] = shape.generateVolumetricMesh;
            obj.Tetrahedra = zeros(0, size(T, 2));
            obj.Vertices = V;
            obj.Tetrahedra = T;
            obj.Density = shape.Density;
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

        function value = get.AxisAngle(obj)
            value = phx.internal.Math.decompAA(obj.Orientation);
        end

        function set.AxisAngle(obj, value)
            obj.Orientation = phx.internal.Math.rotAA(value(1:3), value(4));
        end

        function value = get.EulerAngles(obj)
            value = phx.internal.Math.decomp321(obj.Orientation);
        end

        function set.EulerAngles(obj, value)
            obj.Orientation = phx.internal.Math.rot321(value);
        end

        function value = get.Quaternion(obj)
            value = phx.internal.Math.decompQ(obj.Orientation);
        end

        function set.Quaternion(obj, value)
            obj.Orientation = phx.internal.Math.rotQ(value);
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
            obj.engineSet('tolerance', value);
        end

        function set.Damping(obj, value)
            obj.Damping = value;
            obj.engineSet('damping', value);
        end

        function set.Friction(obj, value)
            obj.Friction = value;
            obj.engineSet('friction', value);
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
                value = phxEngine('get', obj.WorldHandle, obj.ObjectHandle, 'stress')';
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
            % Each corner triangle split at the vertices of its edges, found
            % by the corners they join
            ends = obj.edgeEnds(T);
            middle = reshape(T(:, 5:10), [], 1);
            [~, i] = ismember(sort(value(:, [1 2]), 2), ends, "rows");
            [~, j] = ismember(sort(value(:, [2 3]), 2), ends, "rows");
            [~, k] = ismember(sort(value(:, [3 1]), 2), ends, "rows");
            six = [value middle(i) middle(j) middle(k)];
            value = [six(:, [1 4 6]); six(:, [4 2 5]); six(:, [6 5 3]); six(:, [4 5 6])];
        end

        function value = get.Mass(obj)
            value = obj.Density*obj.volume;
        end

        function set.Mass(obj, value)
            V = obj.volume;
            if V == 0
                error("phx:FlexibleBody:noMesh", "Mass needs the mesh: give its Vertices and Tetrahedra first.");
            end
            obj.Density = value/V;
        end

        function value = get.CurrentVertices(obj)
            if ~isempty(obj.ObjectHandle)
                value = phxEngine('get', obj.WorldHandle, obj.ObjectHandle, 'vertices')';
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
                obj.showVertices(state.Vertices');
            end
        end
    end

    methods (Access = private)
        function value = volume(obj)
            % Of the tetrahedra handed to the engine, curved ones included
            [V, T] = obj.mesh;
            value = sum(phx.internal.Geometry.tetrahedraVolumes(V, T));
        end

        function [V, T] = mesh(obj)
            % The vertices and tetrahedra handed to the engine: a Kx10 mesh
            % as given, or a Kx4 one refined by the middles of its edges
            V = obj.Vertices;
            T = obj.Tetrahedra;
            if size(T, 2) == 4
                [V, T] = phx.internal.Geometry.quadraticTetrahedra(V, T);
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
                if obj.Strip
                    obj.hP.ColorBinding = "object";
                end
                ph.releaseColor;
            else
                ph.holdColor;
                if obj.Strip
                    obj.showStress(obj.VonMisesStress');
                else
                    set(obj.hP, "FaceVertexCData", obj.stressColors(obj.VonMisesStress), "FaceColor", "interp");
                end
            end
        end

        function rgb = stressColors(obj, stress)
            % Colors of the stresses (Nx1) on the colormap, Nx3 for a patch
            map = obj.Colormap;
            r = obj.ColorRange;
            id = round(1 + (stress - r(1))/(r(2) - r(1))*(size(map, 1) - 1));
            id = min(max(id, 1), size(map, 1));
            rgb = map(id, :);
        end

        function showVertices(obj, X)
            % The mesh at the vertices X (3xN, all of them): a patch takes
            % them as they are, the strip those of its surface with the
            % vertex normals, sums of the normals of the faces around (their
            % length twice the face area, which weights them)
            if ~obj.Strip
                obj.hP.Vertices = X';
                return
            end
            X = X(:, obj.Surface);
            F = obj.SurfaceFaces;
            a = X(:, F(:, 1));
            e1 = X(:, F(:, 2)) - a;
            e2 = X(:, F(:, 3)) - a;
            n = [e1(2, :).*e2(3, :) - e1(3, :).*e2(2, :); e1(3, :).*e2(1, :) - e1(1, :).*e2(3, :); ...
                e1(1, :).*e2(2, :) - e1(2, :).*e2(1, :)]*obj.NormalMap;
            n = n./max(sqrt(sum(n.^2, 1)), realmin);
            obj.hP.VertexData = single(X);
            obj.hP.NormalData = single(n);
        end

        function showStress(obj, stress)
            % The stress (1xN, at all vertices) on the colormap, per vertex
            % of the strip
            map = obj.Colormap;
            r = obj.ColorRange;
            id = round(1 + (stress(obj.Surface) - r(1))/(r(2) - r(1))*(size(map, 1) - 1));
            id = min(max(id, 1), size(map, 1));
            rgba = [uint8(255*map'); repmat(uint8(255), 1, size(map, 1))];
            set(obj.hP, "ColorData", rgba(:, id), "ColorBinding", "interpolated");
        end

        function engineSet(obj, field, value)
            if ~isempty(obj.ObjectHandle)
                phxEngine('set', obj.WorldHandle, obj.ObjectHandle, field, value);
            end
        end

        function redrawRestShape(obj)
            % The mesh drawn before the simulation creates the body; the
            % vertices and tetrahedra are set one after the other, so the
            % patch takes them only once they fit together
            if isempty(obj.hP) || ~isempty(obj.ObjectHandle) || isempty(obj.Tetrahedra) ...
                    || any(obj.Tetrahedra > size(obj.Vertices, 1), "all")
                return
            end
            F = obj.Faces;
            if obj.Strip
                % Its own vertices numbered anew, and the normal map
                [obj.Surface, ~, k] = unique(F);
                obj.SurfaceFaces = reshape(k, [], 3);
                m = size(F, 1);
                obj.NormalMap = sparse(repmat((1:m)', 3, 1), k, 1, m, numel(obj.Surface));
                obj.hP.VertexIndices = uint32(reshape(obj.SurfaceFaces', 1, []));
                obj.showVertices(obj.CurrentVertices');
            else
                set(obj.hP, "Faces", F, "Vertices", obj.CurrentVertices);
            end
            if ~isempty(obj.Colormap)
                obj.recolor;
            end
        end
    end

    methods (Static, Access = private)
        function ends = edgeEnds(T)
            % The corners of the edges 12 23 13 14 24 34 of all tetrahedra,
            % sorted (6Kx2, edge by edge)
            ends = sort([reshape(T(:, [1 2 1 1 2 3]), [], 1) reshape(T(:, [2 3 3 4 4 4]), [], 1)], 2);
        end

        function T = positiveTetrahedra(V, T)
            % Corner tetrahedra turned to a positive volume, which the
            % outward winding of the boundary relies on
            T = T(:, 1:4);
            a = V(T(:, 1), :);
            negative = dot(cross(V(T(:, 2), :) - a, V(T(:, 3), :) - a, 2), V(T(:, 4), :) - a, 2) < 0;
            T(negative, [2 3]) = T(negative, [3 2]);
        end
    end

    methods (Access = protected)
        function valid = initObject(obj, world)
            valid = true;
            if isempty(obj.ObjectHandle)
                pose = [obj.Orientation obj.Position'; 0 0 0 1];
                [V, T] = obj.mesh;
                obj.ObjectHandle = phxEngine('add', world, 'flexiblebody', V', ...
                    obj.Faces', pose, obj.Mass, [obj.YoungsModulus obj.PoissonsRatio], ...
                    T');
                obj.WorldHandle = world;
                obj.VertexRadius = obj.VertexRadius;
                obj.Iterations = obj.Iterations;
                obj.Tolerance = obj.Tolerance;
                obj.Damping = obj.Damping;
                obj.Friction = obj.Friction;
            end
        end

        function destroyObject(obj)
            if ~isempty(obj.ObjectHandle)
                phxEngine('remove', obj.WorldHandle, obj.ObjectHandle);
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
                obj.showVertices(phxEngine('get', world, obj.ObjectHandle, 'vertices'));
                if ~isempty(obj.Colormap)
                    stress = phxEngine('get', world, obj.ObjectHandle, 'stress');
                    if obj.Strip
                        obj.showStress(stress);
                    else
                        obj.hP.FaceVertexCData = obj.stressColors(stress');
                    end
                end
            end
        end
    end

end
