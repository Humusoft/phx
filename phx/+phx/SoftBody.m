classdef SoftBody < phx.base.Object
%phx.SoftBody Deformable body made of a triangle mesh
%
%   SoftBody is a mesh whose vertices are particles of equal mass held
%   together by its edges: a cloth if the mesh is open, a cushion or a ball
%   if it is closed, optionally inflated by a gas pressure. It collides with
%   rigid bodies (phx.Body), not with other soft bodies. No joint can hold
%   it directly: a body welded into it with phx.IntersectionWeld holds and
%   loads it instead.
%
%   phx.SoftBody(axes, Vertices=V, Faces=F) creates a soft body of the
%   triangle mesh V (Nx3, in the body frame) and F (Mx3 vertex indices, as
%   for patch) and draws it into the given axes object; [] draws nothing.
%   The mesh is the rest shape, placed at Position and Orientation when the
%   simulation creates the soft body.
%
%   phx.SoftBody(axes, Shape=shape) covers a shape with the surface mesh it
%   generates, its vertices spaced about evenly as the ElementResolution of
%   the shape sets; the shape is a phx.shape.* object or a cell array of a
%   shape name and its arguments, as phx.Body takes it. Only the geometry
%   is taken: the Mass stays that of the soft body.
%
%       cushion = phx.SoftBody(gca, Shape={"Box", "Size", [0.4 0.4 0.1]}, Pressure=100);
%
%   phx.SoftBody(___, Name, Value, ...) sets properties values according to
%   given name-value pairs.
%
%   See also phx.Body, phx.IntersectionWeld, phx.Simulation, phx.shape.Box

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

%#ok<*MCSUP> OK to access other properties in setters
%#ok<*INUSD> OK to see the full list of arguments for callbacks

    properties (Access = private)
        hP

        % Drawn as a TriangleStrip (in the Viewer), with the map that sums
        % the face normals into vertex normals (faces x vertices); else as
        % a patch
        Strip (1, 1) logical = false
        NormalMap
    end

    properties (Dependent, GetAccess = private)
        % Shape covered with a surface mesh, which sets Vertices and Faces: a
        % phx.shape.* object that can generate the mesh, or a cell array of
        % its name and constructor arguments
        Shape
    end

    % Rest shape, read when the simulation creates the soft body
    properties
        % Vertices of the mesh in the body frame (Nx3)
        Vertices (:, 3) double {mustBeFinite} = zeros(0, 3)

        % Triangles of the mesh as vertex indices (Mx3); a closed mesh holding
        % a Pressure lists them counter-clockwise seen from outside
        Faces (:, 3) double {mustBeInteger, mustBePositive} = zeros(0, 3)

        % Position of the rest shape when the soft body is created
        Position (1, 3) double {mustBeFinite} = [0 0 0]

        % Orientation of the rest shape when the soft body is created (3x3)
        Orientation (3, 3) double {mustBeFinite} = eye(3)

        % Total mass (kg), spread evenly over the vertices
        Mass (1, 1) double {mustBePositive} = 1

        % Compliance of the edges (m/N), the inverse of their stiffness;
        % 0 keeps them at their rest length
        Compliance (1, 1) double {mustBeNonnegative} = 0

        % Compliance of the bend edges across each pair of adjacent triangles
        % (m/N); Inf leaves them out
        BendCompliance (1, 1) double {mustBeNonnegative} = Inf
    end

    % Settings that apply at any time
    properties
        % Gas pressure inside a closed mesh, as n*R*T (J): it pushes on the
        % faces with this over the enclosed volume
        Pressure (1, 1) double {mustBeNonnegative} = 0

        % Radius (m) of the spheres the vertices collide as
        VertexRadius (1, 1) double {mustBeNonnegative} = 0

        % Solver substeps of the soft body per simulation step
        Iterations (1, 1) double {mustBeInteger, mustBePositive} = 5

        % Damping of the vertex velocities (1/s): dv/dt = -Damping*v
        Damping (1, 1) double {mustBeNonnegative} = 0

        % Friction coefficient; a contact combines the values of both bodies
        % as their product
        Friction (1, 1) double {mustBeNonnegative} = 0.5

        % Restitution (bounciness) coefficient, 0 = no bounce
        Restitution (1, 1) double {mustBeNonnegative} = 0
    end

    properties (Dependent, SetAccess = private)
        % Present vertex positions in the global frame (Nx3)
        CurrentVertices (:, 3) double
    end

    methods
        function obj = SoftBody(ParentAxes, Options)
            arguments
                ParentAxes = gca
                Options.?phx.SoftBody
            end

            % Set default values
            obj.SimulationOrder = "none";
            obj.RedrawOrder = "after";
            obj.Color = phx.base.Shape.newColor;

            % Process input arguments
            obj.ParentAxes = ParentAxes;
            if isfield(Options, "Shape") && any(isfield(Options, ["Vertices", "Faces"]))
                error("phx:SoftBody:shapeAndMesh", "Give a soft body either a Shape or a mesh (Vertices and Faces), not both.");
            end
            phx.internal.applyArguments(Options, obj);
            if isempty(obj.Faces)
                error("phx:SoftBody:noMesh", "A soft body needs a mesh: give its Vertices and Faces, or a Shape.");
            end

            % In the Viewer a TriangleStrip, which redraws faster than a
            % patch; its material is the "plastic" of the shapes there, and
            % both sides of an open mesh are lit
            parent = obj.Graphics.Parent;
            obj.Strip = ~isempty(parent) && isappdata(parent, "phxAxes");
            if obj.Strip
                obj.hP = matlab.graphics.primitive.world.TriangleStrip("Parent", obj.Graphics, ...
                    "ColorData", uint8([obj.Color 1]*255)', "ColorBinding", "object", ...
                    "NormalBinding", "interpolated", "TwoSidedLighting", "on", "HitTest", "on", ...
                    "AmbientStrength", 0.32, "DiffuseStrength", 0.68, "SpecularStrength", 0.4, ...
                    "SpecularExponent", 10, "SpecularColorReflectance", 1);
            else
                obj.hP = patch(obj.Graphics, "FaceColor", obj.Color, "EdgeColor", "none", "FaceLighting", "gouraud");
            end
            obj.redrawRestShape;
        end

        function bbox = boundingBox(objs)
        %boundingBox Returns the axis-aligned bounding box of soft bodies.
        %
        %   bbox = boundingBox(softBodies) returns the axis-aligned bounding
        %   box, in world coordinates, that encloses the present meshes of
        %   all given soft bodies. The result is a 2x3 matrix:
        %   - first row:  minimum corner [xmin ymin zmin]
        %   - second row: maximum corner [xmax ymax zmax]
        %
        % See also phx.Body.boundingBox, phx.SoftBody.CurrentVertices

            arguments
                objs (1, :) phx.SoftBody
            end

            bbox = [Inf Inf Inf; -Inf -Inf -Inf];
            for obj = objs
                [vmin, vmax] = bounds(obj.CurrentVertices, 1);
                bbox = [min(bbox(1, :), vmin); max(bbox(2, :), vmax)];
            end
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
                error("phx:SoftBody:badShape", ...
                    "The Shape of a soft body must be a phx.shape.* object or a cell array of its name and arguments.");
            end

            % The faces are cleared first, so that the drawing waits until
            % the new vertices and faces fit together
            [V, F] = shape.generateSurfaceMesh;
            obj.Faces = zeros(0, 3);
            obj.Vertices = V;
            obj.Faces = F;
        end

        function set.Vertices(obj, value)
            obj.Vertices = value;
            obj.redrawRestShape;
        end

        function set.Faces(obj, value)
            obj.Faces = value;
            obj.redrawRestShape;
        end

        function set.Position(obj, value)
            obj.Position = value;
            obj.redrawRestShape;
        end

        function set.Orientation(obj, value)
            obj.Orientation = value;
            obj.redrawRestShape;
        end

        function set.Pressure(obj, value)
            obj.Pressure = value;
            obj.engineSet('pressure', value);
        end

        function set.VertexRadius(obj, value)
            obj.VertexRadius = value;
            obj.engineSet('vertexradius', value);
        end

        function set.Iterations(obj, value)
            obj.Iterations = value;
            obj.engineSet('iterations', value);
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

        function value = get.CurrentVertices(obj)
            if ~isempty(obj.ObjectHandle)
                value = phxEngine('get', obj.WorldHandle, obj.ObjectHandle, 'vertices')';
            else
                value = obj.Vertices*obj.Orientation' + obj.Position;
            end
        end
    end

    methods (Access = ?phx.base.Object)
        function state = stateTransfer(obj, state)
            % The engine keeps no state that could be written back: the
            % soft body starts again from its rest shape when it is created
            % anew. What is drawn stays as it was.
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
        function engineSet(obj, field, value)
            if ~isempty(obj.ObjectHandle)
                phxEngine('set', obj.WorldHandle, obj.ObjectHandle, field, value);
            end
        end

        function redrawRestShape(obj)
            % The mesh drawn before the simulation creates the soft body;
            % Vertices and Faces are set one after the other, so the patch
            % takes them only once they fit together
            if isempty(obj.hP) || ~isempty(obj.ObjectHandle) || any(obj.Faces > size(obj.Vertices, 1), "all")
                return
            end
            if obj.Strip
                F = obj.Faces;
                m = size(F, 1);
                obj.NormalMap = sparse(repmat((1:m)', 3, 1), F(:), 1, m, size(obj.Vertices, 1));
                obj.hP.VertexIndices = uint32(reshape(F', 1, []));
                obj.showVertices(obj.CurrentVertices');
            else
                set(obj.hP, "Faces", obj.Faces, "Vertices", obj.CurrentVertices);
            end
        end

        function showVertices(obj, X)
            % The mesh at the vertices X (3xN): a patch takes them as they
            % are, the strip with the vertex normals, sums of the normals of
            % the faces around (their length twice the face area, which
            % weights them)
            if ~obj.Strip
                obj.hP.Vertices = X';
                return
            end
            F = obj.Faces;
            a = X(:, F(:, 1));
            e1 = X(:, F(:, 2)) - a;
            e2 = X(:, F(:, 3)) - a;
            n = [e1(2, :).*e2(3, :) - e1(3, :).*e2(2, :); e1(3, :).*e2(1, :) - e1(1, :).*e2(3, :); ...
                e1(1, :).*e2(2, :) - e1(2, :).*e2(1, :)]*obj.NormalMap;
            n = n./max(sqrt(sum(n.^2, 1)), realmin);
            obj.hP.VertexData = single(X);
            obj.hP.NormalData = single(n);
        end
    end

    methods (Access = protected)
        function valid = initObject(obj, world)
            if isempty(obj.ObjectHandle)
                obj.WorldHandle = world;
                T = [obj.Orientation obj.Position'; 0 0 0 1];
                obj.ObjectHandle = phxEngine('add', world, 'softbody', obj.Vertices', obj.Faces', ...
                    T, obj.Mass, [obj.Compliance obj.BendCompliance]);
                obj.Pressure = obj.Pressure;
                obj.VertexRadius = obj.VertexRadius;
                obj.Iterations = obj.Iterations;
                obj.Damping = obj.Damping;
                obj.Friction = obj.Friction;
                obj.Restitution = obj.Restitution;
            end
            valid = true;
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
            end
        end
    end

end
