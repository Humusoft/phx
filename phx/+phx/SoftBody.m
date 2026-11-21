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
%   simulation creates the soft body; Velocities give its vertices an
%   initial motion. CurrentVertices and CurrentVelocities read the present
%   state: during a run that of the engine, after it the last one.
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

        % The vertices (Nx3, world) the last simulation ended with, until
        % the rest shape changes; empty before
        LastVertices = zeros(0, 3)

        % Their velocities (Nx3, world), likewise
        LastVelocities = zeros(0, 3)

        % Velocities as set: one row per vertex, or one for all
        InitialVelocities = [0 0 0]

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

    % Initial motion, read when the simulation creates the soft body
    properties (Dependent)
        % Velocities of the vertices in the world frame (Nx3, m/s); one row
        % sets all of them alike. Zero by default.
        Velocities (:, 3) double {mustBeFinite}
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

    % Settings that apply at any time
    properties
        % Gas pressure inside a closed mesh, as n*R*T (J): it pushes on the
        % faces with this over the enclosed volume
        Pressure (1, 1) double {mustBeNonnegative} = 0

        % Radius (m) of the spheres the vertices collide as
        VertexRadius (1, 1) double {mustBeNonnegative} = 0

        % Solver substeps of the soft body per simulation step
        Iterations (1, 1) double {mustBeInteger, mustBePositive} = 5

        % Damping of the vertex velocities (1/s): dv/dt = -LinearDamping*v
        LinearDamping (1, 1) double {mustBeNonnegative} = 0

        % Friction coefficient; a contact combines the values of both bodies
        % as their geometric mean, sqrt(mu1*mu2)
        Friction (1, 1) double {mustBeNonnegative} = 0.5

        % Restitution (bounciness) coefficient, 0 = no bounce; a contact
        % bounces with the smaller of the two bodies' values
        Restitution (1, 1) double {mustBeNonnegative} = 0
    end

    % Former name of LinearDamping (PHX 2.0)
    properties (Dependent, Hidden)
        Damping
    end

    properties (Dependent, SetAccess = private)
        % Present vertex positions in the world frame (Nx3): during a run
        % those of the engine, after it the last ones (until the rest shape
        % changes), before the first the rest shape at Position and
        % Orientation
        CurrentVertices (:, 3) double

        % Present vertex velocities in the world frame (Nx3, m/s): during a
        % run those of the engine, after it the last ones (until the rest
        % shape changes), before the first the Velocities
        CurrentVelocities (:, 3) double
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
            if ~isnan(shape.Density)
                warning("phx:Shape:densityMoved", "The density of a shape is deprecated, and a soft " + ...
                    "body has its Mass: the density of its shape is ignored.");
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

        function set.LinearDamping(obj, value)
            obj.LinearDamping = value;
            obj.engineSet('damping', value);
        end

        function value = get.Damping(obj)
            warnDamping;
            value = obj.LinearDamping;
        end

        function set.Damping(obj, value)
            warnDamping;
            obj.LinearDamping = value;
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
            elseif ~isempty(obj.LastVertices)
                value = obj.LastVertices;
            else
                value = obj.Vertices*obj.Orientation' + obj.Position;
            end
        end

        function value = get.Velocities(obj)
            value = obj.InitialVelocities;
            if size(value, 1) == 1
                value = repmat(value, size(obj.Vertices, 1), 1);
            end
        end

        function set.Velocities(obj, value)
            if size(value, 1) ~= 1 && ~isempty(obj.Vertices)
                obj.checkVelocities(value);
            end
            obj.InitialVelocities = value;
        end

        function value = get.CurrentVelocities(obj)
            if ~isempty(obj.ObjectHandle)
                value = phxEngine('get', obj.WorldHandle, obj.ObjectHandle, 'velocities')';
            elseif ~isempty(obj.LastVelocities)
                value = obj.LastVelocities;
            else
                value = obj.checkVelocities(obj.Velocities);
            end
        end
    end

    methods (Access = ?phx.base.Object)
        function state = stateTransfer(obj, state)
            % The engine keeps no state that could be written back: the
            % soft body starts again from its rest shape when it is created
            % anew. The last vertices stay drawn and with their velocities
            % in CurrentVertices and CurrentVelocities, until the rest shape
            % changes.
            arguments
                obj
                state = []
            end

            if isempty(state)
                state.Vertices = obj.CurrentVertices;
                state.Velocities = obj.CurrentVelocities;
            else
                obj.LastVertices = state.Vertices;
                obj.LastVelocities = state.Velocities;
                if isvalid(obj.hP)
                    obj.showVertices(state.Vertices');
                end
            end
        end
    end

    methods (Access = private)
        function engineSet(obj, field, value)
            if ~isempty(obj.ObjectHandle)
                phxEngine('set', obj.WorldHandle, obj.ObjectHandle, field, value);
            end
        end

        function value = checkVelocities(obj, value)
            % Velocities of another number of rows than the vertices fail
            if size(value, 1) ~= size(obj.Vertices, 1)
                error("phx:SoftBody:sizeMismatch", "Give the soft body one velocity per vertex " + ...
                    "(%d), or one for all.", size(obj.Vertices, 1));
            end
        end

        function redrawRestShape(obj)
            % The mesh drawn before the simulation creates the soft body;
            % Vertices and Faces are set one after the other, so the patch
            % takes them only once they fit together
            obj.LastVertices = zeros(0, 3);   % a new rest shape: the last run is over
            obj.LastVelocities = zeros(0, 3);
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
                obj.LinearDamping = obj.LinearDamping;
                obj.Friction = obj.Friction;
                obj.Restitution = obj.Restitution;
                if any(obj.InitialVelocities, "all")
                    phxEngine('set', world, obj.ObjectHandle, 'velocities', obj.checkVelocities(obj.Velocities)');
                end
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

function warnDamping
%warnDamping Warns that the former name Damping of LinearDamping is used.
    warning("phx:SoftBody:renamedProperty", "SoftBody.Damping was renamed to LinearDamping, " + ...
        "the name the particle groups use; the name Damping will be removed in a future release.");
end
