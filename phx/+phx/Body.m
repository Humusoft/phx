classdef Body < phx.base.Object
%phx.Body Physical body
%
%   Body is the principal object of PHX. It represents all
%   movable objects in the virtual world.
%
%   phx.Body(axes) creates a physical object and draws its visual representation
%   into given axes object. If the visualization is not needed, it is possible
%   to pass an empty value []. Target axes object can be also assigned later
%   using the ParentAxes property.
%
%   phx.Body(___, Name, Value, ...) creates a body and sets properties values
%   according to given name-value pairs.
%
%   See also phx.Simulation, phx.Spring, phx.Trace

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^
    
%#ok<*MCSUP> OK to access other properties in setters
%#ok<*INUSD> OK to see the full list of arguments for callbacks

% TODO dependent props for accelerations (computed as totalforce/mass)

    % Internal
    properties (Access = ?phx.base.Object)
        Matrix = eye(4)
    end

    properties (SetAccess = private)
        ShadowGraphics
        States = struct
    end

    properties (SetAccess = protected, Transient)
        WorldHandle = []
    end

    % Kinematic state in a global coordinate system
    properties
        % Linear acceleration
        %LinearAcceleration = [0 0 0]

        % Angular acceleration
        %AngularAcceleration = [0 0 0]

        % Linear velocity
        LinearVelocity = [0 0 0]

        % Angular velocity
        AngularVelocity = [0 0 0]
    end

    % Kinematic state in a global coordinate system
    properties (Dependent)
        % Full transform matrix (4x4)
        Transform (4, 4) double

        % Linear position
        Position (1, 3) double

        % Rotation matrix (3x3)
        Orientation (3, 3) double

        % Axis-angle rotation
        AxisAngle (1, 4) double

        % Euler angles rotation for z->y->x order
        EulerAngles (1, 3) double

        % Quaternion rotation
        Quaternion (1, 4) double

        % Total force in the centre of the body
        TotalForce (1, 3) double

        % Total torque in the centre of the body
        TotalTorque (1, 3) double

        % Kinetic energy of the body (J)
        Energy (1, 1) double
    end

    properties (Dependent, GetAccess = private)
        % Body shape can be defined as
        % - phx.shape.* object
        % - cell array of phx.shape.* function name and constructor arguments 
        Shape
    end

    % General properties
    properties
        % Body type
        % - static
        % - kinematic
        % - dynamic 
        Type {mustBeMember(Type, ["static", "kinematic", "dynamic"])} = "dynamic";

        % Body mass (kg)
        Mass (1, 1) double = 1

        % Body inertia tensor (kg*m^2)
        Inertia (1, 3) double = [1 1 1]

        % Friction coefficients [drag roll spin]
        Friction (1, 3) double {mustBeGreaterThanOrEqual(Friction, 0)} = [0.5 0 0]

        % Restitution (bounciness) coefficient, 0 = no bounce; the resulting
        % bounce of a contact combines the values of both colliding bodies
        Restitution (1, 1) double {mustBeGreaterThanOrEqual(Restitution, 0)} = 0

        % Enable collision response for this body
        Collisions (1, 1) logical = true

        % Collision group
        % CollisionGroup = 0 % TODO implement collision groups

        % Custom callback for double-click action on any part of this body
        % (works only in the phx.extra.Viewer)
        OnDoubleClickFcn = []
    end

    properties (Dependent, Hidden)
        % Numerical representation of Type property
        % (0 = static, 1 = kinematic, 2 = dynamic)
        TypeID
    end

    methods
        function obj = Body(ParentAxes, Options)
            arguments
                ParentAxes = gca
                Options.?phx.Body
            end

            % Set default values
            obj.SimulationOrder = "none";
            obj.RedrawOrder = "after";

            % Process input arguments
            obj.ParentAxes = ParentAxes;
            phx.internal.applyArguments(Options, obj);

            % Draw default shape
            if isempty(obj.Graphics.Children)
                obj.Shape = phx.shape.Box;
            end
        end

        function shadowCopy(objs, axs)
            for obj = objs
                obj.ShadowGraphics = gobjects(size(axs));
                for i = 1:numel(axs)
                    obj.ShadowGraphics(i) = copy(obj.Graphics);
                    obj.ShadowGraphics(i).Parent = axs(i);
                end
            end
        end

        function stateName = storeState(objs, stateName)
        %storeState Stores the current kinematic state of a body. The kinematic state
        % consists of position, orientation, linear velocity and angular velocity.
        %
        %   stateName = storeState(bodies) stores the state for all given
        %   bodies under an auto-generated name, which is returned as
        %   output.
        %
        %   storeState(bodies, stateName) stores the state under given
        %   name. The name must respect conditions for naming variables.
        %
        % See also phx.Body.restoreState, phx.Body.clearStates

            arguments
                objs (1, :)
                stateName (1, 1) string = ""
            end

            for obj = objs
                if stateName == ""
                    stateName = "state"+(numel(fieldnames(obj.States)) + 1);
                end
                obj.States.(stateName) = obj.stateTransfer;
            end
        end

        function stateName = restoreState(objs, stateName)
        %restoreState Restores the kinematic state (position and velocity) of a body.
        %
        %   stateName = restoreState(bodies) restores the last stored state for all
        %   given bodies and returns the name of this restored state as output.
        %
        %   restoreState(bodies, stateName) restores the given state.
        %
        % See also phx.Body.storeState, phx.Body.clearStates

            arguments
                objs (1, :)
                stateName (1, 1) string = ""
            end

            for obj = objs
                if stateName == ""
                    names = fieldnames(obj.States);
                    if ~isempty(names)
                        stateName = string(names{end});
                    else
                        stateName = [];
                        return
                    end
                end
                obj.stateTransfer(obj.States.(stateName));
            end
        end

        function clearStates(objs, stateNames)
        %clearStates Removes one or more kinematic state of a body.
        %
        %   clearStates(bodies) removes all stored states for all given
        %   bodies.
        %
        %   clearStates(bodies, stateNames) removes given states for all
        %   given bodies. Multiple states can be passed as a vector of
        %   strings.
        %
        % See also phx.Body.storeState, phx.Body.restoreState

            arguments
                objs (1, :)
                stateNames (1, :) string = ""
            end

            for obj = objs
                fields = fieldnames(obj.States);
                obj.States = rmfield(obj.States, fields(contains(fields, stateNames)));
            end
        end

        function set.Shape(obj, shape)
            switch class(shape)
                case 'cell'
                    if endsWith(shape{1}, ".stl")
                        shape = phx.shape.STL("Source", shape{:});
                    else
                        shape = feval("phx.shape."+shape{1}, shape{2:end});
                    end
            end

            shape.drawTo(obj.Graphics);
            [obj.Mass, obj.Inertia] = shape.computeMass;
        end

        function value = get.Transform(obj)
            value = obj.Matrix;
        end

        function set.Transform(obj, value)
            obj.Matrix = value;
            obj.matrixUpdated;
        end

        function value = get.Position(obj)
            value = obj.Matrix(13:15);
        end

        function set.Position(obj, value)
            obj.Matrix(13:15) = value;
            obj.matrixUpdated;
        end

        function value = get.Orientation(obj)
            value = obj.Matrix(1:3, 1:3);
            value = value./sqrt(sum(value.^2)); % normalize to remove scale
        end

        function set.Orientation(obj, value)
            obj.Matrix(1:3, 1:3) = value;
            obj.matrixUpdated;
        end

        function value = get.AxisAngle(obj)
            value = phx.internal.Math.decompAA(obj.Matrix(1:3, 1:3));
        end

        function set.AxisAngle(obj, value)
            obj.Matrix(1:3, 1:3) = phx.internal.Math.rotAA(value(1:3), value(4));
            obj.matrixUpdated;
        end

        function value = get.EulerAngles(obj)
            value = phx.internal.Math.decomp321(obj.Matrix(1:3, 1:3));
        end

        function set.EulerAngles(obj, value)
            obj.Matrix(1:3, 1:3) = phx.internal.Math.rot321(value);
            obj.matrixUpdated;
        end

        function value = get.Quaternion(obj)
            value = phx.internal.Math.decompQ(obj.Matrix(1:3, 1:3));
        end

        function set.Quaternion(obj, value)
            obj.Matrix(1:3, 1:3) = phx.internal.Math.rotQ(value);
            obj.matrixUpdated;
        end

        function value = get.TotalForce(obj)
            if ~isempty(obj.ObjectHandle)
                value = phx.engine.io('get', obj.WorldHandle, obj.ObjectHandle, 'totalforce')';
            else
                value = [0 0 0];
            end
        end

        function value = get.TotalTorque(obj)
            if ~isempty(obj.ObjectHandle)
                value = phx.engine.io('get', obj.WorldHandle, obj.ObjectHandle, 'totaltorque')';
            else
                value = [0 0 0];
            end
        end

        function value = get.Energy(obj)
            v = obj.LinearVelocity;
            w = obj.AngularVelocity*obj.Matrix(1:3, 1:3); % world -> local frame
            value = (obj.Mass*(v*v') + sum(obj.Inertia.*w.^2))/2;
        end

        function set.Type(obj, value)
            obj.Type = value;
            if value == "static"
                obj.RedrawOrder = "none";
            else
                obj.RedrawOrder = "after";
            end
            if ~isempty(obj.ObjectHandle)
                phx.engine.io('set', obj.WorldHandle, obj.ObjectHandle, 'state', obj.TypeID);
            end
        end

        function value = get.TypeID(obj)
            value = find(["static", "kinematic", "dynamic"] == obj.Type) - 1;
        end

        function set.Mass(obj, value)
            obj.Mass = value;
            if ~isempty(obj.ObjectHandle)
                phx.engine.io('set', obj.WorldHandle, obj.ObjectHandle, 'mass', value);
            end
        end

        function set.Inertia(obj, value)
            obj.Inertia = value;
            if ~isempty(obj.ObjectHandle)
                phx.engine.io('set', obj.WorldHandle, obj.ObjectHandle, 'inertia', value);
            end            
        end

        % function value = get.Inertia(obj)
        %     if ~isempty(obj.ObjectHandle)
        %         value = phx.engine.io('get', obj.WorldHandle, obj.ObjectHandle, 'inertia');
        %     else
        %         value = obj.Inertia;
        %     end
        % end

        function set.Friction(obj, coeffs)
            obj.Friction = coeffs;
            if ~isempty(obj.ObjectHandle)
                phx.engine.io('set', obj.WorldHandle, obj.ObjectHandle, 'friction', coeffs(1));
                phx.engine.io('set', obj.WorldHandle, obj.ObjectHandle, 'rollfriction', coeffs(2));
                phx.engine.io('set', obj.WorldHandle, obj.ObjectHandle, 'spinfriction', coeffs(3));
            end
        end

        function set.Restitution(obj, value)
            obj.Restitution = value;
            if ~isempty(obj.ObjectHandle)
                phx.engine.io('set', obj.WorldHandle, obj.ObjectHandle, 'restitution', value);
            end
        end

        function set.Collisions(obj, enable)
            obj.Collisions = enable;
            if ~isempty(obj.ObjectHandle)
                phx.engine.io('set', obj.WorldHandle, obj.ObjectHandle, 'collisions', enable);
            end
        end

        function set.AngularVelocity(obj, value)
            obj.AngularVelocity = value;
            if ~isempty(obj.ObjectHandle)
                phx.engine.io('set', obj.WorldHandle, obj.ObjectHandle, 'angvel', value);
            end    
        end

        function value = get.AngularVelocity(obj)
            if ~isempty(obj.ObjectHandle)
                value = phx.engine.io('get', obj.WorldHandle, obj.ObjectHandle, 'angvel')';
            else
                value = obj.AngularVelocity;
            end    
        end

        function set.LinearVelocity(obj, value)
            obj.LinearVelocity = value;
            if ~isempty(obj.ObjectHandle)
                phx.engine.io('set', obj.WorldHandle, obj.ObjectHandle, 'linvel', value);
            end    
        end

        function value = get.LinearVelocity(obj)
            if ~isempty(obj.ObjectHandle)
                value = phx.engine.io('get', obj.WorldHandle, obj.ObjectHandle, 'linvel')';
            else
                value = obj.LinearVelocity;
            end    
        end

        function applyForce(obj, Force, Point, IsLocalForce, IsLocalPoint)
        %applyForce Applies a force to the body acting during one subsequent
        % simulation step. After this step, the applied force is reset.
        %
        %   applyForce(body, force) applies force in the point of origin of
        %   the body.
        %
        %   applyForce(body, force, point) applies force in the given point
        %   in the local space of the body.
        %
        %   applyForce(body, force, point, isLocalForce, isLocalPoint)
        %   specifies if the point and force are in the local (true, default)
        %   or global (false) space.
        %
        % See also phx.Body.applyTorque

            arguments
                obj 
                Force (:, 3) double
                Point (:, 3) double = []
                IsLocalForce (:, 1) logical = true
                IsLocalPoint (:, 1) logical = true 
            end

            if isempty(Point)
                phx.engine.io('apply', obj.WorldHandle, obj.ObjectHandle, 'centralforce', Force, IsLocalForce);
            else
                phx.engine.io('apply', obj.WorldHandle, obj.ObjectHandle, 'force', Force, Point, IsLocalForce, IsLocalPoint);
            end
        end

        function applyTorque(obj, Torque, IsLocalTorque)
        %applyTorque Applies a torque to the body acting during one subsequent
        % simulation step. After this step, the applied torque is reset.
        %
        %   applyTorque(body, torque) applies torque (in any point of the body).
        %
        %   applyTorque(body, torque, isLocalTorque) specifies if the torque is
        %   in the local (true, default) or global (false) space.
        %
        % See also phx.Body.applyForce

            arguments
                obj 
                Torque (:, 3) double
                IsLocalTorque (:, 1) logical = true
            end

            phx.engine.io('apply', obj.WorldHandle, obj.ObjectHandle, 'torque', Torque, IsLocalTorque);
        end

        function groupTransform(objs, transforms)
        %groupTransform Applies a common transformation to a group of bodies.
        %
        %   groupTransform(bodies, Name, Value, ...) rigidly transforms a
        %   group of bodies while preserving their mutual positions and
        %   orientations. The first body in the array is used as the origin
        %   of the group; the remaining bodies are moved rigidly with respect
        %   to it.
        %
        %   The transformation is relative to the current pose of the group
        %   and is specified by one or more of the following name-value
        %   arguments, which are combined together:
        %   - Transform:   4x4 transformation matrix
        %   - Translation: 1x3 translation vector [x y z]
        %   - EulerAngles: 1x3 vector of Euler angles [x y z]
        %   - AxisAngle:   1x4 vector of axis and angle [x y z, angle]
        %
        %   Rotations are applied about the group origin (the first body) and
        %   the translation is added in the world coordinate system.
        %
        % See also phx.Body.Transform

            arguments
                objs (1, :) phx.Body
                transforms.Transform (4, 4) double
                transforms.Translation (1, 3) double
                transforms.AxisAngle (1, 4) double
                transforms.EulerAngles (1, 3) double
            end

            if isfield(transforms, 'Transform')
                P = transforms.Transform(13:15);
                R = transforms.Transform(1:3, 1:3);
            else
                P = [0 0 0];
                R = eye(3);
            end

            if isfield(transforms, 'Translation')
                P = P + transforms.Translation;
            end

            if isfield(transforms, 'EulerAngles')
                R = R*phx.internal.Math.rot321(transforms.EulerAngles);
            end

            if isfield(transforms, 'AxisAngle')
                R = R*phx.internal.Math.rotAA(transforms.AxisAngle(1:3), transforms.AxisAngle(4));
            end

            % Use first body as an origin of the group
            obj = objs(1);
            oldOrigin = obj.Matrix;
            obj.Matrix(1:3, 1:3) = obj.Matrix(1:3, 1:3)*R;
            obj.Matrix(13:15) = obj.Matrix(13:15) + P;
            obj.matrixUpdated;
            newOrigin = obj.Matrix;

            % Transform other bodies according to the origin of the group
            for obj = objs(2:end)
                d = oldOrigin\obj.Matrix;
                obj.Matrix = newOrigin*d;
                obj.matrixUpdated;
            end

            for obj = objs
                for i = 1:numel(obj.Children)
                    ch = obj.Children{i};
                    feval(class(ch)+".updateView", obj.Children(i));
                end
            end
        end

        function transform = offset(obj, translation, rotation)
        %OFFSET Returns a transformation matrix with relative translation
        % and rotation to the object itself.
        %
        %   M = offset(body, translation) returns a matrix for relative
        %   translation only. The translation input argument should be a
        %   1x3 vector.
        %
        %   M = offset(body, translation, rotation) returns matrix for both
        %   relative translation and rotation. According to the size of the
        %   rotation arguments it will be interpreted as:
        %   - 1x1 (scalar): rotation around the Z axis
        %   - 1x3 (vector): euler angles
        %   - 1x4 (vector): axis and angle ([x y z, angle])
        %
        % See also phx.Body.Transform

            arguments
                obj
                translation (1, 3) double
                rotation (1, :) double = 0
            end

            switch numel(rotation)
                case 1
                    T = makehgtform("translate", translation, "zrotate", rotation);
                case 3
                    T = makehgtform("translate", translation, "zrotate", rotation(3), "yrotate", rotation(2), "xrotate", rotation(1));
                case 4
                    T = makehgtform("translate", translation, "axisrotate", rotation(1:3), rotation(4));
                otherwise
                    error("phx:Body:badRotationFormat", "Unsupported rotation format (expected 1, 3 or 4 elements, got %d).", numel(rotation));
            end

            transform = obj.Matrix*T;
        end

        % function dispHull(objs)
        %     for obj = objs
        %         v = phx.engine.io('get', obj.WorldHandle, obj.ObjectHandle, 'vertices');
        %         DT = delaunayTriangulation(v');
        %         %patch(obj.Graphics, "Faces", DT.ConnectivityList, "Vertices", DT.Points, "FaceColor", "none", "EdgeColor", obj.Shape.Color/2);
        %         patch(obj.Graphics, "Faces", DT.ConnectivityList, "Vertices", DT.Points, "FaceColor", obj.Shape.Color, "EdgeColor", "none", "FaceAlpha", 0.25);
        %     end
        % end

        function overlay(obj, type, state)
        %overlay Shows or hides an auxiliary visual overlay of a body.
        %
        %   overlay(body, type) toggles the overlay of the given type on or
        %   off (it is created if not present, otherwise it is removed).
        %
        %   overlay(body, type, state) explicitly turns the overlay "on" or
        %   "off". The default state "switch" toggles the current state.
        %
        %   The type argument selects what is drawn:
        %   - "center": a marker at the body origin
        %   - "system": the body coordinate system (RGB axes triad)
        %   - "bbox":   the bounding box wireframe
        %   - "edit":   an interactive editing gizmo (bounding box and axes)
        %   - "hull":   the convex hull of the body geometry
        %
        % See also phx.Body.boundingBox

            arguments
                obj
                type {mustBeMember(type, ["center", "system", "bbox", "edit", "hull"])}
                state {mustBeMember(state, ["on", "off", "switch"])} = "switch"
            end

            if isempty(obj.ParentAxes)
                return
            end

            tag = "phx_"+type;
            tr = findobj(obj.Graphics, 'Tag', tag);
            
            if state == "switch"
                if isempty(tr)
                    state = "on";
                else
                    state = "off";
                end
            end

            if state == "on"
                if isempty(tr)
                    tr = hgtransform(obj.Graphics, 'Tag', tag);
                else
                    return
                end
            else
                if ~isempty(tr)
                    delete(tr);
                end
                return
            end

            vertices = [-1 -1 -1; 1 1 1];
            for ch = obj.Graphics.Children'
                ph = phx.internal.PrimitiveHelper(ch);
                v = ph.Vertices;
                if ~isempty(v)
                    vertices = v;
                    break
                end
            end

            switch type
                case "center"
                    matlab.graphics.primitive.world.Marker('Parent', tr, 'VertexData', single([0; 0; 0]), 'EdgeColorData', uint8([0 0 0 255]'), 'Style', 'point', 'Size', 20, 'Layer', 'front');
                    matlab.graphics.primitive.world.Marker('Parent', tr, 'VertexData', single([0; 0; 0]), 'EdgeColorData', uint8([255 255 255 255]'), 'Style', 'point', 'Size', 10, 'Layer', 'front');
                case "system"
                    [a, b] = bounds(vertices);
                    s = max(abs(b - a))/3;
                    matlab.graphics.primitive.world.LineStrip('Parent', tr, 'ColorData', uint8([0 0 0 255]'), 'ColorBinding', 'object', 'LineWidth', 3, 'VertexData', single([0 1 0 0 0 0; 0 0 0 1 0 0; 0 0 0 0 0 1]*s), 'Layer', 'front');
                    matlab.graphics.primitive.world.LineStrip('Parent', tr, 'ColorData', uint8([255 0 0 255]'), 'ColorBinding', 'object', 'LineWidth', 1, 'VertexData', single([0.02 0.98; 0 0; 0 0]*s), 'Layer', 'front');
                    matlab.graphics.primitive.world.LineStrip('Parent', tr, 'ColorData', uint8([0 204 0 255]'), 'ColorBinding', 'object', 'LineWidth', 1, 'VertexData', single([0 0; 0.02 0.98; 0 0]*s), 'Layer', 'front');
                    matlab.graphics.primitive.world.LineStrip('Parent', tr, 'ColorData', uint8([0 0 255 255]'), 'ColorBinding', 'object', 'LineWidth', 1, 'VertexData', single([0 0; 0 0; 0.02 0.98]*s), 'Layer', 'front');
                case "bbox"
                    [a, b] = bounds(vertices);
                    [v, ~, f] = phx.internal.Geometry.quadBox((b - a));
                    c = (a + b)/2;
                    v = v + c;
                    patch('Vertices', v, 'Faces', f, 'EdgeColor', [1 1 1], 'FaceColor', 'none', 'Parent', tr, 'HitTest', 'off');
                case "edit"
                    [a, b] = bounds(vertices);
                    s = max(abs(b - a))/3;
                    [v, ~, f] = phx.internal.Geometry.quadBox((b - a));
                    c = (a + b)/2;
                    v = v + c;
                    patch('Vertices', v, 'Faces', f, 'EdgeColor', [1 1 0], 'LineWidth', 1, 'FaceColor', 'none', 'Parent', tr, 'HitTest', 'off');
                    matlab.graphics.primitive.world.LineStrip('Parent', tr, 'ColorData', uint8([0 0 0 255]'), 'ColorBinding', 'object', 'LineWidth', 5, 'VertexData', single([0 1 0 0 0 0; 0 0 0 1 0 0; 0 0 0 0 0 1]*s), 'Layer', 'front');
                    matlab.graphics.primitive.world.LineStrip('Parent', tr, 'ColorData', uint8([255 0 0 255]'), 'ColorBinding', 'object', 'LineWidth', 3, 'VertexData', single([0.02 0.98; 0 0; 0 0]*s), 'Layer', 'front', 'HitTest', 'on');
                    matlab.graphics.primitive.world.LineStrip('Parent', tr, 'ColorData', uint8([0 204 0 255]'), 'ColorBinding', 'object', 'LineWidth', 3, 'VertexData', single([0 0; 0.02 0.98; 0 0]*s), 'Layer', 'front', 'HitTest', 'on');
                    matlab.graphics.primitive.world.LineStrip('Parent', tr, 'ColorData', uint8([0 0 255 255]'), 'ColorBinding', 'object', 'LineWidth', 3, 'VertexData', single([0 0; 0 0; 0.02 0.98]*s), 'Layer', 'front', 'HitTest', 'on');
                case "hull"
                    v = vertices;
                    cv = convhull(v, "Simplify", true);
                    trisurf(cv, v(:, 1), v(:, 2), v(:, 3), 'FaceColor', [1 1 1], 'EdgeColor', 'none', 'FaceAlpha', 0.5, 'Parent', tr, 'HitTest', 'off');
            end
        end

        function bbox = boundingBox(objs)
        %boundingBox Returns the axis-aligned bounding box of one or more bodies.
        %
        %   bbox = boundingBox(bodies) returns the axis-aligned bounding box,
        %   in world coordinates, that encloses the visual geometry of all
        %   given bodies. The result is a 2x3 matrix:
        %   - first row:  minimum corner [xmin ymin zmin]
        %   - second row: maximum corner [xmax ymax zmax]
        %
        % See also phx.Body.Position, phx.Body.Transform

            arguments
                objs (1, :) phx.Body
            end

            bbmin = [Inf Inf Inf];
            bbmax = [-Inf -Inf -Inf];
            for obj = objs
                for ch = obj.Graphics.Children'
                    ph = phx.internal.PrimitiveHelper(ch);
                    v = ph.Vertices;
                    if ~isempty(v)
                        v = v*obj.Graphics.Matrix(1:3, 1:3) + obj.Graphics.Matrix(13:15);
                        [bmin, bmax] = bounds(v);
                        bbmin = min(bbmin, bmin);
                        bbmax = max(bbmax, bmax);
                    end
                end
            end

            bbox = [bbmin; bbmax];
        end

        function ax = gca(obj)
        %gca Returns the main hgtransform object of a body.
        %
        %   ax = gca(body) returns the handle of the graphics transform that
        %   holds the visual representation of the body. It can be used as a
        %   parent for custom graphics attached to the body.
        %
        % See also phx.Body.boundingBox, hgtransform

            ax = obj.Graphics;
        end
    end

    methods (Access = ?phx.base.Object)
        function matrixUpdated(obj)
            if ~isempty(obj.ObjectHandle)
                phx.engine.io('set', obj.WorldHandle, obj.ObjectHandle, 'transform', obj.Matrix);
            else
                obj.Graphics.Matrix = obj.Matrix;
            end
        end

        function state = stateTransfer(obj, state)
            arguments
                obj 
                state = []
            end

            if isempty(state)
                state.Transform = obj.Transform;
                state.LinearVelocity = obj.LinearVelocity;
                state.AngularVelocity = obj.AngularVelocity;
            else
                obj.Transform = state.Transform;
                obj.LinearVelocity = state.LinearVelocity;
                obj.AngularVelocity = state.AngularVelocity;
            end
        end
    end

    methods (Access = protected)
        function valid = initObject(obj, world)
            valid = false;
            if isempty(obj.ObjectHandle)
                state = obj.stateTransfer;
                
                obj.WorldHandle = world;

                for i = 1:numel(obj.Graphics.Children)
                    primitive = obj.Graphics.Children(i);
                    phxShape = getappdata(primitive, "phxShape");
                    if ~isempty(phxShape)
                        obj.ObjectHandle = phxShape.createBody(obj, primitive);
                        valid = true;
                        break
                    end
                end
                
                obj.stateTransfer(state);
                obj.Friction = obj.Friction;
                obj.Restitution = obj.Restitution;
                obj.Collisions = obj.Collisions;
            else
                valid = true;
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
            % implemented within the phx.Simulation.resolveState
        end

        function updateView(cellObjs, dt, time, world)
            for i = 1:numel(cellObjs)
                obj = cellObjs{i};
                obj.Graphics.Matrix_I = obj.Matrix;
                for j = 1:numel(obj.ShadowGraphics)
                    obj.ShadowGraphics(j).Matrix_I = obj.Matrix;
                end
            end
        end
    end

end