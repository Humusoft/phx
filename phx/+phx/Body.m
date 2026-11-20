classdef Body < phx.base.Object
%phx.Body Rigid body with pose, mass, friction and forces
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

% TODO collision groups

    % Internal
    properties (Access = ?phx.base.Object)
        Matrix = eye(4)
    end

    properties (SetAccess = private)
        States = struct
    end

    % Kinematic state in a global coordinate system
    properties
        % Linear velocity
        LinearVelocity = [0 0 0]

        % Angular velocity
        AngularVelocity = [0 0 0]
    end

    % Kinematic state in a global coordinate system
    properties (Dependent)
        % Full transform matrix (4x4)
        Transform (4, 4) double {mustBeFinite}

        % Linear position
        Position (1, 3) double {mustBeFinite}

        % Rotation matrix (3x3)
        Orientation (3, 3) double {mustBeFinite}

        % Axis-angle rotation
        AxisAngle (1, 4) double {mustBeFinite}

        % Euler angles rotation for z->y->x order
        EulerAngles (1, 3) double {mustBeFinite}

        % Quaternion rotation
        Quaternion (1, 4) double {mustBeFinite}

        % Direction of the body X axis, it turns the body from its current
        % orientation by the smallest rotation that points the axis there
        XAxis (1, 3) double {mustBeFinite}

        % Direction of the body Y axis, it turns the body from its current
        % orientation by the smallest rotation that points the axis there
        YAxis (1, 3) double {mustBeFinite}

        % Direction of the body Z axis, it turns the body from its current
        % orientation by the smallest rotation that points the axis there
        ZAxis (1, 3) double {mustBeFinite}
    end

    % Read-only state computed by the engine
    properties (Dependent, SetAccess = private)
        % Resultant of all forces on the body in the last engine step (N,
        % world frame): applied loads, gravity, contacts and joint reactions
        TotalForce (1, 3) double

        % Resultant torque on the body in the last engine step about its
        % centre of mass (N*m, world frame)
        TotalTorque (1, 3) double

        % Mean linear acceleration of the centre of mass over the last
        % step (m/s^2), gravity included: a falling body reads g
        LinearAcceleration (1, 3) double

        % Mean angular acceleration over the last step (rad/s^2)
        AngularAcceleration (1, 3) double

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
        % - kinematic: moves only as driven. A pose (Position, EulerAngles,
        %   ...) is reached within the next engine step, and then the body
        %   stands still: set it before every step, without substeps, or
        %   drive the body by LinearVelocity and AngularVelocity, which hold
        %   until changed
        % - dynamic
        Type {mustBeMember(Type, ["static", "kinematic", "dynamic"])} = "dynamic";

        % Body mass (kg)
        Mass (1, 1) double = 0

        % Principal moments of inertia (kg*m^2) about the centre of mass, in
        % the body axes
        Inertia (1, 3) double = [0 0 0]

        % Centre of mass (m) in the body frame, relative to the body origin
        % The shape gives it, as it gives Mass and Inertia; the body turns
        % about it and gravity acts at it. Changing it, also while the
        % simulation runs, moves neither the body nor its Inertia, which is
        % taken about the new centre as it is.
        CenterOfMass (1, 3) double {mustBeFinite} = [0 0 0]

        % Friction coefficient (Coulomb): a contact resists sliding, and a
        % contact patch turning about its normal, by this coefficient times
        % the normal force; it combines the values of both bodies as their
        % geometric mean, sqrt(mu1*mu2). [mu 0 0] is still accepted as mu.
        Friction (1, :) double {mustBeFrictionValue} = 0.5

        % Rolling resistance (m): the arm of the normal force N that resists
        % rolling, a torque of up to RollingResistance*N about any axis in
        % the contact plane, whatever the friction; a contact adds the values
        % of both bodies. With 0 on both, a ball rolls on forever.
        RollingResistance (1, 1) double {mustBeNonnegative, mustBeFinite} = 0

        % Spin radius (m): the effective radius of the contact patch that a
        % point contact (a ball on a floor) stands for; the contact resists
        % spinning about its normal by mu*SpinRadius*N (mu the contact's
        % combined friction coefficient), in addition to
        % what a flat patch resists by its own extent. A contact adds the
        % values of both bodies; about 2/3 of the radius of the real patch.
        SpinRadius (1, 1) double {mustBeNonnegative, mustBeFinite} = 0

        % Restitution (bounciness) coefficient, 0 = no bounce; a contact
        % bounces with the smaller of the two colliding bodies' values, so
        % a body of 1 (an ideal rigid floor) keeps the other's bounce
        Restitution (1, 1) double {mustBeGreaterThanOrEqual(Restitution, 0)} = 0

        % Enable collision response for this body
        Collisions (1, 1) logical = true

        % Virtual velocity of the body surface (m/s) in the body frame, e.g. for
        % conveyor belts; the body does not move, but contacts with it are solved
        % as if its surface were sliding, so other bodies are dragged along within
        % the limits of the contact friction
        SurfaceVelocity (1, 3) double {mustBeFinite} = [0 0 0]

        % Force at the centre of mass (N) in the body frame
        % It acts in every step, turning with the body, until changed; set it
        % to [0 0 0] to stop it.
        AppliedForce (1, 3) double {mustBeFinite} = [0 0 0]

        % Torque (N*m) in the body frame, acting like AppliedForce until changed
        AppliedTorque (1, 3) double {mustBeFinite} = [0 0 0]

        % Custom callback for double-click action on any part of this body
        % (works only in the phx.extra.Viewer)
        OnDoubleClickFcn = []
    end

    properties (Dependent, Hidden, SetAccess = private)
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
                mass = obj.Mass;
                inertia = obj.Inertia;
                center = obj.CenterOfMass;
                obj.Shape = phx.shape.Box;
                if mass ~= 0
                    obj.Mass = mass;
                end
                if sum(inertia) ~= 0
                    obj.Inertia = inertia;
                end
                obj.CenterOfMass = center; % the box has its own at the origin
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

            if stateName == ""
                % One past the highest auto-generated name on any of the
                % bodies, so a name freed by clearStates is never reused
                last = 0;
                for obj = objs
                    names = string(fieldnames(obj.States));
                    auto = names(matches(names, "state" + digitsPattern));
                    last = max([last; str2double(extractAfter(auto, "state"))]);
                end
                stateName = "state" + (last + 1);
            end

            for obj = objs
                if isfield(obj.States, stateName)
                    % Re-storing a name makes it the last stored state
                    obj.States = rmfield(obj.States, stateName);
                end
                obj.States.(stateName) = obj.stateTransfer;
            end
        end

        function stateName = restoreState(objs, stateName)
        %restoreState Restores the kinematic state (position and velocity) of a body.
        %
        %   stateName = restoreState(bodies) restores, for each given body,
        %   the state it stored last; bodies without a stored state are left
        %   as they are. Returns the name of the state restored on the first
        %   such body, or an empty string when no body has a stored state.
        %
        %   restoreState(bodies, stateName) restores the given state. Every
        %   body must have it, otherwise no body is restored.
        %
        % See also phx.Body.storeState, phx.Body.clearStates

            arguments
                objs (1, :)
                stateName (1, 1) string = ""
            end

            if stateName == ""
                stateName = string.empty;
                for obj = objs
                    names = fieldnames(obj.States);
                    if ~isempty(names)
                        obj.stateTransfer(obj.States.(names{end}));
                        if isempty(stateName)
                            stateName = string(names{end});
                        end
                    end
                end
                return
            end

            for i = 1:numel(objs)
                if ~isfield(objs(i).States, stateName)
                    error("phx:Body:unknownState", ...
                        "Body %d of %d has no stored state named ""%s"".", i, numel(objs), stateName);
                end
            end
            for obj = objs
                obj.stateTransfer(obj.States.(stateName));
            end
        end

        function clearStates(objs, stateNames)
        %clearStates Removes one or more kinematic states of a body.
        %
        %   clearStates(bodies) removes all stored states for all given
        %   bodies.
        %
        %   clearStates(bodies, stateNames) removes given states for all
        %   given bodies. Multiple states can be passed as a vector of
        %   strings; names a body does not have are ignored.
        %
        % See also phx.Body.storeState, phx.Body.restoreState

            arguments
                objs (1, :)
                stateNames (1, :) string = string.empty
            end

            for obj = objs
                fields = fieldnames(obj.States);
                if isempty(stateNames)
                    obj.States = rmfield(obj.States, fields);
                else
                    obj.States = rmfield(obj.States, fields(ismember(fields, stateNames)));
                end
            end
        end

        function set.Shape(obj, shape)
            switch class(shape)
                case 'cell'
                    if endsWith(lower(shape{1}), [".stl", ".obj", ".ply"])
                        shape = phx.shape.Mesh("Source", shape{:});
                    else
                        shape = feval("phx.shape."+shape{1}, shape{2:end});
                    end
            end

            shape.drawTo(obj.Graphics);
            [obj.Mass, obj.Inertia, obj.CenterOfMass] = shape.computeMass;
        end

        function value = get.Transform(obj)
            value = obj.Matrix;
        end

        function set.Transform(obj, value)
            obj.Matrix = value;
            obj.matrixUpdated;
        end

        function value = get.Position(obj)
            % Scalar indexing into a local copy is twice as fast as Matrix(13:15)
            m = obj.Matrix;
            value = [m(13) m(14) m(15)];
        end

        function set.Position(obj, value)
            obj.Matrix(13:15) = value;
            obj.matrixUpdated;
        end

        function value = get.Orientation(obj)
            m = obj.Matrix;
            value = [m(1) m(5) m(9); m(2) m(6) m(10); m(3) m(7) m(11)];
            value = value./sqrt(sum(value.^2)); % normalize to remove scale
        end

        function set.Orientation(obj, value)
            obj.Matrix(1:3, 1:3) = value;
            obj.matrixUpdated;
        end

        function value = get.AxisAngle(obj)
            value = phx.internal.Math.decompAA(obj.Matrix); % reads only the rotation part
        end

        function set.AxisAngle(obj, value)
            obj.Matrix(1:3, 1:3) = phx.internal.Math.rotAA(value(1:3), value(4));
            obj.matrixUpdated;
        end

        function value = get.EulerAngles(obj)
            value = phx.internal.Math.decomp321(obj.Matrix); % reads only the rotation part
        end

        function set.EulerAngles(obj, value)
            obj.Matrix(1:3, 1:3) = phx.internal.Math.rot321(value);
            obj.matrixUpdated;
        end

        function value = get.Quaternion(obj)
            value = phx.internal.Math.decompQ(obj.Matrix); % reads only the rotation part
        end

        function set.Quaternion(obj, value)
            obj.Matrix(1:3, 1:3) = phx.internal.Math.rotQ(value);
            obj.matrixUpdated;
        end

        function value = get.XAxis(obj)
            m = obj.Matrix;
            value = [m(1) m(2) m(3)]/norm([m(1) m(2) m(3)]);
        end

        function set.XAxis(obj, value)
            obj.Matrix(1:3, 1:3) = alignBodyAxis(obj, 1, value, "XAxis");
            obj.matrixUpdated;
        end

        function value = get.YAxis(obj)
            m = obj.Matrix;
            value = [m(5) m(6) m(7)]/norm([m(5) m(6) m(7)]);
        end

        function set.YAxis(obj, value)
            obj.Matrix(1:3, 1:3) = alignBodyAxis(obj, 2, value, "YAxis");
            obj.matrixUpdated;
        end

        function value = get.ZAxis(obj)
            m = obj.Matrix;
            value = [m(9) m(10) m(11)]/norm([m(9) m(10) m(11)]);
        end

        function set.ZAxis(obj, value)
            obj.Matrix(1:3, 1:3) = alignBodyAxis(obj, 3, value, "ZAxis");
            obj.matrixUpdated;
        end

        function value = get.TotalForce(obj)
            if ~isempty(obj.ObjectHandle)
                value = phxEngine('get', obj.WorldHandle, obj.ObjectHandle, 'totalforce');
            else
                value = [0 0 0];
            end
        end

        function value = get.TotalTorque(obj)
            if ~isempty(obj.ObjectHandle)
                value = phxEngine('get', obj.WorldHandle, obj.ObjectHandle, 'totaltorque');
            else
                value = [0 0 0];
            end
        end

        function value = get.LinearAcceleration(obj)
            if ~isempty(obj.ObjectHandle)
                value = phxEngine('get', obj.WorldHandle, obj.ObjectHandle, 'linacc');
            else
                value = [0 0 0];
            end
        end

        function value = get.AngularAcceleration(obj)
            if ~isempty(obj.ObjectHandle)
                value = phxEngine('get', obj.WorldHandle, obj.ObjectHandle, 'angacc');
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
                phxEngine('set', obj.WorldHandle, obj.ObjectHandle, 'type', obj.TypeID);
            end
        end

        function value = get.TypeID(obj)
            value = find(["static", "kinematic", "dynamic"] == obj.Type) - 1;
        end

        function set.Mass(obj, value)
            obj.Mass = value;
            if ~isempty(obj.ObjectHandle)
                phxEngine('set', obj.WorldHandle, obj.ObjectHandle, 'massprops', value, obj.Inertia);
            end
        end

        function set.Inertia(obj, value)
            obj.Inertia = value;
            if ~isempty(obj.ObjectHandle)
                phxEngine('set', obj.WorldHandle, obj.ObjectHandle, 'massprops', obj.Mass, value);
            end
        end

        function set.CenterOfMass(obj, value)
            obj.CenterOfMass = value;
            if ~isempty(obj.ObjectHandle)
                phxEngine('set', obj.WorldHandle, obj.ObjectHandle, 'com', value);
            end
        end

        function set.Friction(obj, value)
            if numel(value) == 3
                % [drag roll spin] of PHX 1.0: the rolling and spinning values
                % meant something else and have no equivalent
                if any(value(2:3))
                    error("phx:Body:frictionTriplet", ...
                        "Friction is the friction coefficient alone. Rolling and spinning friction " + ...
                        "are the properties RollingResistance and SpinRadius, lengths in m with a " + ...
                        "meaning different from [drag roll spin]; set them instead.");
                end
                value = value(1);
            end
            obj.Friction = value;
            obj.pushFriction;
        end

        function set.RollingResistance(obj, value)
            obj.RollingResistance = value;
            obj.pushFriction;
        end

        function set.SpinRadius(obj, value)
            obj.SpinRadius = value;
            obj.pushFriction;
        end

        function set.Restitution(obj, value)
            obj.Restitution = value;
            if ~isempty(obj.ObjectHandle)
                phxEngine('set', obj.WorldHandle, obj.ObjectHandle, 'restitution', value);
            end
        end

        function set.Collisions(obj, enable)
            obj.Collisions = enable;
            if ~isempty(obj.ObjectHandle)
                phxEngine('set', obj.WorldHandle, obj.ObjectHandle, 'collisions', enable);
            end
        end

        function set.SurfaceVelocity(obj, value)
            obj.SurfaceVelocity = value;
            if ~isempty(obj.ObjectHandle)
                phxEngine('set', obj.WorldHandle, obj.ObjectHandle, 'surfacevelocity', value);
            end
        end

        function set.AppliedForce(obj, value)
            obj.AppliedForce = value;
            if ~isempty(obj.ObjectHandle)
                phxEngine('set', obj.WorldHandle, obj.ObjectHandle, 'appliedforce', value);
            end
        end

        function set.AppliedTorque(obj, value)
            obj.AppliedTorque = value;
            if ~isempty(obj.ObjectHandle)
                phxEngine('set', obj.WorldHandle, obj.ObjectHandle, 'appliedtorque', value);
            end
        end

        function set.AngularVelocity(obj, value)
            obj.AngularVelocity = value;
            if ~isempty(obj.ObjectHandle)
                phxEngine('set', obj.WorldHandle, obj.ObjectHandle, 'angvel', value);
            end    
        end

        function value = get.AngularVelocity(obj)
            if ~isempty(obj.ObjectHandle)
                value = phxEngine('get', obj.WorldHandle, obj.ObjectHandle, 'angvel');
            else
                value = obj.AngularVelocity;
            end    
        end

        function set.LinearVelocity(obj, value)
            obj.LinearVelocity = value;
            if ~isempty(obj.ObjectHandle)
                phxEngine('set', obj.WorldHandle, obj.ObjectHandle, 'linvel', value);
            end    
        end

        function value = get.LinearVelocity(obj)
            if ~isempty(obj.ObjectHandle)
                value = phxEngine('get', obj.WorldHandle, obj.ObjectHandle, 'linvel');
            else
                value = obj.LinearVelocity;
            end    
        end

        function applyForce(obj, Force, Point, IsLocalForce, IsLocalPoint)
        %applyForce Applies a force to the body for the next engine step.
        %
        %   The force acts during the next engine step only, which is one
        %   substep of phx.Simulation.step: before step(sim, interval, n) it
        %   acts for interval/n. A force that lasts is AppliedForce, or
        %   applyForce called in every substep from a phx.Function.
        %
        %   applyForce(body, force) applies force at the centre of mass of
        %   the body.
        %
        %   applyForce(body, force, point) applies force at the given point
        %   in the local space of the body (relative to its origin).
        %
        %   applyForce(body, force, point, isLocalForce, isLocalPoint)
        %   specifies if the point and force are in the local (true, default)
        %   or global (false) space.
        %
        %   Several forces at once come as the rows of force and point (N-by-3),
        %   with the flags given once for all or once per row.
        %
        % See also phx.Body.applyTorque

            arguments
                obj 
                Force (:, 3) double
                Point (:, 3) double = []
                IsLocalForce (:, 1) logical = true
                IsLocalPoint (:, 1) logical = true 
            end

            % One engine batch of the same body, a force per row
            hs = repmat(obj.ObjectHandle, 1, size(Force, 1));
            if isempty(Point)
                phxEngine('apply', obj.WorldHandle, hs, 'centralforce', Force', IsLocalForce);
            else
                phxEngine('apply', obj.WorldHandle, hs, 'force', Force', Point', IsLocalForce, IsLocalPoint);
            end
        end

        function applyTorque(obj, Torque, IsLocalTorque)
        %applyTorque Applies a torque to the body for the next engine step.
        %
        %   The torque acts during the next engine step only, one substep of
        %   phx.Simulation.step, as for applyForce. A torque that lasts is
        %   AppliedTorque.
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

            hs = repmat(obj.ObjectHandle, 1, size(Torque, 1));
            phxEngine('apply', obj.WorldHandle, hs, 'torque', Torque', IsLocalTorque);
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
                transforms.Transform (4, 4) double {mustBeFinite}
                transforms.Translation (1, 3) double {mustBeFinite}
                transforms.AxisAngle (1, 4) double {mustBeFinite}
                transforms.EulerAngles (1, 3) double {mustBeFinite}
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

        function guide(obj, type, state)
        %guide Shows or hides an auxiliary guide graphic of a body.
        %
        %   guide(body, type) toggles the guide of the given type on or off
        %   (it is created if not present, otherwise it is removed).
        %
        %   guide(body, type, state) explicitly turns the guide "on" or
        %   "off". The default state "switch" toggles the current state.
        %
        %   Guides are auxiliary graphics, not part of the body's shape. The
        %   type argument selects what is drawn:
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
                        v = v*obj.Graphics.Matrix(1:3, 1:3)' + obj.Graphics.Matrix(13:15);
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
                phxEngine('set', obj.WorldHandle, obj.ObjectHandle, 'transform', obj.Matrix);
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
                obj.pushFriction;
                obj.Restitution = obj.Restitution;
                obj.Collisions = obj.Collisions;
                obj.SurfaceVelocity = obj.SurfaceVelocity;
                % The persistent loads go with the engine body, so a new one
                % takes them over
                if valid && any(obj.AppliedForce)
                    obj.AppliedForce = obj.AppliedForce;
                end
                if valid && any(obj.AppliedTorque)
                    obj.AppliedTorque = obj.AppliedTorque;
                end
            else
                valid = true;
            end
        end

        function destroyObject(obj)
            if ~isempty(obj.ObjectHandle)
                phxEngine('remove', obj.WorldHandle, obj.ObjectHandle);
                obj.ObjectHandle = [];
            end
        end
    end

    methods (Access = private)
        function pushFriction(obj)
            % Hands Friction, RollingResistance and SpinRadius to the engine
            if ~isempty(obj.ObjectHandle)
                phxEngine('set', obj.WorldHandle, obj.ObjectHandle, 'friction', ...
                    [obj.Friction obj.RollingResistance obj.SpinRadius]);
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
            end
        end
    end

end

function R = alignBodyAxis(body, k, value, name)
%alignBodyAxis Orientation of a body turned so that its axis K points along VALUE
    if ~any(value)
        error("phx:Body:invalidAxis", "%s must be a nonzero direction vector.", name);
    end
    R = phx.internal.Math.alignAxis(body.Orientation, k, value);
end

function mustBeFrictionValue(value)
%mustBeFrictionValue Validates Friction: a coefficient, or [mu 0 0] of PHX 1.0
    if ~any(numel(value) == [1 3]) || any(~isfinite(value)) || any(value < 0)
        error("phx:Body:badFriction", ...
            "Friction must be a finite, nonnegative friction coefficient.");
    end
end
