classdef TorsionSpring < phx.base.Object
%phx.TorsionSpring Torsion spring-damper between two bodies
%
%   TorsionSpring is a combined element that consists of an ideal torsion
%   spring, an ideal rotary damper and, with a nonzero Inertance, an ideal
%   rotary inerter in parallel, acting about the z axis of TransformA.
%
%   phx.TorsionSpring(bodyA, bodyB) creates a torsion spring between two
%   bodies A and B about the z axis of their frames of origin.
%   The frames are set by TransformA and TransformB as for the joints; use
%   those of a phx.RevoluteJoint to spring its hinge.
%
%   phx.TorsionSpring(___, name, value, ...) creates a torsion spring and
%   sets properties values according to given name-value pairs.
%
%   The angle is that of the joints (phx.base.Joint.Angle): of the x axis of
%   frame B about the z axis of frame A, but followed continuously while the
%   simulation runs, so that the spring winds over any number of turns. The
%   engine solves it together with the joints and contacts, by a step that
%   keeps its energy and is stable at any stiffness, damping and inertance.
%   The frames are read when the simulation starts.
%
%   See also phx.Spring, phx.RevoluteJoint

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

%#ok<*MCSUP> OK to access other properties in setters
%#ok<*INUSD> OK to see the full list of arguments for callbacks

    properties (Access = private)
        hL
    end

    properties (Dependent, SetAccess = private)
        % Torque acting on the first body (N*m); the second body gets its opposite
        Torque (1, 3) double

        % Angle of frame B about the axis (rad), continuous over the turns
        % while the simulation runs, within [-pi, pi] before
        Angle (1, 1) double

        % Elastic energy stored in the spring (J)
        Energy (1, 1) double
    end

    properties
        % Frame of the spring in the local space of the first body; the
        % spring acts about its z axis
        TransformA (4, 4) double = eye(4)

        % Frame of the spring in the local space of the second body
        TransformB (4, 4) double = eye(4)

        % Angle at which the spring is relaxed (rad)
        RestAngle (1, 1) double {mustBeFinite} = 0

        % Torsional stiffness of the spring (N*m/rad)
        Stiffness (1, 1) double {mustBeNonnegative, mustBeFinite} = 1

        % Rotary damping coefficient of the damper (N*m*s/rad)
        Damping (1, 1) double {mustBeNonnegative, mustBeFinite} = 0

        % Rotary inertance of an inerter in parallel (kg*m^2): a torque
        % proportional to the relative angular acceleration about the axis
        Inertance (1, 1) double {mustBeNonnegative, mustBeFinite} = 0

        % Radius of the drawn symbol (m)
        Radius (1, 1) double {mustBePositive} = 0.1
    end

    methods
        function obj = TorsionSpring(ParentA, ParentB, Options)
            arguments
                ParentA (1, 1) {mustBeA(ParentA, "phx.Body")}
                ParentB (1, 1) {mustBeA(ParentB, "phx.Body")}
                Options.?phx.TorsionSpring
            end

            % Set default values; the engine does the work of the steps
            obj.SimulationOrder = "none";
            obj.RedrawOrder = "after";
            obj.ParentAxes = ParentA.ParentAxes;

            % Process input arguments; a failure leaves no half-built object
            % attached to the parents
            try
                obj.Parents = addChild([ParentA ParentB], obj);
                phx.internal.applyArguments(Options, obj);
            catch err
                obj.abandon([ParentA ParentB]);
                rethrow(err);
            end

            % Create graphics objects: a circle about the axis and a pointer
            % to the angle
            obj.hL = matlab.graphics.primitive.world.LineStrip('Parent', obj.Graphics, 'LineWidth', 2.0, 'ColorBinding', 'object', 'ColorData', uint8([obj.Color*255 255]'), 'StripData', uint32([1 27 29]), 'Layer', phx.internal.choose({'middle', 'front'}, obj.Overlay + 1));
            phx.TorsionSpring.updateView({obj});
        end

        function value = get.Torque(obj)
            value = [0 0 0];
            if ~isempty(obj.ObjectHandle)
                M = phx.engine.io('get', obj.WorldHandle, obj.ObjectHandle, 'spring');
                Ma = obj.Parents{1}.Matrix*obj.TransformA;
                value = -M*Ma(9:11);
            end
        end

        function value = get.Angle(obj)
            if ~isempty(obj.ObjectHandle)
                [~, value] = phx.engine.io('get', obj.WorldHandle, obj.ObjectHandle, 'spring');
            else
                value = obj.wrappedAngle;
            end
        end

        function value = get.Energy(obj)
            value = obj.Stiffness*(obj.Angle - obj.RestAngle)^2/2;
        end

        function set.RestAngle(obj, value)
            obj.RestAngle = value;
            if ~isempty(obj.ObjectHandle)
                phx.engine.io('set', obj.WorldHandle, obj.ObjectHandle, 'angle', value);
            end
        end

        function set.Stiffness(obj, value)
            obj.Stiffness = value;
            obj.pushSpring;
        end

        function set.Damping(obj, value)
            obj.Damping = value;
            obj.pushSpring;
        end

        function set.Inertance(obj, value)
            obj.Inertance = value;
            if ~isempty(obj.ObjectHandle)
                phx.engine.io('set', obj.WorldHandle, obj.ObjectHandle, 'inertance', value);
            end
        end
    end

    methods (Access = private)
        function value = wrappedAngle(obj)
            % As phx.base.Joint.Angle
            Ma = obj.Parents{1}.Matrix*obj.TransformA;
            Mb = obj.Parents{2}.Matrix*obj.TransformB;
            xb = Mb(1:3)';
            value = atan2(Ma(5:7)*xb, Ma(1:3)*xb);
        end

        function pushSpring(obj)
            if ~isempty(obj.ObjectHandle)
                phx.engine.io('set', obj.WorldHandle, obj.ObjectHandle, 'spring', obj.Stiffness, obj.Damping);
            end
        end
    end

    methods (Access = protected)
        function valid = checkObject(obj)
            % A torsion spring twists two bodies against each other, so it
            % needs both of them.
            valid = numel(obj.Parents) == 2 && all(cellfun(@isvalid, obj.Parents));
        end

        function valid = initObject(obj, world)
            valid = obj.checkObject;
            if valid && isempty(obj.ObjectHandle)
                % Built once and kept across pipeline rebuilds; the setters
                % update it, except for the frames
                obj.WorldHandle = world;
                obj.ObjectHandle = phx.engine.io('add', world, 'torsionspringconstraint', ...
                    obj.Parents{1}.ObjectHandle, obj.Parents{2}.ObjectHandle, obj.TransformA, obj.TransformB, ...
                    obj.RestAngle, obj.Stiffness, obj.Damping);
                if obj.Inertance > 0
                    phx.engine.io('set', world, obj.ObjectHandle, 'inertance', obj.Inertance);
                end
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
            t = linspace(0, 2*pi, 26);
            for i = 1:numel(cellObjs)
                obj = cellObjs{i};

                % A circle about the axis at the origin of frame A, and a
                % pointer at the angle of frame B
                Ma = obj.Parents{1}.Matrix*obj.TransformA;
                c = Ma(13:15)';
                x = Ma(1:3)';
                y = Ma(5:7)';
                a = obj.wrappedAngle;
                circle = c + obj.Radius*(x*cos(t) + y*sin(t));
                pointer = [c, c + obj.Radius*(x*cos(a) + y*sin(a))];
                obj.hL.VertexData = single([circle, pointer]);
            end
        end
    end

end
