classdef RevoluteJoint < phx.base.Joint
%phx.RevoluteJoint Revolute joint (hinge, 1 DOF)
%
%   Revolute joint realizes a kinematic constraint with 1 degree of freedom
%   specified as rotations around the axis. The rotation axis is the Z axis of
%   the joint coordinate systems of both connected bodies.
%
%   phx.RevoluteJoint(bodyA, bodyB) creates a joint between two bodies A and B
%   attached to their points of origin and with rotation axes aligned to axis Z
%   of the local coordinate system of each body.
%   Custom connection points and direction vectors of rotation axes can be set
%   using PointA, PointB, AxisA and AxisB properties.
%
%   phx.RevoluteJoint(___, name, value, ...) creates a joint and sets properties
%   values according to given name-value pairs.
%
%   The joint can be driven by a motor. The motor regulates the joint velocity
%   to TargetVelocity as long as it needs less than MaxTorque, which is zero
%   (no motor) by default. Because the motor saturates, a deliberately
%   unreachable TargetVelocity turns it into a pure torque source delivering
%   MaxTorque, and TargetVelocity = 0 makes it a friction brake holding the
%   joint until the load exceeds MaxTorque.
%
%   AppliedTorque adds a torque about the axis, acting on the second body and,
%   reversed, on the first one until changed. It is independent of the motor:
%   the two act together, so a motor can regulate against an applied load.
%
%   Limits bounds the rotation to a range [lower upper] of the Angle (rad),
%   with hard end stops that do not bounce; their reaction is part of
%   ForceA, TorqueA, ForceB and TorqueB. Equal limits lock the joint at that
%   angle, and [-Inf Inf], the default, leaves it free. As Angle, the limits
%   lie within (-pi, pi), and both are finite unless the joint is free: the
%   rotation is bounded on both sides. A body that passes a stop by more
%   than half the gap behind the stops within one step comes out at the
%   other stop, so a fast joint needs that gap.
%
%   The motor properties, AppliedTorque and Limits can be changed while the
%   simulation runs or set from Simulink.
%
%   See also phx.SphericalJoint, phx.CylindricalJoint

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

%#ok<*MCSUP> OK to access other properties in setters
%#ok<*INUSD> OK to see the full list of arguments for callbacks

    properties (Access = private)
        hL
        hM
    end

    properties
        % Target motor velocity in the joint axis (rad/s)
        % The sign gives the direction of rotation.
        TargetVelocity (1, 1) double = 0

        % Maximal torque the motor can deliver (N*m), 0 = no motor
        % The motor regulates to TargetVelocity while it needs less than this;
        % when the target is out of reach it simply delivers MaxTorque.
        MaxTorque (1, 1) double {mustBeNonnegative} = 0

        % Torque applied about the joint axis (N*m)
        % Acts on the second body and, reversed, on the first one, until
        % changed, together with the motor if there is one.
        AppliedTorque (1, 1) double {mustBeFinite} = 0

        % Range of the joint angle [lower upper] (rad, hard stops)
        % Both within (-pi, pi), equal to lock the joint at that angle, or
        % [-Inf Inf] to leave it free (default).
        Limits (1, 2) double {phx.internal.mustBeAngularLimits} = [-Inf Inf]
    end

    methods
        function obj = RevoluteJoint(ParentA, ParentB, Options)
            arguments
                ParentA (1, 1) {mustBeA(ParentA, "phx.Body")}
                ParentB (1, 1) {mustBeA(ParentB, "phx.Body")}
                Options.?phx.RevoluteJoint
            end

            % Set default values
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

            % Create graphics objects
            clr = uint8([obj.Color*255 255]');
            obj.hL = matlab.graphics.primitive.world.LineStrip('Parent', obj.Graphics, 'LineWidth', 1.0, 'ColorBinding', 'object', 'ColorData', clr, 'Layer', phx.internal.choose({'middle', 'front'}, obj.Overlay + 1));
            obj.hM = matlab.graphics.primitive.world.Marker('Parent', obj.Graphics, 'EdgeColorData', clr, 'Style', 'circle', 'Size', 10, 'Layer', phx.internal.choose({'middle', 'front'}, obj.Overlay + 1));
            phx.RevoluteJoint.updateView({obj});
        end

        function set.TargetVelocity(obj, value)
            obj.TargetVelocity = value;
            obj.applyMotor;
        end

        function set.MaxTorque(obj, value)
            obj.MaxTorque = value;
            obj.applyMotor;
        end

        function set.AppliedTorque(obj, value)
            obj.AppliedTorque = value;
            if ~isempty(obj.ObjectHandle)
                phxEngine('set', obj.WorldHandle, obj.ObjectHandle, 'appliedtorque', value);
            end
        end

        function set.Limits(obj, value)
            obj.Limits = value;
            if ~isempty(obj.ObjectHandle)
                phxEngine('set', obj.WorldHandle, obj.ObjectHandle, 'anglimits', value(1), value(2));
            end
        end
    end

    methods (Access = private)
        function applyMotor(obj)
        %applyMotor Pushes the motor setting to the engine.
        % A zero MaxTorque switches the motor off rather than clamping it to
        % zero, so that the solver does not assemble its constraint row at all.

            if ~isempty(obj.ObjectHandle)
                phxEngine('set', obj.WorldHandle, obj.ObjectHandle, 'motor', ...
                    obj.MaxTorque > 0, obj.TargetVelocity, obj.MaxTorque);
            end
        end
    end

    methods (Access = protected)
        function valid = initObject(obj, world)
            valid = obj.checkObject;
            if valid && isempty(obj.ObjectHandle)
                % The engine takes the frames only as it creates the
                % constraint, so the constraint is built once and then kept
                % across pipeline rebuilds. Only the motor (TargetVelocity,
                % MaxTorque), AppliedTorque and Limits write through from
                % their setters; other properties changed after that do not
                % reach the constraint.
                obj.WorldHandle = world;
                obj.ObjectHandle = phxEngine('add', world, 'revolute', [obj.Parents{1}.ObjectHandle obj.Parents{2}.ObjectHandle], ...
                    obj.TransformA, obj.TransformB, obj.MutualCollisions);
                % Seed the fresh constraint with the current settings; later
                % changes reach the engine from the setters
                obj.applyMotor;
                if obj.AppliedTorque ~= 0
                    phxEngine('set', world, obj.ObjectHandle, 'appliedtorque', obj.AppliedTorque);
                end
                if any(isfinite(obj.Limits))
                    phxEngine('set', world, obj.ObjectHandle, 'anglimits', obj.Limits(1), obj.Limits(2));
                end
            end
        end
    end

    methods (Static, Access = protected)
        function resolveState(cellObjs, dt, time, world)
        end

        function updateView(cellObjs, dt, time, world)
            for i = 1:numel(cellObjs)
                obj = cellObjs{i};

                pa = phx.internal.transformPoint(obj.Parents{1}.Matrix, obj.PointA);
                pb = phx.internal.transformPoint(obj.Parents{2}.Matrix, obj.PointB);
                vd = single([pa' pb']);

                obj.hL.VertexData = vd;
                obj.hM.VertexData = vd;
            end
        end
    end

end
