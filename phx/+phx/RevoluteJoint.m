classdef RevoluteJoint < phx.base.Joint
%phx.RevoluteJoint Revolute joint
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
%   Both motor properties can be changed while the simulation runs or set from
%   Simulink.
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

        % Draw joint as overlay
        Overlay (1, 1) logical = false
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

            % Process input arguments
            obj.Parents = addChild([ParentA ParentB], obj);
            phx.internal.applyArguments(Options, obj);

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
    end

    methods (Access = private)
        function applyMotor(obj)
        %applyMotor Pushes the motor setting to the engine.
        % A zero MaxTorque switches the motor off rather than clamping it to
        % zero, so that the solver does not assemble its constraint row at all.

            if ~isempty(obj.ObjectHandle)
                phx.engine.io('set', obj.WorldHandle, obj.ObjectHandle, 'motor', ...
                    obj.MaxTorque > 0, obj.TargetVelocity, obj.MaxTorque);
            end
        end
    end

    methods (Access = protected)
        function valid = initObject(obj, world)
            valid = numel(obj.Parents) == 2 && all(cellfun(@isvalid, obj.Parents));
            if valid
                % The constraint is rebuilt from scratch here, so the
                % previous one has to be taken out of the world first.
                obj.destroyObject;
                obj.WorldHandle = world;
                obj.ObjectHandle = phx.engine.io('add', world, 'hingeconstraint', obj.Parents{1}.ObjectHandle, obj.Parents{2}.ObjectHandle, ...
                    obj.PointA, obj.PointB, obj.AxisA, obj.AxisB, true, ~obj.MutualCollisions);
                % The constraint is created anew on every pipeline rebuild, so
                % the motor setting has to be reapplied here
                obj.applyMotor;
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
