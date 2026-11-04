classdef PrismaticJoint < phx.base.Joint
%phx.PrismaticJoint Prismatic joint (slider, 1 DOF)
%
%   Prismatic joint realizes a kinematic constraint with 1 degree of freedom
%   specified as translation along the sliding axis. The sliding axis is the
%   Z axis of the joint coordinate systems of both connected bodies.
%
%   phx.PrismaticJoint(bodyA, bodyB) creates a joint between two bodies A and B
%   attached to their points of origin, with the sliding axis aligned to axis Z
%   of the local coordinate system of each body.
%   Custom sliding axes can be set using the AxisA and AxisB properties, custom
%   joint coordinate systems using TransformA and TransformB or the PointA,
%   PointB, EulerAnglesA and EulerAnglesB helpers; in every case the sliding
%   axis is the local Z axis of each coordinate system.
%   Only the components perpendicular to the sliding axis place the bodies with
%   respect to each other; an offset along the sliding axis merely shifts the
%   zero of the free translation.
%
%   Because this joint also locks the rotation about the sliding axis, AxisA
%   and AxisB alone do not fully describe it: they fix where each coordinate
%   system points its Z axis, but not how the two are rolled about it. They are
%   enough when both bodies share an orientation, as a body and a part sliding
%   straight out of it usually do. When they do not, set TransformA and
%   TransformB (or the EulerAngles helpers) so that both coordinate systems
%   coincide in every axis, otherwise the solver twists the bodies into line
%   over the first steps.
%
%   phx.PrismaticJoint(___, name, value, ...) creates a joint and sets properties
%   values according to given name-value pairs.
%
%   The joint can be driven by a motor. The motor regulates the sliding velocity
%   to TargetVelocity as long as it needs less than MaxForce, which is zero (no
%   motor) by default. Because the motor saturates, a deliberately unreachable
%   TargetVelocity turns it into a pure force source delivering MaxForce, and
%   TargetVelocity = 0 makes it a friction brake holding the joint until the
%   load exceeds MaxForce.
%
%   AppliedForce adds a force along the axis, acting on the second body and,
%   reversed, on the first one until changed. It is independent of the motor:
%   the two act together, so a motor can regulate against an applied load.
%
%   The motor properties and AppliedForce can be changed while the simulation
%   runs or set from Simulink.
%
%   See also phx.RevoluteJoint, phx.CylindricalJoint, phx.FixedJoint

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
        % Target motor velocity along the joint axis (m/s)
        % The sign gives the direction of travel.
        TargetVelocity (1, 1) double = 0

        % Maximal force the motor can deliver (N), 0 = no motor
        % The motor regulates to TargetVelocity while it needs less than this;
        % when the target is out of reach it simply delivers MaxForce.
        MaxForce (1, 1) double {mustBeNonnegative} = 0

        % Force applied along the joint axis (N)
        % Acts on the second body and, reversed, on the first one, until
        % changed, together with the motor if there is one.
        AppliedForce (1, 1) double {mustBeFinite} = 0
    end

    methods
        function set.TargetVelocity(obj, value)
            obj.TargetVelocity = value;
            obj.applyMotor;
        end

        function set.MaxForce(obj, value)
            obj.MaxForce = value;
            obj.applyMotor;
        end

        function set.AppliedForce(obj, value)
            obj.AppliedForce = value;
            if ~isempty(obj.ObjectHandle)
                phx.engine.io('set', obj.WorldHandle, obj.ObjectHandle, 'appliedforce', value);
            end
        end

        function obj = PrismaticJoint(ParentA, ParentB, Options)
            arguments
                ParentA (1, 1) {mustBeA(ParentA, "phx.Body")}
                ParentB (1, 1) {mustBeA(ParentB, "phx.Body")}
                Options.?phx.PrismaticJoint
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
            phx.PrismaticJoint.updateView({obj});
        end
    end

    methods (Access = private)
        function applyMotor(obj)
        %applyMotor Pushes the motor setting to the engine.
        % A zero MaxForce switches the motor off rather than clamping it to
        % zero, so that the solver does not assemble its constraint row at all.

            if ~isempty(obj.ObjectHandle)
                phx.engine.io('set', obj.WorldHandle, obj.ObjectHandle, 'motor', ...
                    obj.MaxForce > 0, obj.TargetVelocity, obj.MaxForce);
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
                % MaxForce) and AppliedForce write through from their setters;
                % other properties changed after that do not reach the
                % constraint.
                obj.WorldHandle = world;
                obj.ObjectHandle = phx.engine.io('add', world, 'prismatic', [obj.Parents{1}.ObjectHandle obj.Parents{2}.ObjectHandle], ...
                    obj.TransformA, obj.TransformB, obj.MutualCollisions);
                % Seed the fresh constraint with the current settings; later
                % changes reach the engine from the setters
                obj.applyMotor;
                if obj.AppliedForce ~= 0
                    phx.engine.io('set', world, obj.ObjectHandle, 'appliedforce', obj.AppliedForce);
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
