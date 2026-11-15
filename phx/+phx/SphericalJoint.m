classdef SphericalJoint < phx.base.Joint
%phx.SphericalJoint Spherical joint (ball joint, 3 DOF)
%
%   Spherical joint realizes a kinematic constraint with 3 degrees of freedom
%   specified as rotations around the X, Y and Z axis.
%
%   phx.SphericalJoint(bodyA, bodyB) creates a joint between two bodies A and B
%   attached to their points of origin.
%   Custom connection points can be set using PointA and PointB properties.
%
%   ConeAngle and TwistLimits bound the rotation with hard end stops that do
%   not bounce; their reaction is part of ForceA, TorqueA, ForceB and TorqueB.
%   ConeAngle bounds the swing of the Z axis of the joint coordinate system of
%   body B: [angleX angleY] are the largest angles it may turn by about the X
%   and the Y axis of that of body A, the half-angles of an elliptic cone
%   about the Z axis of A; a scalar makes the cone circular, 0 locks the
%   swing in that direction and pi, the default, leaves it free (any angle
%   above about 179.5 deg bounds nothing). TwistLimits bounds the rotation
%   about the Z axis to [lower upper], as Limits of phx.RevoluteJoint does:
%   both within (-pi, pi), or [-Inf Inf], the default, for no stop. The twist
%   is measured after the swing, so at zero swing it is the Angle of the
%   joint. A circular cone is exact; an elliptic one is exact on its axes and
%   close to an elliptic cone between them. Near a swing of pi the twist is
%   undefined, so bound the twist only with a cone well inside that.
%
%   With the limits, the axes of the joint coordinate systems matter: set
%   them with AxisA and AxisB, TransformA and TransformB or the EulerAngles
%   helpers (the Z axis is the axis of the cone and of the twist). Free, the
%   joint uses only the points.
%
%   TwistLimits = 0 with a free cone makes a constant-velocity joint: body B
%   turns about its Z axis by just the angle body A turns about its own,
%   whatever the angle between the two axes.
%
%   ConeAngle and TwistLimits can be changed while the simulation runs or set
%   from Simulink.
%
%   phx.SphericalJoint(___, name, value, ...) creates a joint and sets properties
%   values according to given name-value pairs.
%
%   See also phx.RevoluteJoint, phx.FixedJoint, phx.GenericJoint

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
        % Largest swing about the X and Y axes [angleX angleY] (rad, hard stops)
        % Half-angles of a cone about the Z axis of frame A within [0, pi]: a
        % scalar for a circular cone, 0 to lock that direction, pi to leave
        % it free (default).
        ConeAngle (1, 2) double {phx.internal.mustBeConeAngle} = [pi pi]

        % Range of the twist about the Z axis [lower upper] (rad, hard stops)
        % Both within (-pi, pi), equal to lock the twist at that angle, or
        % [-Inf Inf] to leave it free (default).
        TwistLimits (1, 2) double {phx.internal.mustBeAngularLimits} = [-Inf Inf]
    end

    methods
        function set.ConeAngle(obj, value)
            obj.ConeAngle = value;
            if ~isempty(obj.ObjectHandle)
                phxEngine('set', obj.WorldHandle, obj.ObjectHandle, 'cone', value);
            end
        end

        function set.TwistLimits(obj, value)
            obj.TwistLimits = value;
            if ~isempty(obj.ObjectHandle)
                phxEngine('set', obj.WorldHandle, obj.ObjectHandle, 'anglimits', value(1), value(2));
            end
        end

        function obj = SphericalJoint(ParentA, ParentB, Options)
            arguments
                ParentA (1, 1) {mustBeA(ParentA, "phx.Body")}
                ParentB (1, 1) {mustBeA(ParentB, "phx.Body")}
                Options.?phx.SphericalJoint
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
            phx.SphericalJoint.updateView({obj});
        end
    end

    methods (Access = protected)
        function valid = initObject(obj, world)
            valid = obj.checkObject;
            if valid && isempty(obj.ObjectHandle)
                % The engine takes the frames only as it creates the
                % constraint, so the constraint is built once and then kept
                % across pipeline rebuilds. Only ConeAngle and TwistLimits
                % write through from their setters; other properties changed
                % after that do not reach the constraint.
                obj.WorldHandle = world;
                obj.ObjectHandle = phxEngine('add', world, 'spherical', [obj.Parents{1}.ObjectHandle obj.Parents{2}.ObjectHandle], obj.TransformA, obj.TransformB, obj.MutualCollisions);
                % Seed the fresh constraint, created free, with the limits
                if any(obj.ConeAngle < pi)
                    phxEngine('set', world, obj.ObjectHandle, 'cone', obj.ConeAngle);
                end
                if any(isfinite(obj.TwistLimits))
                    phxEngine('set', world, obj.ObjectHandle, 'anglimits', obj.TwistLimits(1), obj.TwistLimits(2));
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