classdef CylindricalJoint < phx.base.Joint
%phx.CylindricalJoint Cylindrical joint (slide and spin about one axis, 2 DOF)
%
%   Cylindrical joint realizes a kinematic constraint with 2 degrees of freedom
%   specified as translation along an axis and rotation about the same axis - a
%   shaft free to slide and spin inside a sleeve. The axis is the Z axis of the
%   joint coordinate systems of both connected bodies; the remaining four
%   degrees of freedom are locked.
%
%   phx.CylindricalJoint(bodyA, bodyB) creates a joint between two bodies A and
%   B attached to their points of origin, with the joint axis aligned to axis Z
%   of the local coordinate system of each body.
%   Custom connection points and direction vectors of the joint axis can be set
%   using the PointA, PointB, AxisA and AxisB properties, whole joint coordinate
%   systems using TransformA and TransformB or the EulerAngles helpers.
%   Only the components perpendicular to the joint axis place the bodies with
%   respect to each other; an offset along the axis merely shifts the zero of
%   the free translation, and a roll about it the zero of the free rotation.
%
%   Unlike phx.PrismaticJoint, this joint leaves the rotation about the axis
%   free, so AxisA and AxisB alone fully describe it - the two coordinate
%   systems need not agree on how they are rolled about the axis.
%
%   LinearLimits bounds the travel to a range [lower upper] of the Distance
%   (m), AngularLimits the rotation to a range of the Angle (rad), with hard
%   end stops that do not bounce; their reaction is part of ForceA, TorqueA,
%   ForceB and TorqueB. Equal limits lock that motion at that value, and
%   [-Inf Inf], the default, leaves it free. A linear limit of -Inf or Inf
%   leaves out the stop on that side. As Angle, the angular limits lie
%   within (-pi, pi), and both are finite unless the rotation is free. A
%   body that passes an angular stop by more than half the gap behind the
%   stops within one step comes out at the other stop, so a fast joint needs
%   that gap. The limits can be changed while the simulation runs or set
%   from Simulink.
%
%   phx.CylindricalJoint(___, name, value, ...) creates a joint and sets
%   properties values according to given name-value pairs.
%
%   See also phx.PrismaticJoint, phx.RevoluteJoint, phx.GenericJoint

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
        % Range of the joint distance [lower upper] (m, hard stops)
        % Equal to lock the travel at that distance, -Inf or Inf for no stop
        % on that side, [-Inf Inf] to leave it free (default).
        LinearLimits (1, 2) double {phx.internal.mustBeLinearLimits} = [-Inf Inf]

        % Range of the joint angle [lower upper] (rad, hard stops)
        % Both within (-pi, pi), equal to lock the rotation at that angle, or
        % [-Inf Inf] to leave it free (default).
        AngularLimits (1, 2) double {phx.internal.mustBeAngularLimits} = [-Inf Inf]
    end

    methods
        function set.LinearLimits(obj, value)
            obj.LinearLimits = value;
            obj.pushLimits('linlimits', value);
        end

        function set.AngularLimits(obj, value)
            obj.AngularLimits = value;
            obj.pushLimits('anglimits', value);
        end

        function obj = CylindricalJoint(ParentA, ParentB, Options)
            arguments
                ParentA (1, 1) {mustBeA(ParentA, "phx.Body")}
                ParentB (1, 1) {mustBeA(ParentB, "phx.Body")}
                Options.?phx.CylindricalJoint
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
            phx.CylindricalJoint.updateView({obj});
        end
    end

    methods (Access = private)
        function pushLimits(obj, field, value)
        %pushLimits Writes the linear or angular limits through to the engine.
        % The joint is a 6-DOF constraint with the axes X and Y of frame A
        % locked (0 to 0) and Z bounded by the limits.

            if ~isempty(obj.ObjectHandle)
                phxEngine('set', obj.WorldHandle, obj.ObjectHandle, field, [0 0 value(1)], [0 0 value(2)]);
            end
        end
    end

    methods (Access = protected)
        function valid = initObject(obj, world)
            valid = obj.checkObject;
            if valid && isempty(obj.ObjectHandle)
                % The engine takes the frames only as it creates the
                % constraint, so the constraint is built once and then kept
                % across pipeline rebuilds. Only LinearLimits and
                % AngularLimits write through from their setters; other
                % properties changed after that do not reach the constraint.
                obj.WorldHandle = world;
                obj.ObjectHandle = phxEngine('add', world, 'generic', [obj.Parents{1}.ObjectHandle obj.Parents{2}.ObjectHandle], ...
                    obj.TransformA, obj.TransformB, obj.MutualCollisions);
                % The engine creates it with every axis locked: free the
                % axis within the current limits
                obj.pushLimits('linlimits', obj.LinearLimits);
                obj.pushLimits('anglimits', obj.AngularLimits);
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
