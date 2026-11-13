classdef GenericJoint < phx.base.Joint
%phx.GenericJoint Generic joint (6 DOF with limits)
%
%   Generic joint realizes a kinematic constraint with 6 degrees of freedom -
%   three translations and three rotations - each of them independently free,
%   locked or bounded by a pair of hard end stops.
%
%   The degrees of freedom are the axes of the joint coordinate system of body
%   A: LowerLinearLimits and UpperLinearLimits (m) bound the translation of
%   body B along them, LowerAngularLimits and UpperAngularLimits (rad) bound
%   its rotation about them, measured in the right-handed sense. On each axis:
%
%     lower == upper  the axis is locked at that value - this is the default
%     lower < upper   the motion is bounded to that range
%     lower > upper   the axis is free (no constraint)
%
%   All limits default to zero, so a joint created without them is rigid, like
%   phx.FixedJoint; open only the axes that are meant to move. The limits can
%   be changed at any time, also while the simulation runs.
%
%   The rotation is split into a twist and a swing: the rotation about the Z
%   axis of the joint coordinate system is the twist, those about its X and Y
%   axes are the swing, and each of the three is limited separately. For a
%   rotation about one axis alone the angle is simply the angle of that
%   rotation; when several rotate at once, a free rotation is best left to
%   the Z axis (the twist), as phx.CylindricalJoint does.
%
%   phx.GenericJoint(bodyA, bodyB) creates a rigid joint between two bodies A
%   and B with the joint coordinate system of each of them coinciding with the
%   local coordinate system of that body.
%   Custom joint coordinate systems can be set using the TransformA and
%   TransformB properties, or via the PointA, PointB, EulerAnglesA and
%   EulerAnglesB helpers.
%
%   phx.GenericJoint(___, name, value, ...) creates a joint and sets properties
%   values according to given name-value pairs.
%
%   See also phx.BushingJoint, phx.FixedJoint, phx.PrismaticJoint,
%   phx.RevoluteJoint, phx.CylindricalJoint

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
        % Lower linear limits along the joint axes (hard stop, m)
        % Set lower == upper to lock the axis at that value (default),
        % lower > upper to leave it free
        LowerLinearLimits(1, 3) double = [0 0 0]

        % Upper linear limits along the joint axes (hard stop, m)
        % Set lower == upper to lock the axis at that value (default),
        % lower > upper to leave it free
        UpperLinearLimits(1, 3) double = [0 0 0]

        % Lower angular limits about the joint axes (hard stop, rad)
        % Set lower == upper to lock the axis at that value (default),
        % lower > upper to leave it free
        LowerAngularLimits(1, 3) double = [0 0 0]

        % Upper angular limits about the joint axes (hard stop, rad)
        % Set lower == upper to lock the axis at that value (default),
        % lower > upper to leave it free
        UpperAngularLimits(1, 3) double = [0 0 0]
    end

    methods
        function obj = GenericJoint(ParentA, ParentB, Options)
            arguments
                ParentA (1, 1) {mustBeA(ParentA, "phx.Body")}
                ParentB (1, 1) {mustBeA(ParentB, "phx.Body")}
                Options.?phx.GenericJoint
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
            phx.GenericJoint.updateView({obj});
        end

        function set.LowerLinearLimits(obj, value)
            obj.LowerLinearLimits = value;
            obj.pushLimits("linlimits");
        end

        function set.UpperLinearLimits(obj, value)
            obj.UpperLinearLimits = value;
            obj.pushLimits("linlimits");
        end

        function set.LowerAngularLimits(obj, value)
            obj.LowerAngularLimits = value;
            obj.pushLimits("anglimits");
        end

        function set.UpperAngularLimits(obj, value)
            obj.UpperAngularLimits = value;
            obj.pushLimits("anglimits");
        end
    end

    methods (Access = private)
        function pushLimits(obj, field)
        %pushLimits Writes the linear or angular limits through to the engine.
            if ~isempty(obj.ObjectHandle)
                if field == "linlimits"
                    phxEngine('set', obj.WorldHandle, obj.ObjectHandle, 'linlimits', obj.LowerLinearLimits, obj.UpperLinearLimits);
                else
                    phxEngine('set', obj.WorldHandle, obj.ObjectHandle, 'anglimits', obj.LowerAngularLimits, obj.UpperAngularLimits);
                end
            end
        end
    end

    methods (Access = protected)
        function valid = initObject(obj, world)
            valid = obj.checkObject;
            if valid && isempty(obj.ObjectHandle)
                % The frames are taken as the engine creates the constraint,
                % which is then kept across pipeline rebuilds; the limits
                % write through from their setters at any time.
                obj.WorldHandle = world;
                obj.ObjectHandle = phxEngine('add', world, 'generic', [obj.Parents{1}.ObjectHandle obj.Parents{2}.ObjectHandle], ...
                    obj.TransformA, obj.TransformB, obj.MutualCollisions);
                obj.pushLimits("linlimits");
                obj.pushLimits("anglimits");
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