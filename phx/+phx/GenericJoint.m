classdef GenericJoint < phx.base.Joint
%phx.GenericJoint Generic joint
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
%   phx.FixedJoint; open only the axes that are meant to move. The limits are
%   read when the simulation is built, so set them at construction time -
%   assigning them later has no effect on a running simulation.
%
%   Rotation about the Y axis of the joint coordinate system is limited to
%   roughly +-90 deg by the solver and degenerates near that value. Leave the Y
%   axis locked or narrowly bounded, and orient the joint coordinate systems so
%   that a freely rotating axis is X or Z.
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

        % Draw joint as overlay
        Overlay (1, 1) logical = false
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

            % Process input arguments
            obj.Parents = addChild([ParentA ParentB], obj);
            phx.internal.applyArguments(Options, obj);

            % Create graphics objects
            clr = uint8([obj.Color*255 255]');
            obj.hL = matlab.graphics.primitive.world.LineStrip('Parent', obj.Graphics, 'LineWidth', 1.0, 'ColorBinding', 'object', 'ColorData', clr, 'Layer', phx.internal.choose({'middle', 'front'}, obj.Overlay + 1));
            obj.hM = matlab.graphics.primitive.world.Marker('Parent', obj.Graphics, 'EdgeColorData', clr, 'Style', 'circle', 'Size', 10, 'Layer', phx.internal.choose({'middle', 'front'}, obj.Overlay + 1));
            phx.GenericJoint.updateView({obj});
        end
    end

    methods (Access = protected)
        function valid = initObject(obj, world)
            valid = numel(obj.Parents) == 2 && all(cellfun(@isvalid, obj.Parents));
            if valid
                obj.WorldHandle = world;
                obj.ObjectHandle = phx.engine.io('add', world, 'generic6dofconstraint', obj.Parents{1}.ObjectHandle, obj.Parents{2}.ObjectHandle, obj.TransformA(:), obj.TransformB(:), 'xyz', ~obj.MutualCollisions);
                phx.engine.io('set', obj.WorldHandle, obj.ObjectHandle, 'linlimits', obj.LowerLinearLimits, obj.UpperLinearLimits);
                % The engine measures the angles the other way round, so the
                % limits are negated and swapped to keep the right-handed sense
                phx.engine.io('set', obj.WorldHandle, obj.ObjectHandle, 'anglimits', -obj.UpperAngularLimits, -obj.LowerAngularLimits);
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