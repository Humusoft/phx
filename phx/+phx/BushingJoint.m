classdef BushingJoint < phx.base.Joint
%phx.BushingJoint Bushing joint (compliant fixed joint)
%
%   Bushing joint realizes a compliant counterpart of a fixed joint - an
%   elastic bushing that holds two bodies together with per-axis linear and
%   angular stiffness and damping across all 6 degrees of freedom. Where
%   phx.FixedJoint locks the relative pose rigidly, phx.BushingJoint lets it
%   deflect against a springy bedding.
%
%   Very stiff springs are clamped by the solver for stability (the ceiling
%   scales with the bodies' mass/inertia and shrinks with the time step), so
%   above a certain stiffness the response stops getting stiffer. Use more
%   substeps (a smaller time step) for stiffer bushings, or phx.FixedJoint
%   for a truly rigid connection.
%
%   Optional hard end stops bound the travel: LowerLinearLimits and
%   UpperLinearLimits (m) for translation, LowerAngularLimits and
%   UpperAngularLimits (rad) for rotation. On each axis, setting lower > upper
%   disables the stop, leaving the axis free (spring only) - this is the
%   default; lower == upper locks the axis rigidly at that value; lower < upper
%   limits the travel to that range while the spring acts inside it.
%
%   The stiffness, damping and limits all act along and about the axes of the
%   joint coordinate system of body A, the angles being measured in the
%   right-handed sense. Rotation about its Y axis is limited to roughly
%   +-90 deg by the solver and degenerates near that value.
%
%   phx.BushingJoint(bodyA, bodyB) creates a bushing joint between two bodies
%   A and B with the joint coordinate system of each of them coinciding with
%   the local coordinate system of that body.
%   Custom joint coordinate systems can be set using the TransformA and
%   TransformB properties, or via the PointA, PointB, EulerAnglesA and
%   EulerAnglesB helpers.
%
%   phx.BushingJoint(___, name, value, ...) creates a bushing joint and sets
%   properties values according to given name-value pairs.
%
%   See also phx.FixedJoint

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
        % Linear stiffness
        LinearStiffness(1, 3) double = [1e4 1e4 1e4]

        % Linear damping
        LinearDamping(1, 3) double = [1e3 1e3 1e3]

        % Angular stiffness
        AngularStiffness(1, 3) double = [1e3 1e3 1e3]

        % Angular damping
        AngularDamping(1, 3) double = [1e2 1e2 1e2]

        % Lower linear limits (hard stop)
        % Set lower > upper to completely disable limits (default)
        LowerLinearLimits(1, 3) double = [1 1 1]

        % Upper linear limits (hard stop)
        % Set lower > upper to completely disable limits (default)
        UpperLinearLimits(1, 3) double = [-1 -1 -1]

        % Lower angular limits (hard stop)
        % Set lower > upper to completely disable limits (default)
        LowerAngularLimits(1, 3) double = [1 1 1]

        % Upper angular limits (hard stop)
        % Set lower > upper to completely disable limits (default)
        UpperAngularLimits(1, 3) double = [-1 -1 -1]

        % Draw joint as overlay
        Overlay (1, 1) logical = false
    end

    methods
        function obj = BushingJoint(ParentA, ParentB, Options)
            arguments
                ParentA (1, 1) {mustBeA(ParentA, "phx.Body")}
                ParentB (1, 1) {mustBeA(ParentB, "phx.Body")}
                Options.?phx.BushingJoint
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
            phx.BushingJoint.updateView({obj});
        end
    end

    methods (Access = protected)
        function valid = initObject(obj, world)
            valid = numel(obj.Parents) == 2 && all(cellfun(@isvalid, obj.Parents));
            if valid
                obj.WorldHandle = world;
                obj.ObjectHandle = phx.engine.io('add', world, 'generic6dofspringconstraint', obj.Parents{1}.ObjectHandle, obj.Parents{2}.ObjectHandle, obj.TransformA(:), obj.TransformB(:), 'xyz', ~obj.MutualCollisions);
                isOnLinear = double(obj.LinearStiffness | obj.LinearDamping);
                isOnAngular = double(obj.AngularStiffness | obj.AngularDamping);
                phx.engine.io('set', obj.WorldHandle, obj.ObjectHandle, 'springinfo', isOnLinear, isOnAngular, obj.LinearDamping, obj.AngularDamping, obj.LinearStiffness, obj.AngularStiffness);
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
