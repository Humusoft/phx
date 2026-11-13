classdef BushingJoint < phx.base.Joint
%phx.BushingJoint Bushing joint (compliant fixed joint)
%
%   Bushing joint realizes a compliant counterpart of a fixed joint - an
%   elastic bushing that holds two bodies together with per-axis linear and
%   angular stiffness and damping across all 6 degrees of freedom. Where
%   phx.FixedJoint locks the relative pose rigidly, phx.BushingJoint lets it
%   deflect against a springy bedding.
%
%   The springs are solved implicitly, so a bushing stays stable at any
%   stiffness and a very stiff one holds almost like phx.FixedJoint. The
%   implicit step damps a little of its own, the more the coarser the time
%   step is compared to the period of the spring.
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
%   right-handed sense. The rotation is split into a twist about the Z axis
%   and a swing about the X and Y axes, as for phx.GenericJoint. The springs
%   and the limits can be changed at any time, also while the simulation
%   runs.
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
%   See also phx.FixedJoint, phx.GenericJoint, phx.Spring

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
        % Linear stiffness along the joint axes (N/m), 0 = no spring
        LinearStiffness(1, 3) double {mustBeFinite, mustBeNonnegative} = [1e4 1e4 1e4]

        % Linear damping along the joint axes (N*s/m)
        LinearDamping(1, 3) double {mustBeFinite, mustBeNonnegative} = [1e3 1e3 1e3]

        % Angular stiffness about the joint axes (N*m/rad), 0 = no spring
        AngularStiffness(1, 3) double {mustBeFinite, mustBeNonnegative} = [1e3 1e3 1e3]

        % Angular damping about the joint axes (N*m*s/rad)
        AngularDamping(1, 3) double {mustBeFinite, mustBeNonnegative} = [1e2 1e2 1e2]

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
            phx.BushingJoint.updateView({obj});
        end

        function set.LinearStiffness(obj, value)
            obj.LinearStiffness = value;
            obj.pushSprings;
        end

        function set.LinearDamping(obj, value)
            obj.LinearDamping = value;
            obj.pushSprings;
        end

        function set.AngularStiffness(obj, value)
            obj.AngularStiffness = value;
            obj.pushSprings;
        end

        function set.AngularDamping(obj, value)
            obj.AngularDamping = value;
            obj.pushSprings;
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
        function pushSprings(obj)
        %pushSprings Writes the stiffness and damping through to the engine.
            if ~isempty(obj.ObjectHandle)
                phxEngine('set', obj.WorldHandle, obj.ObjectHandle, 'springs', ...
                    obj.LinearStiffness, obj.AngularStiffness, obj.LinearDamping, obj.AngularDamping);
            end
        end

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
                % which is then kept across pipeline rebuilds; the springs and
                % the limits write through from their setters at any time.
                obj.WorldHandle = world;
                obj.ObjectHandle = phxEngine('add', world, 'bushing', [obj.Parents{1}.ObjectHandle obj.Parents{2}.ObjectHandle], ...
                    obj.TransformA, obj.TransformB, obj.MutualCollisions);
                obj.pushSprings;
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
