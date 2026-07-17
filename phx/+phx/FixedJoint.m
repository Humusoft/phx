classdef FixedJoint < phx.base.Joint
%phx.FixedJoint Fixed joint
%
%   Fixed joint realizes a kinematic constraint with 0 degrees of freedom.
%
%   phx.FixedJoint(bodyA, bodyB) creates a joint between two bodies A and B
%   attached to their points of origin.
%   Custom connection points can be set using PointA, PointB properties.
%
%   phx.FixedJoint(___, name, value, ...) creates a joint and sets properties
%   values according to given name-value pairs.
%
%   See also phx.SphericalJoint

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

%#ok<*MCSUP> OK to access other properties in setters
%#ok<*INUSD> OK to see the full list of arguments for callbacks

    properties (Access = private)
        hL
    end

    properties
        % Transformation matrix relative to the first body
        TransformA (4, 4) double = eye(4)

        % Transformation matrix relative to the second body
        TransformB (4, 4) double = eye(4)
    end

    properties (Dependent)
        % Connecting point in the local space of the first body
        PointA (1, 3) double

        % Connecting point in the local space of the second body
        PointB (1, 3) double

        % Euler angles rotation (for z->y->x order) of the first body
        EulerAnglesA (1, 3) double

        % Euler angles rotation (for z->y->x order) of the second body
        EulerAnglesB (1, 3) double
    end

    properties (Dependent)
    end

    methods
        function obj = FixedJoint(ParentA, ParentB, Options)
            arguments
                ParentA (1, 1) {mustBeA(ParentA, "phx.Body")}
                ParentB (1, 1) {mustBeA(ParentB, "phx.Body")}
                Options.?phx.FixedJoint
            end

            % Set default values
            obj.SimulationOrder = "none";
            obj.RedrawOrder = "after";
            obj.ParentAxes = ParentA.ParentAxes;

            % Process input arguments
            obj.Parents = addChild([ParentA ParentB], obj);
            phx.internal.applyArguments(Options, obj);

            % Create graphics objects
            obj.hL = line(obj.Graphics, [NaN NaN], [NaN NaN], [NaN NaN], "Color", [0.6 0.6 0.6], "LineWidth", 1.0, "Marker", "o", "MarkerSize", 10);
            phx.FixedJoint.updateView({obj});
        end

        function set.PointA(obj, value)
            obj.TransformA(13:15) = value;
        end

        function value = get.PointA(obj)
            value = obj.TransformA(13:15)';
        end

        function set.PointB(obj, value)
            obj.TransformB(13:15) = value;
        end

        function value = get.PointB(obj)
            value = obj.TransformB(13:15)';
        end

        function set.EulerAnglesA(obj, value)
            obj.TransformA(1:3, 1:3) = phx.internal.Math.rot321(value);
        end

        function value = get.EulerAnglesA(obj)
            value = phx.internal.Math.decomp321(obj.TransformA(1:3, 1:3));
        end

        function set.EulerAnglesB(obj, value)
            obj.TransformB(1:3, 1:3) = phx.internal.Math.rot321(value);
        end

        function value = get.EulerAnglesB(obj)
            value = phx.internal.Math.decomp321(obj.TransformB(1:3, 1:3));
        end
    end

    methods (Access = protected)
        function valid = initObject(obj, world)
            valid = numel(obj.Parents) == 2 && all(cellfun(@isvalid, obj.Parents));
            if valid
                obj.WorldHandle = world;
                obj.ObjectHandle = phx.engine.io('add', world, 'fixedconstraint', obj.Parents{1}.ObjectHandle, obj.Parents{2}.ObjectHandle, obj.TransformA(:), obj.TransformB(:), ~obj.MutualCollisions);
                phx.engine.io('set', obj.WorldHandle, obj.ObjectHandle, 'error', 0.1, 0.00001);
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

                pa = phx.internal.transformPoint(obj.Parents{1}.Matrix, obj.TransformA(13:15));
                pb = phx.internal.transformPoint(obj.Parents{2}.Matrix, obj.TransformB(13:15));

                hL = obj.hL;
                hL.XData_I = [pa(1) pb(1)];
                hL.YData_I = [pa(2) pb(2)];
                hL.ZData_I = [pa(3) pb(3)];
            end
        end
    end

end