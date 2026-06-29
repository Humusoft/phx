classdef GenericSpring < phx.base.Joint
%phx.GenericSpring Generic spring - UNDER DEVELOPMENT
%
%   Generic spring realizes a kinematic constraint with 6 degrees of freedom.
%
%   phx.GenericSpring(bodyA, bodyB) creates a spring between two bodies A and B
%   attached to their points of origin and with rotation axes aligned to axis Z
%   of the local coordinate system of each body.
%   Custom connection points and direction vectors of rotation axes can be set 
%   using PointA, PointB, AxisA and AxisB properties.
%
%   phx.GenericSpring(___, name, value, ...) creates a spring and sets properties
%   values according to given name-value pairs.
%
%   See also phx.Spring

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
        % Connecting point in the local space of the first body
        PointA (1, 3) double = [0 0 0]

        % Connecting point in the local space of the second body
        PointB (1, 3) double = [0 0 0]

        % Linear limits
        LinearLimits(2, 3) double = [0 0 0; 0 0 0]

        % Angular limits
        AngularLimits(2, 3) double = [0 0 0; 0 0 0]
    end

    properties (Dependent)
    end

    methods
        function obj = GenericSpring(ParentA, ParentB, Options)
            arguments
                ParentA (1, 1) {mustBeA(ParentA, "phx.Body")}
                ParentB (1, 1) {mustBeA(ParentB, "phx.Body")}
                Options.?phx.GenericSpring
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
            phx.GenericSpring.updateView({obj});
        end
    end

    methods (Access = protected)
        function valid = initObject(obj, world)
            valid = numel(obj.Parents) == 2 && all(cellfun(@isvalid, obj.Parents));
            if valid
                obj.WorldHandle = world;
                obj.WorldHandle = world;
                ta = eye(4);
                ta(13:15) = obj.PointA;
                tb = eye(4);
                tb(13:15) = obj.PointB;
                obj.ObjectHandle = phx.engine.io('add', world, 'generic6dofspringconstraint', obj.Parents{1}.ObjectHandle, obj.Parents{2}.ObjectHandle, ta(:), tb(:), 'xyz');
                isOnLinear = [1 1 1];
                isOnAngular = [1 1 1]*0;
                linDamping = [1 1 1]*100;
                angDamping = [1 1 1]*100;
                linStiffness = [1 1 1]*0;
                angStiffness = [1 1 1]*1e6;
                %phx.engine.io('set', obj.WorldHandle, obj.ObjectHandle, 'linlimits', obj.LinearLimits(1, :), obj.LinearLimits(2, :));
                %phx.engine.io('set', obj.WorldHandle, obj.ObjectHandle, 'anglimits', obj.AngularLimits(1, :), obj.AngularLimits(2, :));
                phx.engine.io('set', obj.WorldHandle, obj.ObjectHandle, 'springinfo', isOnLinear, isOnAngular, linDamping, angDamping, linStiffness, angStiffness);
            end
        end

        function destroyObject(obj) %#ok<MANU> function prototype
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

                hL = obj.hL;
                hL.XData_I = [pa(1) pb(1)];
                hL.YData_I = [pa(2) pb(2)];
                hL.ZData_I = [pa(3) pb(3)];
            end
        end
    end

end