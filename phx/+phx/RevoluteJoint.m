classdef RevoluteJoint < phx.base.Joint
%phx.RevoluteJoint Revolute joint
%
%   Revolute joint realizes a kinematic constraint with 1 degree of freedom
%   specified as rotations around the axis.
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
%   See also phx.SphericalJoint

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

%#ok<*MCSUP> OK to access other properties in setters
%#ok<*INUSD> OK to see the full list of arguments for callbacks

% TODO motorized joint

    properties (Access = private)
        hL
        hM
    end

    properties
        % Connecting point in the local space of the first body
        PointA (1, 3) double = [0 0 0]

        % Connecting point in the local space of the second body
        PointB (1, 3) double = [0 0 0]
        
        % Direction vector of the rotation axis in the local space of the first body
        AxisA (1, 3) double = [0 0 1]

        % Direction vector of the rotation axis in the local space of the second body
        AxisB (1, 3) double = [0 0 1]

        % Draw joint as overlay
        Overlay (1, 1) logical = false
    end

    properties (Dependent)
        % Angle in the joint axis
        Angle (1, 1) double
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

        function value = get.Angle(obj)
            objectHandle = obj.ObjectHandle;
            if ~isempty(objectHandle)
                value  = phx.engine.io('get', obj.WorldHandle, objectHandle, 'angle');
            else
                value = NaN;
            end
        end
    end

    methods (Access = protected)
        function valid = initObject(obj, world)
            valid = numel(obj.Parents) == 2 && all(cellfun(@isvalid, obj.Parents));
            if valid
                obj.WorldHandle = world;
                obj.ObjectHandle = phx.engine.io('add', world, 'hingeconstraint', obj.Parents{1}.ObjectHandle, obj.Parents{2}.ObjectHandle, ...
                    obj.PointA, obj.PointB, obj.AxisA, obj.AxisB, true);
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