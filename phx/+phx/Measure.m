classdef Measure < phx.base.Object
%phx.Measure Measure
%
%   Measure allows to measure mutual kinematic quantities between two moving
%   bodies. These quantities are intended to be read immediately during the
%   simulation.
%
%   m = phx.Measure(bodyA, bodyB) creates the measuring object.
%
%   m = phx.Measure(bodyA, bodyB, pointA, pointB) creates the mesuring object
%   and specifies measuring points of both bodies in their local spaces.
%
%   See also phx.Logger

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

%#ok<*INUSD> OK to see the full list of arguments for callbacks

    properties (Access = private)
        hL
        hM
        Ta = eye(4)
        Tb = eye(4)
        Pa = nan(1, 3)
        Pb = nan(1, 3)
        Vab
    end

    properties (Dependent)
        % Scalar distance between both measuring points
        Distance

        % Position vector between both measuring points in the global frame
        Position

        % Position vector of the second measuring point in the local frame of the first body
        PositionInA

        % Position vector of the first measuring point in the local frame of the second body
        PositionInB

        % Velocity vector between both measuring points in the global frame
        Velocity

        % Velocity vector of the second measuring point in the local frame of the first body
        VelocityInA

        % Velocity vector of the first measuring point in the local frame of the second body
        VelocityInB
    end

    properties
        % Measuring point in the local frame of the first body
        PointA (1, 3) double = [0 0 0]

        % Measuring point in the local frame of the second body
        PointB (1, 3) double = [0 0 0]

        % Draw measure as overlay
        Overlay (1, 1) logical = false
    end

    methods
        function obj = Measure(ParentA, ParentB, Options)
            arguments
                ParentA (1, 1) {mustBeA(ParentA, "phx.Body")}
                ParentB (1, 1) {mustBeA(ParentB, "phx.Body")}
                Options.?phx.Measure
            end

            % Set default values
            obj.SimulationOrder = "after";
            obj.RedrawOrder = "after";
            obj.ParentAxes = ParentA.ParentAxes;

            % Process input arguments
            obj.Parents = addChild([ParentA ParentB], obj);
            phx.internal.applyArguments(Options, obj);

            % Create graphics objects
            clr = uint8([obj.Color*255 255]');
            obj.hL = matlab.graphics.primitive.world.LineStrip('Parent', obj.Graphics, 'LineWidth', 1.0, 'LineStyle', 'dashed', 'ColorBinding', 'object', 'ColorData', clr, 'Layer', phx.internal.choose({'middle', 'front'}, obj.Overlay + 1), 'PickableParts', 'none');
            obj.hM = matlab.graphics.primitive.world.Marker('Parent', obj.Graphics, 'EdgeColorData', clr, 'Style', 'point', 'Size', 10, 'Layer', phx.internal.choose({'middle', 'front'}, obj.Overlay + 1), 'PickableParts', 'none');
            phx.Measure.resolveState({obj}, Inf);
            phx.Measure.updateView({obj});
        end

        function dist = get.Distance(obj)
            dist = norm(obj.Pb - obj.Pa);
        end

        function pos = get.Position(obj)
            pos = obj.Pb - obj.Pa;
        end

        function pos = get.PositionInA(obj)
            pos = (obj.Ta(1:3, 1:3)'*(obj.Pb - obj.Pa)')';
        end

        function pos = get.PositionInB(obj)
            pos = (obj.Tb(1:3, 1:3)'*(obj.Pa - obj.Pb)')';
        end

        function vel = get.Velocity(obj)
            vel = obj.Vab;
        end

        function vel = get.VelocityInA(obj)
            vel = (obj.Ta(1:3, 1:3)'*obj.Vab')';
        end

        function vel = get.VelocityInB(obj)
            vel = (obj.Tb(1:3, 1:3)'*-obj.Vab')';
        end
    end

    methods (Access = protected)
        function valid = initObject(obj, world)
            valid = numel(obj.Parents) == 2 && all(cellfun(@isvalid, obj.Parents));
        end

        function destroyObject(obj) %#ok<MANU> function prototype
        end
    end

    methods (Static, Access = protected)
        function resolveState(cellObjs, dt, time, world)
            for i = 1:numel(cellObjs)
                obj = cellObjs{i};
                
                Ta = obj.Parents{1}.Matrix;
                Tb = obj.Parents{2}.Matrix;
                Pa = phx.internal.transformPoint(Ta, obj.PointA);
                Pb = phx.internal.transformPoint(Tb, obj.PointB);

                obj.Ta = Ta;
                obj.Tb = Tb;

                Vab = ((Pb - Pa) - (obj.Pb - obj.Pa))/dt;
                Vab(isnan(Vab)) = 0;
                obj.Vab = Vab;
                
                obj.Pa = Pa;
                obj.Pb = Pb;
            end
        end

        function updateView(cellObjs, dt, time, world)
            for i = 1:numel(cellObjs)
                obj = cellObjs{i};
                v = single([obj.Pa' obj.Pb']);
                obj.hL.VertexData = v;
                obj.hM.VertexData = v;
            end
        end
    end

end