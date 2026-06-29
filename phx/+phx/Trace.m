classdef Trace < phx.base.Object
%phx.Trace Trace
%
%   Trace allows you to draw the trace of a moving object into the scene. It is
%   possible to set any color of the trace, the number of points and the 
%   position of the tracked point in the local space of the body.
%
%   phx.Trace(body) creates a tracer attached to the point of origin of the
%   given body.
%
%   phx.Trace(___, name, value, ...) creates a tracer and sets properties values
%   according to given name-value pairs.
%
%   See also phx.Logger

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

%#ok<*MCSUP> OK to access other properties in setters
%#ok<*INUSD> OK to see the full list of arguments for callbacks

    properties (Access = private)
        hL
        XYZData
    end

    properties
        % Point in the local frame of the body to be traced
        Point (1, 3) double = [0 0 0]

        % Number of points of the tracing curve
        TracePoints (1, 1) double = 100

        % Draw tracing as overlay
        Overlay (1, 1) logical = false
    end

    methods
        function obj = Trace(Parent, Options)
            arguments
                Parent (1, 1) {mustBeA(Parent, "phx.Body")}
                Options.?phx.Trace
            end

            % Set default values            
            obj.SimulationOrder = "none";
            obj.RedrawOrder = "after";
            obj.ParentAxes = Parent.ParentAxes;

            % Process input arguments
            obj.Parents = addChild(Parent, obj);
            phx.internal.applyArguments(Options, obj);

            % Create graphics objects
            obj.hL = matlab.graphics.primitive.world.LineStrip('Parent', obj.Graphics, 'LineWidth', 1.0, 'ColorBinding', 'object', 'ColorData', uint8([obj.Color 1]'*255), 'Layer', phx.internal.choose({'middle', 'front'}, obj.Overlay + 1));
        end
    end

    methods (Access = protected)
        function valid = initObject(obj, world)
            % Prepare data structures
            p = phx.internal.transformPoint(obj.Parents{1}.Matrix, obj.Point);
            obj.XYZData = single(zeros(3, obj.TracePoints) + p');
            obj.hL.VertexData = obj.XYZData;
            obj.hL.StripData = uint32([1 obj.TracePoints+1]);

            valid = all(cellfun(@isvalid, obj.Parents));
        end

        function destroyObject(obj)
        end
    end

    methods (Static, Access = protected)
        function resolveState(cellObjs, dt, time, world)
        end

        function updateView(cellObjs, dt, time, world)
            for i = 1:numel(cellObjs)
                obj = cellObjs{i};
                
                p = phx.internal.transformPoint(obj.Parents{1}.Matrix, obj.Point);
                xyzdata = circshift(obj.XYZData, [0 1]);
                xyzdata(:, 1) = p;
                obj.XYZData = xyzdata;

                obj.hL.VertexData = xyzdata;
            end
        end
    end

end