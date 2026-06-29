classdef Camera < phx.base.Object
%phx.Camera Camera
%
%   Camera object allows you to attach camera position and target to
%   selected bodies.
%
%   phx.Camera(bodyA, bodyB) creates a Camera looking from body A to body B
%   respecting their points of origins.
%   Custom offset to both points of origin can be set using the PointA and
%   PointB properties.
%
%   phx.Camera(___, name, value, ...) creates a Camera and sets properties values
%   according to given name-value pairs.
%
%   See also phx.Trace

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

%#ok<*MCSUP> OK to access other properties in setters
%#ok<*INUSD> OK to see the full list of arguments for callbacks

    properties (Access = private)
        Video
        NextTime = 0
    end

    properties
        % Connecting point in the local space of the first body
        PointA (1, 3) double = [0 0 0]

        % Connecting point in the local space of the second body
        PointB (1, 3) double = [0 0 0]

        % PHX viewer
        Viewer = []

        % Video file name
        RecordFile (1, 1) string

        % Video frame rate based on simulation time
        RecordFPS (1, 1) double = 30
    end

    methods
        function obj = Camera(ParentA, ParentB, Options)
            arguments
                ParentA (1, 1) {mustBeA(ParentA, "phx.Body")}
                ParentB (1, 1) {mustBeA(ParentB, "phx.Body")}
                Options.?phx.Camera
            end

            % Set default values
            obj.SimulationOrder = "none";
            obj.RedrawOrder = "after";
            obj.ParentAxes = ParentA.ParentAxes;

            % Process input arguments
            obj.Parents = addChild([ParentA ParentB], obj);
            phx.internal.applyArguments(Options, obj);

            % Get default viewer or axes object
            if isempty(obj.Viewer)
                v = getappdata(gcf, "phxViewer");
                if ~isempty(v)
                    obj.Viewer = v;
                else
                    obj.Viewer = gca;
                end
            end

            % Create graphics objects
            phx.Camera.updateView({obj});
        end
    end

    methods (Access = protected)
        function valid = initObject(obj, world)
            if obj.RecordFile ~= ""
                obj.Video = VideoWriter(obj.RecordFile, "MPEG-4");
                obj.Video.FrameRate = obj.RecordFPS;
                obj.Video.open;
                obj.NextTime = 0;
            end

            valid = numel(obj.Parents) == 2 && all(cellfun(@isvalid, obj.Parents));
        end

        function destroyObject(obj) %#ok<MANU> function prototype
            if ~isempty(obj.Video)
                obj.Video.close;
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
                obj.Viewer.CameraPosition = pa;
                obj.Viewer.CameraTarget = pb;

                if ~isempty(obj.Video) && time >= obj.NextTime
                    obj.Video.writeVideo(getframe(gcf));
                    obj.NextTime = time + 1/obj.RecordFPS;
                end
            end
        end
    end

end