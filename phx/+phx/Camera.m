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

        % Time the camera lags behind the tracked pose in seconds
        % (0 for rigid tracking, Inf for a static camera)
        TrackingLag (1, 1) double {mustBeNonnegative} = 0

        % PHX viewer or axes the camera drives
        % Defaults to the viewer of the figure the first body is drawn in,
        % or to that body's axes
        Viewer = []

        % Video file name
        % Recording starts with the first simulation build and the file is
        % finalized when the camera is deleted
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

            % Process input arguments; a failure leaves no half-built object
            % attached to the parents
            try
                if ParentA == ParentB
                    addChild(ParentA, obj); % two same bodies would cause a warning
                    obj.Parents = {ParentA ParentB};
                else
                    obj.Parents = addChild([ParentA ParentB], obj);
                end
                phx.internal.applyArguments(Options, obj);

                % Default to the viewer (or axes) the scene is drawn in
                if isempty(obj.Viewer)
                    ax = ParentA.ParentAxes;
                    if isempty(ax)
                        error("phx:Camera:noView", ...
                            "The camera needs a Viewer, or a first body drawn in axes.");
                    end
                    v = getappdata(ancestor(ax, "figure"), "phxViewer");
                    if ~isempty(v) && isvalid(v)
                        obj.Viewer = v;
                    else
                        obj.Viewer = ax;
                    end
                end
            catch err
                obj.abandon([ParentA ParentB]);
                rethrow(err);
            end

            % Create graphics objects
            obj.Viewer.CameraPosition = phx.internal.transformPoint(obj.Parents{1}.Matrix, obj.PointA);
            phx.Camera.updateView({obj}, 0.01);
        end
    end

    methods (Access = protected)
        function valid = checkObject(obj)
            % The camera looks from one body at another, so it needs both.
            valid = numel(obj.Parents) == 2 && all(cellfun(@isvalid, obj.Parents));
        end

        function valid = initObject(obj, world)
            % Open the video once; later pipeline rebuilds keep recording
            % into the same file
            if obj.RecordFile ~= "" && isempty(obj.Video)
                obj.Video = VideoWriter(obj.RecordFile, "MPEG-4");
                obj.Video.FrameRate = obj.RecordFPS;
                obj.Video.open;
                obj.NextTime = 0;
            end

            valid = obj.checkObject;
        end

        function destroyObject(obj)
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
                tl = obj.TrackingLag;

                if tl > 0
                    w = 1 - exp(-dt/tl);
                    obj.Viewer.CameraPosition = obj.Viewer.CameraPosition*(1 - w) + pa*w;
                    obj.Viewer.CameraTarget = obj.Viewer.CameraTarget*(1 - w) + pb*w;
                else
                    obj.Viewer.CameraPosition = pa;
                    obj.Viewer.CameraTarget = pb;
                end

                if ~isempty(obj.Video) && time >= obj.NextTime
                    if isa(obj.Viewer, "phx.extra.Viewer")
                        fig = obj.Viewer.Figure;
                    else
                        fig = ancestor(obj.Viewer, "figure");
                    end
                    obj.Video.writeVideo(getframe(fig));
                    % Keep to the frame rate on average; if redraws come less
                    % often than frames, take one frame per redraw
                    obj.NextTime = max(obj.NextTime + 1/obj.RecordFPS, time);
                end
            end
        end
    end

end