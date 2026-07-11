classdef Viewer < handle
%phx.extra.Viewer Enhanced viewer
%
%   Mouse navigation
%   - left button: pan or move selected object
%   - middle button: orbit or rotate selected object
%   - right button: context menu
%   - scroll wheel: zoom
%   - shift+scroll wheel: field of view
%   - double click: select/unselect object or activate assigned event
% 
%   Edit mode (for selected body)
%   - If you click directly on the axis of the highlighted coordinate
%     system, you can move or rotate objects relative to that axis.
%
%   Keyboard navigation (if enabled)
%   - arrow keys: camera panning
%   - WASD keys: camera movement
%
%   Functions
%   - F1: show help
%   - F2: headlight (on/off)
%   - F3: view mode (texture/axis/plain)
%   - F5: free run (start/stop)
%   - Home: default view
%   - PgUp/PgDown: cycle through basic views
%
%   See also phx.Simulation

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^
    
%#ok<*MCSUP> OK to access other properties in setters
%#ok<*INUSD> OK to see the full list of arguments for callbacks

    properties (Access = private)
        ContextMenu

        Shift = false
        PressedKeys = [0 0 0 0 0 0 0 0]
        LastInteresctionPoint = [0 0 0]
        LastHitPoint = [0 0]
        LastHitObject = []
        PreviousBodyState = ""
        NavMode = "none"
        BasicViewID = 0
        DragAxis = [0 0 1]
        FreeRunPrevTime = tic

        % Additional objects
        FreerunSim = []
        Triad = []
        CamLight = []
        SkySphere = []
        HUD = []

        % Timers and menus
        AnimationTimer = []
        FreerunTimer = []
    end

    properties (SetAccess = private, WeakHandle)
        % Viewer figure
        Figure matlab.ui.Figure

        % Viewer axes
        Axes matlab.graphics.axis.Axes
    end

    properties (SetAccess = private)
        % Currently selected body (empty if none). Selection is toggled by
        % double-clicking a body, or programmatically via the select and
        % deselect methods. While a body is selected it is temporarily set to
        % the kinematic type so it can be dragged without falling.
        SelectedBody = []
    end

    properties
        % Default camera position (where you can return using the Home key)
        DefaultCameraPosition (1, 3) double = [60 -60 15];

        % Default camera target (where you can return using the Home key)
        DefaultCameraTarget (1, 3) double = [0 0 0];

        % Default camera view angle (which you can revert using the Home key)
        DefaultCameraViewAngle (1, 1) double = 30;

        % Viewing mode
        ViewMode {mustBeMember(ViewMode, ["texture", "plain", "axis"])} = "texture"

        % Background texture file (in equirectangular projection)
        Texture (1, 1) string = ""

        % Enable headlight
        Headlight (1, 1) logical = true

        % Enable camera panning using arrow keys
        ArrowsEnable (1, 1) logical = false

        % Enable camera movement using WASD keys
        WASDEnable (1, 1) logical = false

        % Restrict mouse navigation to the active area of the parent axes
        RestrictedNavigation (1, 1) logical = false

        % Sky sphere size multiplier
        SkySphereSize = 4;

        % Free run step size
        FreeRunStep (1, 1) double = 0.05;

        % Free run speed factor (1 for real-time, Inf for maximal speed)
        FreeRunSpeed (1, 1) double = 1;

        % Callback fired whenever the selection changes (select or deselect).
        % It is invoked as fcn(viewer); read the SelectedBody property to get
        % the new selection. Intended for embedding the viewer in apps.
        SelectionChangedFcn = []
    end

    properties (Dependent)
        % Actual camera position (use this property instead of direct access
        % to the axes.CameraPosition for proper viewer behavior)
        CameraPosition (1, 3) double

        % Actual camera target (use this property instead of direct access
        % to the axes.CameraTarget for proper viewer behavior)
        CameraTarget (1, 3) double

        % Figure position
        Position (1, 4) double
    end

    methods
        function [obj, ParentAxes] = Viewer(ParentAxes, Options)
            arguments
                ParentAxes = []
                Options.?phx.extra.Viewer
            end

            if isempty(ParentAxes)
                ParentAxes = gca;
            elseif ParentAxes == "clear"
                ParentAxes = uiaxes(clf, 'Units', 'normalized', 'Position', [0.1 0.1 0.8 0.8]);
            elseif ParentAxes == "newfigure"
                ParentAxes = uiaxes(uifigure, 'Units', 'normalized', 'Position', [0.1 0.1 0.8 0.8]);
            end

            obj.Axes = ParentAxes;

            % Find the very top parent (e.g. in apps)
            parentFigure = ParentAxes.Parent;
            while ~isa(parentFigure, 'matlab.ui.Figure')
                parentFigure = parentFigure.Parent;
            end
            obj.Figure = parentFigure;

            % Store viewer object
            delete(getappdata(obj.Figure, 'phxViewer'));
            setappdata(obj.Figure, 'phxViewer', obj);
            setappdata(obj.Axes, 'phxAxes', true); 

            % Assign navigation callbacks
            obj.Figure.WindowButtonDownFcn = @obj.navBtnDown;
            obj.Figure.WindowButtonUpFcn = @obj.navBtnUp;
            obj.Figure.WindowButtonMotionFcn = [];
            obj.Figure.WindowKeyPressFcn = @obj.navKeyPress;
            obj.Figure.WindowKeyReleaseFcn = @obj.navKeyRelease;
            obj.Figure.WindowScrollWheelFcn = @obj.navScrollWheel;

            % Get "default default" camera settings
            if ~isempty(obj.Axes.Children)
                obj.DefaultCameraTarget = obj.CameraTarget;
                obj.DefaultCameraPosition = obj.CameraPosition;
                obj.DefaultCameraViewAngle = obj.Axes.CameraViewAngle_I;
            end

            % Set default 3D view settings
            obj.Axes.Interactions = [];
            obj.Axes.Toolbar = []; %obj.hA.Toolbar.Visible = "off";
            obj.Axes.Visible = false;
            obj.Axes.NextPlot = 'add';
            obj.Axes.Clipping = false;
            obj.Axes.DataAspectRatio = [1 1 1]; %axis(obj.hA, "equal");
            obj.Axes.XGrid = true;
            obj.Axes.YGrid = true;
            obj.Axes.ZGrid = true;
            obj.Axes.XLimitMethod = 'tight';
            obj.Axes.YLimitMethod = 'tight';
            obj.Axes.ZLimitMethod = 'tight';
            obj.Axes.Projection = 'perspective';

            % Prepare context menu
            obj.ContextMenu = uicontextmenu(obj.Figure);
            uimenu(obj.ContextMenu, 'Text', 'Look At', 'MenuSelectedFcn', @obj.menuLookAt);
            sub = uimenu(obj.ContextMenu, 'Text', 'View');
                uimenu(sub, 'Text', 'Take snapshot', 'MenuSelectedFcn', @obj.menuSnapshot);
                uimenu(sub, 'Text', 'Show camera coordinates...', 'MenuSelectedFcn', @obj.menuCamera);
            uimenu(obj.ContextMenu, 'Text', 'Material...', 'Separator', 'on', 'MenuSelectedFcn', @obj.menuChangeColor);
            uimenu(obj.ContextMenu, 'Text', 'Properites...', 'MenuSelectedFcn', @obj.menuProps);
            uimenu(obj.ContextMenu, 'Text', 'Info...', 'MenuSelectedFcn', @obj.menuInfo);
            uimenu(obj.ContextMenu, 'Text', 'Expose in workspace', 'Separator', 'on', 'MenuSelectedFcn', @obj.menuExpose);
            sub = uimenu(obj.ContextMenu, 'Text', 'Additional graphics');
                uimenu(sub, 'Text', 'Center of gravity', 'MenuSelectedFcn', @obj.menuAdditionalCenter);
                uimenu(sub, 'Text', 'Coordinate system', 'MenuSelectedFcn', @obj.menuAdditionalCoordSys);
                uimenu(sub, 'Text', 'Convex hull', 'MenuSelectedFcn', @obj.menuAdditionalConvHull);
            sub = uimenu(obj.ContextMenu, 'Text', 'Scene', 'Separator', 'on');
                uimenu(sub, 'Text', 'Clear', 'Separator', 'on', 'MenuSelectedFcn', @(~, ~) obj.cla);
                uimenu(sub, 'Text', 'Load model...', 'MenuSelectedFcn', @obj.menuLoad);
                uimenu(sub, 'Text', 'Save model...', 'MenuSelectedFcn', @obj.menuSave);
                uimenu(sub, 'Text', 'Change environment...', 'Separator', 'on', 'MenuSelectedFcn', @obj.menuChangeTexture);

            % Sky sphere
            obj.SkySphere = hgtransform(obj.Axes, 'Tag', 'phxViewer');
            obj.SkySphere.Matrix([1 6 11]) = norm(obj.CameraPosition)*obj.SkySphereSize;
            [V, N, F, T] = phx.internal.Geometry.sphere(2, 48);
            vertices = single(V');
            normals = single(N');
            faces = uint32(F');
            uvs = single(T');
            tx = matlab.graphics.primitive.world.Texture('SamplingFilter', 'bilinear');
            matlab.graphics.primitive.world.TriangleStrip('Parent', obj.SkySphere, 'VertexIndices', faces(:)', 'VertexData', vertices, 'NormalData', normals, 'ColorData', uvs, 'ColorType', 'texturemapped', 'ColorBinding', 'interpolated', 'Texture', tx, 'NormalBinding', 'interpolated', 'AmbientStrength', 1, 'DiffuseStrength', 1, 'SpecularStrength', 0, 'HitTest', 'on', 'Layer', 'back');

            % Triad
            obj.Triad = hgtransform(obj.Axes, 'Visible', false, 'Tag', 'phxViewer');
            matlab.graphics.primitive.world.LineStrip('Parent', obj.Triad, 'ColorData', uint8([255 255 255 255]'), 'ColorBinding', 'object', 'LineWidth', 4, 'VertexData', single([0 1 0 0 0 0; 0 0 0 1 0 0; 0 0 0 0 0 1]), 'Layer', 'front');
            matlab.graphics.primitive.world.LineStrip('Parent', obj.Triad, 'ColorData', uint8([255 0 0 255]'), 'ColorBinding', 'object', 'LineWidth', 2, 'VertexData', single([0.02 0.98; 0 0; 0 0]), 'Layer', 'front');
            matlab.graphics.primitive.world.LineStrip('Parent', obj.Triad, 'ColorData', uint8([0 204 0 255]'), 'ColorBinding', 'object', 'LineWidth', 2, 'VertexData', single([0 0; 0.02 0.98; 0 0]), 'Layer', 'front');
            matlab.graphics.primitive.world.LineStrip('Parent', obj.Triad, 'ColorData', uint8([0 0 255 255]'), 'ColorBinding', 'object', 'LineWidth', 2, 'VertexData', single([0 0; 0 0; 0.02 0.98]), 'Layer', 'front');

            % Timers
            obj.AnimationTimer = timer('Period', 0.05, 'ExecutionMode', 'fixedSpacing', 'TimerFcn', @obj.animatedNavigation);
            obj.FreerunTimer = timer('Period', 0.01, 'ExecutionMode', 'fixedSpacing', 'TimerFcn', @obj.freeRun);

            % Set input name/value pairs
            phx.internal.applyArguments(Options, obj);

            % Axes delete callback
            obj.Axes.DeleteFcn = @obj.axesDeleted;

            % Tweaks
            obj.Axes.Camera.DepthSort = 'on'; % can be off when there are only lowlevel objects in the scene

            % Apply initial view settings
            obj.ViewMode = obj.ViewMode;
            if obj.Texture == ""
                obj.Texture = strrep(mfilename("fullpath"), mfilename, "defaultSky.jpg");
            end
            %obj.hA.CameraViewAngle = obj.DefaultCameraViewAngle;
            obj.Headlight = obj.Headlight;
            obj.basicView("home");
        end

        function close(objs)
            arguments
                objs (1, :) phx.extra.Viewer
            end

            for obj = objs
                close(obj.hF);
                delete(obj);
            end
        end

        function select(obj, body)
        %select Selects a body for interactive editing.
        %
        %   select(viewer, body) selects the given phx.Body. Any previously
        %   selected body is deselected first. The selected body is shown with
        %   an editing gizmo and is temporarily switched to the kinematic type
        %   so it can be dragged or rotated without falling. Use deselect to
        %   restore its original type.
        %
        % See also phx.extra.Viewer.deselect, phx.extra.Viewer.SelectedBody

            arguments
                obj
                body (1, 1) phx.Body
            end

            if ~isempty(obj.SelectedBody)
                obj.deselect;
            end
            obj.SelectedBody = body;
            obj.PreviousBodyState = body.Type;
            body.Type = "kinematic";
            body.overlay("edit", "on");
            obj.notifySelection;
        end

        function basicView(obj, view)
        %basicView Sets the camera to a named or indexed standard view.
        %
        %   basicView(viewer, "home") restores the default (home) view.
        %   basicView(viewer, "next") / basicView(viewer, "previous") cycle
        %   through the basic views (front/side/back/top/bottom).
        %
        % See also phx.extra.Viewer

            if ~isnumeric(view)
                switch lower(string(view))
                    case "home"
                        obj.BasicViewID = 0;
                    case "next"
                        if obj.BasicViewID < 6
                            obj.BasicViewID = obj.BasicViewID + 1;
                        else
                            obj.BasicViewID = 1;
                        end
                    case "previous"
                        if obj.BasicViewID > 1
                            obj.BasicViewID = obj.BasicViewID - 1;
                        else
                            obj.BasicViewID = 6;
                        end
                    otherwise
                        return
                end
            end

            len = norm(obj.CameraPosition - obj.CameraTarget);
            switch obj.BasicViewID
                case 0
                    obj.CameraTarget = obj.DefaultCameraTarget;
                    obj.CameraPosition = obj.DefaultCameraPosition;
                    obj.Axes.CameraUpVector = [0 0 1];
                    obj.Axes.CameraViewAngle = obj.DefaultCameraViewAngle;
                case 1
                    obj.CameraPosition = obj.CameraTarget + [0 -len 0];
                    obj.Axes.CameraUpVector = [0 0 1];
                case 2
                    obj.CameraPosition = obj.CameraTarget + [len 0 0];
                    obj.Axes.CameraUpVector = [0 0 1];
                case 3
                    obj.CameraPosition = obj.CameraTarget + [0 len 0];
                    obj.Axes.CameraUpVector = [0 0 1];
                case 4
                    obj.CameraPosition = obj.CameraTarget + [-len 0 0];
                    obj.Axes.CameraUpVector = [0 0 1];
                case 5
                    obj.CameraPosition = obj.CameraTarget + [0 0 len];
                    obj.Axes.CameraUpVector = [0 1 0];
                case 6
                    obj.CameraPosition = obj.CameraTarget + [0 0 -len];
                    obj.Axes.CameraUpVector = [0 1 0];
            end
        end

        function deselect(obj)
        %deselect Deselects the currently selected body.
        %
        %   deselect(viewer) removes the editing gizmo and restores the
        %   original type of the currently selected body. Does nothing if no
        %   body is selected.
        %
        % See also phx.extra.Viewer.select, phx.extra.Viewer.SelectedBody

            if isempty(obj.SelectedBody)
                return
            end
            if isvalid(obj.SelectedBody)
                obj.SelectedBody.overlay("edit", "off");
                obj.SelectedBody.Type = obj.PreviousBodyState;
            end
            obj.SelectedBody = [];
            obj.notifySelection;
        end

        function set.ViewMode(obj, mode)
            obj.ViewMode = mode;
            switch mode
                case "plain"
                    obj.Axes.Visible = false;
                    obj.SkySphere.Visible = false;
                    set(obj.Axes, "XLimMode", "manual", "YLimMode", "manual", "ZLimMode", "manual");
                case "axis"
                    obj.Axes.Visible = true;
                    obj.SkySphere.Visible = false;
                    set(obj.Axes, "XLimMode", "auto", "YLimMode", "auto", "ZLimMode", "auto");
                case "texture"
                    obj.Axes.Visible = false;
                    obj.SkySphere.Visible = true;
                    set(obj.Axes, "XLimMode", "manual", "YLimMode", "manual", "ZLimMode", "manual");
            end
            
            unknownLimits = true;
            if obj.ViewMode == "axis"
                bodies = phx.Simulation.findBodies(obj.Axes);
                if ~isempty(bodies)
                    bbox = bodies.boundingBox;
                    obj.Axes.XLim = bbox(1:2);
                    obj.Axes.YLim = bbox(3:4);
                    obj.Axes.ZLim = bbox(5:6);
                    unknownLimits = false;
                end
            end

            if unknownLimits
                r = norm(obj.CameraPosition)*obj.SkySphereSize;
                obj.Axes.XLim = [-1 1]*r;
                obj.Axes.YLim = [-1 1]*r;
                obj.Axes.ZLim = [-1 1]*r;
            end
        end

        function set.Texture(obj, fileName)
            % Resolve texture file
            if ~isfile(fileName)
                % Redirect to default textures
                testName = strrep(mfilename("fullpath"), mfilename, fileName)+".*";
                defTexture = dir(testName);
                if ~isempty(defTexture)
                    fileName = fullfile(defTexture(1).folder, defTexture(1).name);
                else
                    error("phx:Viewer:fileNotFound", "File '%s' is not an existing file or name of a default texture.", fileName);
                end
            end

            % Read image
            texture = imread(fileName);

            % Add dither to prevent banding (except of small images used as
            % a solid color sample)
            imres = size(texture);
            if imres(1) > 128
                tr = rng;
                noise = randi(7, [128 128 imres(3)], 'uint8') - 4;
                noise = repmat(noise, ceil(imres./size(noise)));
                texture = texture + noise(1:imres(1), 1:imres(2), :);
                rng(tr);
            end

            % Apply texture
            texture(:, :, 4) = uint8(255);
            texture = permute(texture, [3 2 1]);
            obj.SkySphere.Children(1).Texture.CData = texture;
            obj.Texture = fileName;
        end

        function set.Headlight(obj, enable)
            % Check the headlight object
            if isempty(obj.CamLight) || ~isvalid(obj.CamLight)
                obj.CamLight = light(obj.Axes, 'Style', 'local', 'Tag', 'phxViewer');
                %obj.CamLight = matlab.graphics.primitive.world.LightSource('Parent', obj.hA, 'Style', 'local');
            end

            obj.Headlight = enable;
            obj.CamLight.Visible = enable;
        end

        function pos = get.CameraPosition(obj)
            pos = obj.Axes.CameraPosition_I;
        end

        function set.CameraPosition(obj, pos)
            obj.Axes.CameraPosition = pos;
            obj.CamLight.Position = pos;

            % Update sky sphere scale and axes limits
            r = norm(pos)*obj.SkySphereSize;
            obj.SkySphere.Matrix([1 6 11]) = [r r r];
        end

        function pos = get.CameraTarget(obj)
            pos = obj.Axes.CameraTarget_I;
        end

        function set.CameraTarget(obj, pos)
            %disp("Target"+mat2str(pos));
            obj.Axes.CameraTarget = pos;

            % Update triad scale and position
            u = obj.Axes.CameraTarget_I - obj.Axes.CameraPosition_I;
            scl = [1 1 1]*0.05*norm(u);
            obj.Triad.Matrix([1 6 11 13 14 15]) = [scl pos];
        end

        function pos = get.Position(obj)
            pos = obj.Figure.Position;
        end

        function set.Position(obj, pos)
            obj.Figure.Position = pos;
        end

        function cla(obj)
            n = numel(obj.Axes.Children);
            id = true(1, n);
            for i = 1:n
                id(i) = obj.Axes.Children(i).Tag ~= "phxViewer";
            end
            delete(obj.Axes.Children(id));
            obj.LastHitObject = [];
            obj.SelectedBody = [];
        end

        function delete(obj)
            if isvalid(obj.Figure)
                % Remove mouse callbacks
                obj.Figure.WindowButtonDownFcn = [];
                obj.Figure.WindowButtonUpFcn = [];
                obj.Figure.WindowButtonMotionFcn = [];
                obj.Figure.WindowScrollWheelFcn = [];

                % Remove keyboard callbacks
                obj.Figure.WindowKeyPressFcn = [];
                obj.Figure.WindowKeyReleaseFcn = [];
                obj.Figure.KeyPressFcn = [];
                obj.Figure.KeyReleaseFcn = [];

                % Delete internal objects
                delete(obj.SkySphere);
                delete(obj.CamLight);
                delete(obj.Triad);
                delete(obj.AnimationTimer);
                delete(obj.FreerunTimer);
            end
        end

        function displayText(obj, text, mode, maxLines, fontSize, fontColor)
        %displayText Shows a text overlay (HUD) in the corner of the viewer.
        %
        %   displayText(viewer, text) shows the given text. The text can be
        %   a single string or a column vector of strings, one element per
        %   line.
        %
        %   displayText(viewer, "") removes the overlay.
        %
        %   displayText(viewer, text, mode) controls how the text is combined
        %   with the current content:
        %   - "replace" (default) overwrites the overlay,
        %   - "replacelast" overwrites only the last line,
        %   - "below" appends the lines under the current content (scrolling
        %     up, like a log),
        %   - "above" inserts the lines above the current content.
        %
        %   displayText(viewer, text, mode, maxLines) keeps at most maxLines
        %   lines in the "below" and "above" modes (default 10); older lines
        %   are dropped.
        %
        %   displayText(___, fontSize, fontColor) sets the font size (default
        %   16) and color (default [1 1 1]). These apply when the label is
        %   first created.
        %
        % See also phx.Logger, phx.extra.Viewer

            arguments
                obj
                text (:, 1) string
                mode {mustBeMember(mode, ["replace", "replacelast", "below", "above"])} = "replace"
                maxLines (1, 1) double = 10
                fontSize = 16
                fontColor = [1 1 1]
            end
            
            % Create uilabel object if not exist
            if isempty(obj.HUD)
                drawnow;
                if obj.Axes.Parent == obj.Figure
                    posInFig = [20 20 1 1];
                else
                    posInFig = getpixelposition(obj.Axes, true);
                    posInFig(3:4) = [1 1];
                end
                obj.HUD = uilabel(obj.Figure, "Text", "", "Position", posInFig, "FontSize", fontSize, "FontName", "Consolas", "FontColor", fontColor, "VerticalAlignment", "bottom");
            end

            % Update text
            switch mode
                case "replace"
                    if text == ""
                        delete(obj.HUD);
                        obj.HUD = [];
                        return
                    end
                case "replacelast"
                    text = vertcat(obj.HUD.Text(1:end-1, :), text);
                case "below"
                    text = vertcat(obj.HUD.Text, text);
                    text = text(max(end - maxLines + 1, 1):end);
                case "above"
                    text = vertcat(text, obj.HUD.Text);
                    text = text(1:min(maxLines, end));
            end

            % Update position
            obj.HUD.Text = text;
            r = numel(text);
            c = max(strlength(text));
            obj.HUD.Position(3:4) = [c*0.6 r*1.25]*fontSize;
        end
    end

    methods (Access = private)
        function navBtnDown(obj, source, event)
            % Navigate only when cursor is inside axes region
            if obj.RestrictedNavigation
                cp = event.Point;
                ar = getpixelposition(obj.Axes, true);
                ar(3:4) = ar(3:4) + ar(1:2);
                if cp(1) < ar(1) || cp(1) > ar(3) || cp(2) < ar(2) || cp(2) > ar(4)
                    return
                end
            end

            % Store hit data
            obj.LastInteresctionPoint = event.IntersectionPoint;
            obj.LastHitPoint = event.Point;
            obj.LastHitObject = event.HitObject;

            % Find current phx.Body object
            if isa(event.HitObject.Parent, 'matlab.graphics.primitive.Transform')
                obj.DragAxis = [];
                if event.HitObject.Parent.Tag == "phx_edit"
                    body = getappdata(event.HitObject.Parent.Parent, 'phxObject');
                    a = event.HitObject.VertexData(:, 2)';
                    obj.DragAxis = a/max(a);
                else
                    body = getappdata(event.HitObject.Parent, 'phxObject');
                end
            else
                body = [];
            end

            % Perform actions by a mouse button
            switch obj.Figure.SelectionType
                case 'open'
                    % double click
                    if ~isempty(body)
                        if isempty(body.OnDoubleClickFcn)
                            if ~isempty(obj.SelectedBody) && isequal(obj.SelectedBody, body)
                                obj.deselect;
                            else
                                obj.select(body);
                            end
                        else
                            switch class(body.OnDoubleClickFcn)
                                case "function_handle"
                                    body.OnDoubleClickFcn(body, event);
                                case "string"
                                    eval(body.OnDoubleClickFcn);
                            end
                        end
                    end
                case 'normal'
                    % left button
                    if isempty(obj.SelectedBody) || ~isequal(obj.SelectedBody, body)
                        obj.NavMode = "pan";
                    else
                        obj.NavMode = "movebody";
                    end
                    obj.Figure.WindowButtonMotionFcn = @obj.navBtnMotion;
                case 'alt'
                    % right button
                    obj.ContextMenu.open(event.Point(1), event.Point(2));
                case 'extend'
                    % middle button (or left+shift)
                    if isempty(obj.SelectedBody) || ~isequal(obj.SelectedBody, body)
                        obj.NavMode = "orbit";
                    else
                        obj.NavMode = "rotatebody";
                    end
                    obj.Figure.WindowButtonMotionFcn = @obj.navBtnMotion;
            end

            obj.Triad.Matrix(13:15) = obj.CameraPosition; % put triad at invisible place (so it appears only during movement)
            obj.Triad.Visible = "on";
        end

        function navBtnUp(obj, source, event)
            obj.NavMode = "none";
            obj.Figure.WindowButtonMotionFcn = [];
            obj.Triad.Visible = "off";
        end

        function navBtnMotion(obj, source, event)
            dp = obj.LastHitPoint - event.Point;
            switch obj.NavMode
                case "pan"
                    dp = dp*0.002;
                    v = obj.CameraTarget - obj.CameraPosition;
                    xa = cross(obj.Axes.CameraUpVector, v);
                    M = makehgtform("zrotate", dp(1), "axisrotate", xa, dp(2));
                    obj.CameraTarget = obj.CameraPosition + v*M(1:3, 1:3);
                case "orbit"
                    dp = dp*0.004;
                    if obj.LastHitObject == obj.SkySphere.Children(1) || any(isnan(obj.LastInteresctionPoint))
                        C = [0 0 0];
                    else
                        C = obj.LastInteresctionPoint;
                    end
                    t = obj.CameraTarget;
                    p = obj.CameraPosition;
                    xa = cross(obj.Axes.CameraUpVector, (t - p));
                    M = makehgtform("zrotate", -dp(1), "axisrotate", xa, -dp(2));
                    obj.CameraTarget = C + (t - C)*M(1:3, 1:3);
                    obj.CameraPosition = C + (p - C)*M(1:3, 1:3);
                case "movebody"
                    len = norm(obj.CameraPosition - obj.LastInteresctionPoint)*0.01;
                    m = view(obj.Axes);
                    m = m(1:3, 1:3);
                    dp3 = [dp(1) 0 dp(2)];
                    if isempty(obj.DragAxis)
                        vec = len*dp3*m.*[0.1 0.1 0];
                    else
                        vec = len*dp3*m.*[0.1 0.1 -0.1];
                        vec = vec*obj.SelectedBody.Orientation;
                        vec = vec.*obj.DragAxis;
                        vec = vec*obj.SelectedBody.Orientation';
                    end
                    obj.SelectedBody.Position = obj.SelectedBody.Position - vec;
                case "rotatebody"
                    if isempty(obj.DragAxis)
                        a = [0 0 1];
                    else
                        a = obj.DragAxis;
                    end
                    m = makehgtform("axisrotate", a, -dp(1)*0.01);
                    obj.SelectedBody.Orientation = obj.SelectedBody.Orientation*m(1:3, 1:3);
            end
            %obj.hA.CameraUpVector = [0 0 1];
            %obj.CamLight.Position = obj.CameraPosition;
            obj.LastHitPoint = event.Point;
        end

        function navScrollWheel(obj, source, event)
            if obj.Shift
                if event.VerticalScrollCount > 0
                    obj.Axes.CameraViewAngle = min(obj.Axes.CameraViewAngle + 5, 90);
                else
                    obj.Axes.CameraViewAngle = max(obj.Axes.CameraViewAngle - 5, 5);
                end
            else
                p = obj.CameraPosition;
                t = obj.CameraTarget;
                %t = (obj.hA.CameraTarget + obj.LastInteresctionPoint)/2;
                %obj.hA.CameraTarget = t;
                dp = p - t;
                if any(isnan(obj.LastInteresctionPoint))
                    do = [0 0 0] - t;
                else
                    % cp = obj.hF.CurrentPoint;
                    % do = matlab.graphics.interaction.internal.calculateIntersectionPoint(cp, obj.hA) - t;
                    do = obj.LastInteresctionPoint - t;
                end

                if event.VerticalScrollCount > 0
                    obj.CameraPosition = p + dp*0.1 - do*0.1;
                    obj.CameraTarget = t - do*0.1;
                else
                    obj.CameraPosition = p - dp*0.1 + do*0.1;
                    obj.CameraTarget = t + do*0.1;
                end
                obj.Axes.CameraUpVector = [0 0 1];
            end
        end

        function navKeyPress(obj, source, event)
            switch event.Key
                case 'leftarrow'
                    obj.PressedKeys([1 2]) = [1 0];
                case 'rightarrow'
                    obj.PressedKeys([1 2]) = [0 1];
                case 'uparrow'
                    obj.PressedKeys([3 4]) = [1 0];
                case 'downarrow'
                    obj.PressedKeys([3 4]) = [0 1];
                case 'w'
                    obj.PressedKeys([5 6]) = [1 0];
                case 's'
                    obj.PressedKeys([5 6]) = [0 1];
                case 'a'
                    obj.PressedKeys([7 8]) = [1 0];
                case 'd'
                    obj.PressedKeys([7 8]) = [0 1];
                case 'shift'
                    obj.Shift = true;
                case 'f1'
                    uialert(obj.Figure, evalc("help phx.extra.Viewer"), "Help", "Icon", "info", "Interpreter", "html");
                case 'f2'
                    obj.Headlight = ~obj.Headlight;
                case 'f3'
                    switch obj.ViewMode
                        case "texture"
                            obj.ViewMode = "axis";
                        case "axis"
                            obj.ViewMode = "plain";
                        case "plain"
                            obj.ViewMode = "texture";
                    end
                case 'f5'
                    if strcmp(obj.FreerunTimer.Running, 'off')
                        obj.FreerunSim = phx.Simulation(obj.Axes);
                        obj.FreerunTimer.start;
                    else
                        obj.FreerunTimer.stop;
                        delete(obj.FreerunSim);
                        obj.displayText("");
                    end
                case 'home'
                    obj.basicView("home");
                case 'pageup'
                    obj.basicView("next");
                case 'pagedown'
                    obj.basicView("previous");
                otherwise
                    return
            end

            if any(obj.PressedKeys) && strcmp(obj.AnimationTimer.Running, 'off')
                obj.AnimationTimer.start;
            end
        end

        function navKeyRelease(obj, source, event)
            switch event.Key
               case 'leftarrow'
                    obj.PressedKeys(1) = 0;
                case 'rightarrow'
                    obj.PressedKeys(2) = 0;
                case 'uparrow'
                    obj.PressedKeys(3) = 0;
                case 'downarrow'
                    obj.PressedKeys(4) = 0;
                case 'w'
                    obj.PressedKeys(5) = 0;
                case 's'
                    obj.PressedKeys(6) = 0;
                case 'a'
                    obj.PressedKeys(7) = 0;
                case 'd'
                    obj.PressedKeys(8) = 0;
                case 'shift'
                    obj.Shift = false;
            end
        end

        function animatedNavigation(obj, ~, ~)
            if ~any(obj.PressedKeys)
                obj.AnimationTimer.stop;
                return
            end

            oz = obj.PressedKeys(1) - obj.PressedKeys(2);
            oy = obj.PressedKeys(3) - obj.PressedKeys(4);
            mf = obj.PressedKeys(5) - obj.PressedKeys(6);
            ms = obj.PressedKeys(7) - obj.PressedKeys(8);
            if obj.ArrowsEnable && (oz ~= 0 || oy ~= 0)
                campan(-oz*2, oy*2);
            end
            if obj.WASDEnable && (mf ~= 0 || ms ~= 0)
                d = (obj.CameraTarget - obj.CameraPosition);
                dpf = d*mf*0.02;
                dps = cross(obj.Axes.CameraUpVector, d)*ms*0.02;
                obj.CameraTarget = obj.CameraTarget+dpf+dps;
                obj.CameraPosition = obj.CameraPosition+dpf+dps;
            end
        end

        function freeRun(obj, ~, ~)
            stepSize = obj.FreeRunStep;
            obj.FreerunSim.step(stepSize, round(stepSize*100));
            obj.displayText(sprintf("▶ Free run: %0.2f", obj.FreerunSim.Time));

            dt = toc(obj.FreeRunPrevTime);
            obj.FreeRunPrevTime = tic;

            per = max(stepSize - dt*obj.FreeRunSpeed, 0);

            pause(per);
        end

        function menuLookAt(obj, source, event)
            p1 = obj.CameraTarget;
            p2 = obj.LastInteresctionPoint;
            for i = 0:0.05:1
                obj.Axes.CameraTarget = p1 + (p2 - p1)*i;
                obj.Axes.CameraUpVector = [0 0 1];
                drawnow;
            end
            obj.Axes.CameraTarget = obj.LastInteresctionPoint;
        end

        function menuSnapshot(obj, source, event)
            copygraphics(obj.Axes);
            uialert(obj.Figure, "The snapshot has been copied to the clipboard.", source.Text, "Icon", "success");
        end

        function menuCamera(obj, source, event)
            txt = ["Camera target: "+mat2str(round(obj.CameraTarget, 2)); "Camera position: "+mat2str(round(obj.CameraPosition, 2))];
            uialert(obj.Figure, txt, source.Text, "Icon", "info");
        end

        function menuChangeColor(obj, source, event)
            ph = phx.internal.PrimitiveHelper(obj.LastHitObject);
            app = phx.internal.MaterialApp(ph);
            uiwait(app.UIFigure);
        end

        function menuChangeTexture(obj, source, event)
            [file, path] = uigetfile("*.jpg;*.jpeg;*.png");
            if ischar(file)
                obj.Texture = fullfile(path, file);
                obj.SkySphere.Visible = true;
            end
        end

        function menuInfo(obj, source, event)
            phxObj = getappdata(obj.LastHitObject.Parent, 'phxObject');
            if ~isa(phxObj, 'phx.base.Object')
                return
            end

            txt = ["Object structure"; string(strsplit(evalc("phxObj.dispStructure"), newline))'];
            uialert(obj.Figure, txt, source.Text, "Icon", "info");
        end

        function menuProps(obj, source, event)
            phxObj = getappdata(obj.LastHitObject.Parent, "phxObject");
            if ~isempty(phxObj)
                inspect(phxObj);
            end
        end

        function menuExpose(obj, source, event)
            phxObj = getappdata(obj.LastHitObject.Parent, "phxObject");
            if ~isempty(phxObj)
                varName = matlab.lang.makeValidName(class(phxObj));
                assignin("base", varName, phxObj);
            end
        end

        function menuAdditionalCenter(obj, source, event)
            phxObj = getappdata(obj.LastHitObject.Parent, "phxObject");
            if ~isempty(phxObj)
                phxObj.overlay("center", "switch");
            end
        end

        function menuAdditionalCoordSys(obj, source, event)
            phxObj = getappdata(obj.LastHitObject.Parent, "phxObject");
            if ~isempty(phxObj)
                phxObj.overlay("system", "switch");
            end
        end

        function menuAdditionalConvHull(obj, source, event)
            phxObj = getappdata(obj.LastHitObject.Parent, "phxObject");
            if ~isempty(phxObj)
                phxObj.overlay("hull", "switch");
            end
        end

        function menuLoad(obj, source, event)
            [file, folder] = uigetfile("*.mat", source.Text);
            if ischar(file)
                obj.cla;
                sim = phx.Simulation(fullfile(folder, file));
                sim.propagate("ParentAxes", obj.Axes);
                delete(sim);
            end
        end

        function menuSave(obj, source, event)
            [file, folder] = uiputfile("*.mat", source.Text, "model.mat");
            if ischar(file)
                bodies = phx.Simulation.findBodies(obj.Axes);
                save(fullfile(folder, file), "bodies");
            end
        end
    end

    methods (Access = private)
        function notifySelection(obj)
            if ~isempty(obj.SelectionChangedFcn)
                feval(obj.SelectionChangedFcn, obj);
            end
        end

        function axesDeleted(obj, evn, data)
            delete(obj);
        end
    end

end