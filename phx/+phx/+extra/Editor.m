classdef Editor < handle
%phx.extra.Editor Interactive WYSIWYG editor for PHX scenes
%
%   The editor is a no-code sandbox for building PHX scenes. Bodies are
%   added from a palette, arranged directly in the 3D view (drag to move,
%   shift-drag to rotate, click the colored axes to constrain to one axis) and
%   tuned in the property inspector. Scenes are saved to MAT files that can be
%   simulated with phx.Simulation or used as the scene source of the PHX
%   Simulink block.
%
%   phx.extra.Editor opens an empty editor with a ground plane.
%
%   phx.extra.Editor(fileName) opens the editor and loads the given scene
%   MAT file.
%
%   editor = phx.extra.Editor(___) returns the editor object.
%
%   The Scene panel shows the full object tree of the scene; selecting a node
%   selects that object (a body also gets the 3D editing gizmo). The Inspector
%   shows the selected object's properties as a name/value table. Edit a value
%   inline (double-click the cell) or right-click a row for a type-aware quick
%   edit (toggle a logical, pick from a set, choose a color).
%
%   Mouse (inherited from phx.extra.Viewer)
%   - double click: select / deselect a body
%   - left drag:    pan the camera, or move the selected body
%   - shift drag:   orbit the camera, or rotate the selected body
%   - scroll:       zoom
%   - on a colored axis of the selected body: constrain the move/rotate
%
%   See also phx.extra.Viewer, phx.Simulation, phx.Body

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

%#ok<*INUSD> OK to see the full list of arguments for callbacks

    properties (SetAccess = private)
        % Main figure
        UIFigure

        % The embedded 3D viewer
        Viewer

        % Target axes of the viewer
        Axes

        % The running simulation (empty while editing)
        Simulation = []

        % Currently selected scene object (any phx.base.Object, or empty)
        SelectedObject = []
    end

    properties
        % Backend for the AI assistant (spike). A function handle called as
        %   [ops, reply] = AgentBackend(userText, sceneSummaryJson, toolSpec)
        % where ops is a cell array of operation structs ({op, ...}) and reply
        % is a status string. Defaults to a built-in rule-based mock; call
        % useClaudeBackend(apiKey) to route it to the Claude API instead.
        AgentBackend = []
    end

    properties (Access = private)
        % UI handles
        Tree
        Table
        InspectorMenu
        ChatLog
        ChatInput
        SendButton
        AddButton
        InsertMenu
        AddMenu
        TreeMenu

        % Descriptor (struct array) for the rows currently shown in the table
        InspectorRows = []

        % Parallel arrays mapping tree nodes to their scene objects
        NodeObjs = {}
        NodeNodes = []

        % Last row clicked in the inspector table (fallback for the menu)
        LastSelRow = 0

        % Playback timer
        PlayTimer = []

        % Per-shape counters for default names
        Counters struct = struct

        % Controls disabled while a simulation is running
        EditControls = {}

        % Toolbar buttons that change state
        Toolbar struct = struct

        % Catalog of palette shapes: label and shape cell for phx.Body
        Palette = struct( ...
            "Box",      {{"Box"}}, ...
            "Sphere",   {{"Sphere"}}, ...
            "Cylinder", {{"Cylinder"}}, ...
            "Capsule",  {{"Capsule"}}, ...
            "Cone",     {{"Cone"}}, ...
            "Globe",    {{"Globe"}}, ...
            "Rock",     {{"Rock"}})
    end

    methods
        function obj = Editor(fileName)
            arguments
                fileName (1, 1) string = ""
            end

            obj.buildUI;

            % Default AI backend is the offline mock; useClaudeBackend swaps it.
            obj.AgentBackend = @obj.mockBackend;

            % Create the embedded viewer on our axes and listen to selection.
            % RestrictedNavigation keeps mouse navigation inside the axes area,
            % which matters because the viewer only occupies part of the window.
            obj.Viewer = phx.extra.Viewer(obj.Axes, "RestrictedNavigation", true);
            obj.Viewer.SelectionChangedFcn = @(~) obj.onSelectionChanged;

            % Refresh the inspector pose after a drag finishes
            viewerBtnUp = obj.UIFigure.WindowButtonUpFcn;
            obj.UIFigure.WindowButtonUpFcn = @(s, e) obj.onButtonUp(s, e, viewerBtnUp);

            if fileName == ""
                obj.addGround;
            else
                obj.loadScene(fileName);
            end

            obj.refreshTree;
            obj.refreshInspector;
        end

        function delete(obj)
            obj.stopPlay;
            if ~isempty(obj.Viewer) && isvalid(obj.Viewer)
                delete(obj.Viewer);
            end
            if isvalid(obj.UIFigure)
                delete(obj.UIFigure);
            end
        end
    end

    % ----------------------------------------------------------------- scene
    methods (Access = private)
        function body = addBody(obj, kind, shapeCell)
            % Create a body of the given palette kind, drop it above origin and
            % select it for immediate editing. shapeCell defaults to the palette
            % default; the interactive add passes a configured shape cell.
            if nargin < 3 || isempty(shapeCell)
                shapeCell = obj.Palette.(kind);
            end
            name = obj.uniqueName(kind);
            body = phx.Body(obj.Axes, "Shape", shapeCell, "Name", name, ...
                "Position", [0 0 3], "Type", "dynamic");
            ud.designType = "dynamic";
            ud.shape = shapeCell;
            body.UserData = ud;
            obj.refreshTree;
            obj.setSelected(body);
        end

        function addBodyInteractive(obj, kind)
            % Mini-editor shown when inserting a body: edit the shape's public
            % parameters with a live preview (shape.drawTo), then create the
            % body. PHX shapes are fixed after creation, so this is where the
            % user configures geometry.
            proto = feval("phx.shape." + kind);
            [names, kinds] = obj.shapeFields(proto);
            if isempty(names)
                obj.addBody(kind);   % nothing editable — just add the default
                return
            end
            overrides = struct();   % only user-changed params get passed on
            current = proto;        % live shape rebuilt from overrides

            d = uifigure("Name", "Add " + kind, "Position", [0 0 660 400], ...
                "WindowStyle", "modal", "Resize", "off");
            obj.centerDialog(d);
            g = uigridlayout(d, [2 2], "RowHeight", {"1x", 32}, ...
                "ColumnWidth", {280, "1x"}, "Padding", 10, "RowSpacing", 8, "ColumnSpacing", 8);

            tbl = uitable(g, "ColumnName", {'Parameter', 'Value'}, ...
                "ColumnEditable", [false true], "RowName", {}, ...
                "ColumnWidth", {120, "1x"}, "CellEditCallback", @(~, e) onEdit(e));

            ax = uiaxes(g);
            ax.Layout.Row = 1;
            ax.Layout.Column = 2;

            btnRow = uigridlayout(g, [1 3], "Padding", 0, "ColumnSpacing", 6, ...
                "ColumnWidth", {"1x", 80, 80});
            btnRow.Layout.Row = 2;
            btnRow.Layout.Column = [1 2];
            uilabel(btnRow);   % spacer
            uibutton(btnRow, "Text", "Add", "ButtonPushedFcn", @(~, ~) onOK());
            uibutton(btnRow, "Text", "Cancel", "ButtonPushedFcn", @(~, ~) delete(d));
            d.CloseRequestFcn = @(~, ~) delete(d);

            fillTable();
            redraw();

            function fillTable()
                data = cell(numel(names), 2);
                for k = 1:numel(names)
                    data{k, 1} = names{k};
                    data{k, 2} = obj.formatVal(kinds(k), current.(names{k}));
                end
                tbl.Data = data;
            end

            function onEdit(e)
                row = e.Indices(1);
                desc = struct('Kind', kinds(row), 'Options', {{}});
                [val, ok] = obj.parseValue(desc, string(e.NewData));
                if ok
                    overrides.(names{row}) = val;
                    rebuild();
                end
                fillTable();   % normalize, or revert an invalid entry
                redraw();
            end

            function rebuild()
                pairs = obj.paramPairs(overrides);
                try
                    current = feval("phx.shape." + kind, pairs{:});
                catch
                    % keep the last valid shape if the new combination is invalid
                end
            end

            function redraw()
                cla(ax);
                try
                    current.drawTo(hgtransform(ax));
                catch
                    % drawing failed — leave the preview empty
                end
                axis(ax, "equal");
                view(ax, 3);
                if isempty(findobj(ax, 'Type', 'light'))
                    light(ax);
                end
            end

            function onOK()
                shapeCell = obj.buildShapeCell(kind, overrides);
                delete(d);
                obj.addBody(kind, shapeCell);
            end
        end

        function [names, kinds] = shapeFields(obj, shp)
            % Public, publicly-settable shape properties of an editable type.
            % Pure rendering/appearance internals are hidden to keep the
            % mini-editor focused on geometry.
            skip = ["Radius", "Style", "Material", "ForcePatch", ...
                "SkeletColor", "SkeletStyle", "Texture", "TextureBlend"];
            names = {};
            kinds = strings(1, 0);
            for p = metaclass(shp).PropertyList'
                if p.Hidden || p.Constant || p.Abstract
                    continue
                end
                if ismember(string(p.Name), skip)
                    continue
                end
                if ~(ischar(p.GetAccess) && strcmp(p.GetAccess, 'public'))
                    continue
                end
                if ~(ischar(p.SetAccess) && strcmp(p.SetAccess, 'public'))
                    continue
                end
                if p.Dependent && isempty(p.SetMethod)
                    continue
                end
                try
                    v = shp.(p.Name);
                catch
                    continue
                end
                k = obj.valueKind(p.Name, v);
                if k == ""
                    continue
                end
                names{end + 1} = p.Name; %#ok<AGROW>
                kinds(end + 1) = k; %#ok<AGROW>
            end
        end

        function pairs = paramPairs(~, params)
            fn = fieldnames(params);
            pairs = cell(1, 2 * numel(fn));
            for i = 1:numel(fn)
                pairs{2 * i - 1} = fn{i};
                pairs{2 * i} = params.(fn{i});
            end
        end

        function c = buildShapeCell(obj, kind, params)
            c = [{char(kind)}, obj.paramPairs(params)];
        end

        function addGround(obj)
            shapeCell = {"Box", "Size", [20 20 0.5], "Color", [0.6 0.6 0.6]};
            body = phx.Body(obj.Axes, "Name", "Ground", "Type", "static", ...
                "Position", [0 0 -0.25], "Shape", shapeCell);
            ud.designType = "static";
            ud.shape = shapeCell;
            body.UserData = ud;
        end

        function bodies = bodies(obj)
            if isempty(obj.Axes) || ~isvalid(obj.Axes)
                bodies = phx.Body.empty;
            else
                bodies = phx.Simulation.findBodies(obj.Axes);
            end
        end

        function name = uniqueName(obj, kind)
            if ~isfield(obj.Counters, kind)
                obj.Counters.(kind) = 0;
            end
            obj.Counters.(kind) = obj.Counters.(kind) + 1;
            name = kind + obj.Counters.(kind);
        end

        function deleteSelected(obj)
            o = obj.SelectedObject;
            if isempty(o) || ~isvalid(o)
                return
            end
            if isa(o, 'phx.Body') && isequal(obj.Viewer.SelectedBody, o)
                obj.Viewer.deselect;   % restores type before removal
            end
            obj.SelectedObject = [];
            delete(o);
            obj.refreshTree;
            obj.refreshInspector;
        end

        function duplicateSelected(obj)
            src = obj.SelectedObject;
            if isempty(src) || ~isa(src, 'phx.Body')
                return
            end
            if ~isstruct(src.UserData) || ~isfield(src.UserData, "shape")
                uialert(obj.UIFigure, ...
                    "This body cannot be duplicated (its shape is unknown).", "Duplicate");
                return
            end
            obj.Viewer.deselect;
            t = src.UserData.designType;
            body = phx.Body(obj.Axes, "Shape", src.UserData.shape, ...
                "Name", obj.uniqueName("Copy"), "Type", t);
            body.Transform = src.offset([1 0 0]);
            body.Mass = src.Mass;
            body.Friction = src.Friction;
            body.Color = src.Color;
            ud.designType = t;
            ud.shape = src.UserData.shape;
            body.UserData = ud;
            obj.refreshTree;
            obj.setSelected(body);
        end

        function loadScene(obj, fileName)
            obj.Viewer.deselect;
            obj.Viewer.cla;
            obj.SelectedObject = [];
            obj.Counters = struct;
            sim = phx.Simulation(string(fileName));
            sim.propagate("ParentAxes", obj.Axes);
            for b = obj.bodies
                if ~isstruct(b.UserData) || ~isfield(b.UserData, "designType")
                    b.UserData = struct("designType", b.Type);
                end
            end
            delete(sim);
            obj.Viewer.basicView("home");
        end
    end

    % ------------------------------------------------------------- playback
    methods (Access = private)
        function startPlay(obj)
            if ~isempty(obj.Simulation)
                return
            end
            obj.Viewer.deselect;
            bs = obj.bodies;
            for b = bs
                b.Type = b.UserData.designType;   % restore design types
                b.storeState("editorInitial");
            end
            obj.Simulation = phx.Simulation(obj.Axes);
            obj.setEditingEnabled(false);
            obj.PlayTimer = timer("Period", 0.03, "ExecutionMode", "fixedSpacing", ...
                "TimerFcn", @(~, ~) obj.playStep, "BusyMode", "drop");
            start(obj.PlayTimer);
            obj.Toolbar.Play.Enable = "off";
            obj.Toolbar.Pause.Enable = "on";
        end

        function playStep(obj)
            if isempty(obj.Simulation) || ~isvalid(obj.Simulation)
                obj.stopPlay;
                return
            end
            obj.Simulation.step(0.03, 6);
        end

        function pausePlay(obj)
            obj.stopTimer;
            obj.Toolbar.Play.Enable = "on";
            obj.Toolbar.Pause.Enable = "off";
        end

        function singleStep(obj)
            startedHere = isempty(obj.Simulation);
            if startedHere
                obj.startPlay;
                obj.pausePlay;
            end
            obj.Simulation.step(0.03, 6);
        end

        function stopPlay(obj)
            obj.stopTimer;
            if ~isempty(obj.Simulation) && isvalid(obj.Simulation)
                bs = obj.bodies;
                delete(obj.Simulation);      % clears engine handles
                obj.Simulation = [];
                for b = bs                   % reset to the initial layout
                    b.restoreState("editorInitial");  % also resets the graphics
                    b.clearStates;
                    b.Type = b.UserData.designType;
                end
                drawnow;
            end
            obj.Simulation = [];
            obj.setEditingEnabled(true);
            if isfield(obj.Toolbar, "Play") && isvalid(obj.Toolbar.Play)
                obj.Toolbar.Play.Enable = "on";
                obj.Toolbar.Pause.Enable = "off";
            end
            obj.refreshInspector;
        end

        function stopTimer(obj)
            if ~isempty(obj.PlayTimer) && isvalid(obj.PlayTimer)
                stop(obj.PlayTimer);
                delete(obj.PlayTimer);
            end
            obj.PlayTimer = [];
        end
    end

    % ----------------------------------------------------------------- files
    methods (Access = private)
        function fileNew(obj)
            obj.stopPlay;
            obj.Viewer.deselect;
            obj.Viewer.cla;
            obj.SelectedObject = [];
            obj.Counters = struct;
            obj.addGround;
            obj.refreshTree;
            obj.refreshInspector;
            if ~isempty(obj.ChatLog) && isvalid(obj.ChatLog)
                obj.ChatLog.Value = {'New scene. Ask me to build or edit it.'};
            end
        end

        function fileOpen(obj)
            [file, folder] = uigetfile("*.mat", "Open scene");
            if ~ischar(file)
                return
            end
            obj.stopPlay;
            obj.loadScene(fullfile(folder, file));
            obj.refreshTree;
            obj.refreshInspector;
        end

        function fileSave(obj)
            [file, folder] = uiputfile("*.mat", "Save scene", "scene.mat");
            if ~ischar(file)
                return
            end
            obj.stopPlay;
            obj.Viewer.deselect;
            bodies = obj.bodies;
            save(fullfile(folder, file), "bodies");
        end
    end

    % ------------------------------------------------------------ selection
    methods (Access = private)
        function setSelected(obj, target)
            % Central selection entry point for any scene object.
            if ~isempty(target) && isa(target, 'phx.Body')
                obj.Viewer.select(target);   % fires onSelectionChanged
            else
                obj.Viewer.deselect;         % fires onSelectionChanged (clears)
                obj.SelectedObject = target;
                obj.highlightTree;
                obj.refreshInspector;
            end
        end

        function onSelectionChanged(obj)
            % Fired by the viewer (e.g. user double-clicked a body in 3D).
            % When nothing is selected, re-assert design types in case a type
            % was changed in the inspector while the body was selected.
            if isempty(obj.Viewer.SelectedBody)
                for b = obj.bodies
                    if isstruct(b.UserData) && isfield(b.UserData, "designType")
                        b.Type = b.UserData.designType;
                    end
                end
            end
            obj.SelectedObject = obj.Viewer.SelectedBody;
            obj.highlightTree;
            obj.refreshInspector;
        end

        function onButtonUp(obj, source, event, viewerBtnUp)
            if ~isempty(viewerBtnUp)
                feval(viewerBtnUp, source, event);
            end
            if isempty(obj.Simulation)
                obj.refreshInspector;   % pose may have changed during a drag
            end
        end

        function onTreeSelect(obj)
            nodes = obj.Tree.SelectedNodes;
            if isempty(nodes)
                obj.setSelected([]);
            else
                obj.setSelected(nodes(1).NodeData);
            end
        end
    end

    % ----------------------------------------------------------------- tree
    methods (Access = private)
        function refreshTree(obj)
            if isempty(obj.Tree) || ~isvalid(obj.Tree)
                return
            end
            delete(obj.Tree.Children);
            obj.NodeObjs = {};
            obj.NodeNodes = [];
            for b = obj.bodies
                obj.addTreeNode(obj.Tree, b);
            end
            expand(obj.Tree);
            obj.highlightTree;
        end

        function addTreeNode(obj, parent, o)
            node = uitreenode(parent, "Text", obj.nodeLabel(o), "NodeData", o);
            obj.NodeObjs{end + 1} = o;
            obj.NodeNodes = [obj.NodeNodes node];
            for i = 1:numel(o.Children)
                obj.addTreeNode(node, o.Children{i});
            end
        end

        function highlightTree(obj)
            if isempty(obj.Tree) || ~isvalid(obj.Tree)
                return
            end
            o = obj.SelectedObject;
            if ~isempty(o)
                for i = 1:numel(obj.NodeObjs)
                    if isequal(obj.NodeObjs{i}, o)
                        obj.Tree.SelectedNodes = obj.NodeNodes(i);
                        return
                    end
                end
            end
            obj.Tree.SelectedNodes = [];
        end

        function s = nodeLabel(~, o)
            cls = string(class(o));
            if startsWith(cls, "phx.")
                cls = extractAfter(cls, "phx.");
            end
            if ~ismissing(o.Name) && o.Name ~= ""
                s = char(o.Name + "  (" + cls + ")");
            else
                s = char("(" + cls + ")");
            end
        end
    end

    % ------------------------------------------------------------ inspector
    methods (Access = private)
        function refreshInspector(obj)
            if isempty(obj.Table) || ~isvalid(obj.Table)
                return
            end
            o = obj.SelectedObject;
            if isempty(o) || ~isvalid(o)
                obj.InspectorRows = [];
                obj.Table.Data = {};
                return
            end
            obj.InspectorRows = obj.buildRows(o);
            n = numel(obj.InspectorRows);
            data = cell(n, 2);
            for i = 1:n
                d = obj.InspectorRows(i);
                data{i, 1} = char(d.Label);
                data{i, 2} = obj.formatVal(d.Kind, d.Get());
            end
            obj.Table.Data = data;
        end

        function rows = buildRows(obj, o)
            % Bodies get a curated layout (nice labels, degrees, design type);
            % every other class is reflected generically from its public
            % properties of editable types.
            if isa(o, 'phx.Body')
                rows = obj.bodyRows(o);
            else
                rows = obj.reflectRows(o);
            end
        end

        function rows = bodyRows(obj, o)
            % Curated descriptor for phx.Body. Each row carries its display
            % label, a kind that drives display/editing, options (for enums)
            % and getter/setter closures over the object.
            r = {};
            r{end + 1} = obj.row("Name", "text", {}, @() o.Name, @(v) obj.setName(o, v));
            r{end + 1} = obj.row("Type", "enum", {'static', 'kinematic', 'dynamic'}, ...
                @() obj.designType(o), @(v) obj.setBodyType(o, v));
            r{end + 1} = obj.row("Position", "vector", {}, ...
                @() o.Position, @(v) obj.setProp(o, 'Position', v));
            r{end + 1} = obj.row("Rotation [deg]", "vector", {}, ...
                @() rad2deg(o.EulerAngles), @(v) obj.setEuler(o, v));
            r{end + 1} = obj.row("Mass [kg]", "numeric", {}, ...
                @() o.Mass, @(v) obj.setProp(o, 'Mass', v));
            r{end + 1} = obj.row("Friction", "numeric", {}, ...
                @() o.Friction(1), @(v) obj.setFriction(o, v));
            r{end + 1} = obj.row("Collisions", "logical", {}, ...
                @() o.Collisions, @(v) obj.setProp(o, 'Collisions', v));
            r{end + 1} = obj.row("Color", "color", {}, ...
                @() o.Color, @(v) obj.setProp(o, 'Color', v));
            rows = [r{:}];
        end

        function rows = reflectRows(obj, o)
            % Generic inspector for any object without a curated layout: list
            % public, publicly-settable properties whose value is of an
            % editable type (logical / numeric / vector / text / enumeration).
            r = {};
            for p = metaclass(o).PropertyList'
                if p.Hidden || p.Constant || p.Abstract
                    continue
                end
                if ~(ischar(p.GetAccess) && strcmp(p.GetAccess, 'public'))
                    continue
                end
                if ~(ischar(p.SetAccess) && strcmp(p.SetAccess, 'public'))
                    continue
                end
                if p.Dependent && isempty(p.SetMethod)
                    continue   % read-only dependent (e.g. joint reaction forces)
                end
                name = p.Name;
                try
                    val = o.(name);
                catch
                    continue   % skip properties that error on read
                end
                [kind, options] = obj.valueKind(name, val);
                if kind == ""
                    continue   % unsupported type
                end
                r{end + 1} = obj.row(name, kind, options, ...
                    obj.makeGetter(o, name), obj.makeSetter(o, name)); %#ok<AGROW> small list
            end
            rows = [r{:}];
            % Show Name first, like the curated body layout.
            if ~isempty(rows)
                isName = [rows.Label] == "Name";
                rows = [rows(isName) rows(~isName)];
            end
        end

        function [kind, options] = valueKind(~, name, val)
            kind = "";
            options = {};
            if string(name) == "Color"
                kind = "color";
            elseif isenum(val)
                kind = "enum";
                options = cellstr(string(enumeration(class(val)))');
            elseif islogical(val) && isscalar(val)
                kind = "logical";
            elseif (ischar(val) && (isrow(val) || isempty(val))) || (isstring(val) && isscalar(val))
                kind = "text";
            elseif isnumeric(val) && isscalar(val)
                kind = "numeric";
            elseif isnumeric(val) && isvector(val) && numel(val) >= 2 && numel(val) <= 4
                kind = "vector";
            end
        end

        function f = makeGetter(~, o, name)
            f = @() o.(name);
        end

        function f = makeSetter(obj, o, name)
            if string(name) == "Name"
                f = @(v) obj.setName(o, v);
            else
                f = @(v) obj.setProp(o, char(name), v);
            end
        end

        function s = row(~, label, kind, options, getter, setter)
            s.Label = string(label);
            s.Kind = string(kind);
            s.Options = options;
            s.Get = getter;
            s.Set = setter;
        end

        % --- value formatting / parsing -----------------------------------

        function s = formatVal(~, kind, v)
            switch kind
                case "logical"
                    if v
                        s = "true";
                    else
                        s = "false";
                    end
                case "vector"
                    s = "[" + join(string(round(v, 4)), " ") + "]";
                case "numeric"
                    s = string(round(v, 6));
                case "color"
                    if numel(v) == 3 && all(~isnan(v))
                        s = "[" + join(string(round(v, 2)), " ") + "]";
                    else
                        s = "(auto)";
                    end
                otherwise
                    s = string(v);
            end
            if ismissing(s)
                s = "";
            end
            s = char(s);
        end

        function [val, ok] = parseValue(~, d, txt)
            ok = true;
            val = [];
            switch d.Kind
                case "logical"
                    s = lower(strip(txt));
                    if ismember(s, ["true", "1", "on", "yes"])
                        val = true;
                    elseif ismember(s, ["false", "0", "off", "no"])
                        val = false;
                    else
                        ok = false;
                    end
                case "enum"
                    if ismember(txt, string(d.Options))
                        val = char(txt);
                    else
                        ok = false;
                    end
                case "numeric"
                    x = str2double(txt);
                    if isnan(x)
                        ok = false;
                    else
                        val = x;
                    end
                case {"vector", "color"}
                    x = str2num(txt); %#ok<ST2NM> intentional flexible parse
                    if isempty(x) || ~isnumeric(x)
                        ok = false;
                    else
                        val = x(:)';
                    end
                otherwise
                    val = string(txt);
            end
        end

        % --- editing actions ----------------------------------------------

        function applySet(obj, row, val)
            if isempty(row) || row < 1 || row > numel(obj.InspectorRows)
                return
            end
            try
                obj.InspectorRows(row).Set(val);
            catch err
                uialert(obj.UIFigure, err.message, "Invalid value");
            end
            obj.refreshInspector;
        end

        function onCellEdit(obj, ~, event)
            row = event.Indices(1);
            if row < 1 || row > numel(obj.InspectorRows)
                return
            end
            [val, ok] = obj.parseValue(obj.InspectorRows(row), string(event.NewData));
            if ok
                obj.applySet(row, val);
            else
                obj.refreshInspector;   % revert the cell
            end
        end

        function onCellSelect(obj, ~, event)
            if ~isempty(event.Indices)
                obj.LastSelRow = event.Indices(1);
            end
        end

        function row = currentRow(obj)
            row = [];
            if ~isempty(obj.Table.Selection)
                row = obj.Table.Selection(1);
            elseif obj.LastSelRow >= 1
                row = obj.LastSelRow;
            end
            if ~isempty(row) && (row < 1 || row > numel(obj.InspectorRows))
                row = [];
            end
        end

        function buildRowMenu(obj)
            delete(obj.InspectorMenu.Children);
            row = obj.currentRow;
            if isempty(row)
                return
            end
            d = obj.InspectorRows(row);
            cm = obj.InspectorMenu;
            switch d.Kind
                case "logical"
                    uimenu(cm, "Text", "true", "MenuSelectedFcn", @(~, ~) obj.applySet(row, true));
                    uimenu(cm, "Text", "false", "MenuSelectedFcn", @(~, ~) obj.applySet(row, false));
                case "enum"
                    for o = string(d.Options)
                        uimenu(cm, "Text", char(o), "MenuSelectedFcn", @(~, ~) obj.applySet(row, char(o)));
                    end
                case "color"
                    uimenu(cm, "Text", "Choose color…", "MenuSelectedFcn", @(~, ~) obj.pickColor(row));
                otherwise
                    uimenu(cm, "Text", "Edit…", "MenuSelectedFcn", @(~, ~) obj.editFree(row));
            end
        end

        function pickColor(obj, row)
            d = obj.InspectorRows(row);
            cur = d.Get();
            if numel(cur) ~= 3 || any(isnan(cur))
                cur = [0.5 0.5 0.5];
            end
            c = uisetcolor(cur, "Color");
            if numel(c) == 3
                obj.applySet(row, c);
            end
        end

        function editFree(obj, row)
            d = obj.InspectorRows(row);
            cur = {obj.formatVal(d.Kind, d.Get())};
            answer = inputdlg(char(d.Label), "Edit value", 1, cur);
            if isempty(answer)
                return
            end
            [val, ok] = obj.parseValue(d, string(answer{1}));
            if ok
                obj.applySet(row, val);
            end
        end

        % --- property setters ---------------------------------------------

        function setProp(~, o, prop, v)
            o.(prop) = v;
        end

        function setName(obj, o, v)
            o.Name = string(v);
            obj.refreshTree;
        end

        function setEuler(~, o, v)
            o.EulerAngles = deg2rad(v);
        end

        function setFriction(~, o, v)
            o.Friction = [v o.Friction(2:3)];
        end

        function t = designType(~, o)
            if isstruct(o.UserData) && isfield(o.UserData, "designType")
                t = o.UserData.designType;
            else
                t = o.Type;
            end
        end

        function setBodyType(obj, o, v)
            v = string(v);
            if isstruct(o.UserData)
                ud = o.UserData;
            else
                ud = struct;
            end
            ud.designType = v;
            o.UserData = ud;
            % Keep the viewer's remembered "resting" type in sync if selected.
            if ~isempty(obj.Viewer.SelectedBody) && isequal(obj.Viewer.SelectedBody, o)
                obj.Viewer.deselect;
                o.Type = v;
                obj.Viewer.select(o);
            else
                o.Type = v;
            end
        end
    end

    % --------------------------------------------------------- object catalog
    methods (Access = private)
        function c = catalog(obj)
            % Single registry of creatable object types. Every "add" surface
            % (Insert menu, the +Add toolbar button, the tree context menu) is
            % generated from this list, so a new type means one entry here.
            c = struct('Category', {}, 'Label', {}, 'Create', {});
            for k = string(fieldnames(obj.Palette))'
                c(end + 1) = struct('Category', "Bodies", 'Label', k, ...
                    'Create', @() obj.addBodyInteractive(k)); %#ok<AGROW> small list
            end

            % Connectors (need two bodies — handled by addConnector)
            connectors = { ...
                "Joints",  "Fixed joint",     "FixedJoint"; ...
                "Joints",  "Revolute joint",  "RevoluteJoint"; ...
                "Joints",  "Spherical joint", "SphericalJoint"; ...
                "Springs", "Spring",          "Spring"};
            for i = 1:size(connectors, 1)
                c(end + 1) = struct('Category', connectors{i, 1}, ...
                    'Label', connectors{i, 2}, ...
                    'Create', obj.connectorFcn(connectors{i, 3}, connectors{i, 2})); %#ok<AGROW>
            end

            % Single-body attachments (force/helper elements)
            attachments = { ...
                "Forces",  "Thruster", "Thruster"; ...
                "Helpers", "Trace",    "Trace"};
            for i = 1:size(attachments, 1)
                c(end + 1) = struct('Category', attachments{i, 1}, ...
                    'Label', attachments{i, 2}, ...
                    'Create', obj.attachmentFcn(attachments{i, 3}, attachments{i, 2})); %#ok<AGROW>
            end
            % Future: Logger, Measure, ... go here.
        end

        function f = connectorFcn(obj, className, label)
            % Capture by value (avoids the closure-over-loop-variable bug).
            f = @() obj.addConnector(className, label);
        end

        function f = attachmentFcn(obj, className, label)
            f = @() obj.addAttachment(className, label);
        end

        function populateAddMenu(obj, parent)
            delete(parent.Children);
            c = obj.catalog;
            cats = unique(string({c.Category}), "stable");
            if numel(cats) <= 1
                for i = 1:numel(c)
                    uimenu(parent, "Text", char(c(i).Label), "MenuSelectedFcn", obj.addFcn(c(i)));
                end
            else
                allCats = string({c.Category});
                for cat = cats
                    sub = uimenu(parent, "Text", char(cat));
                    for i = find(allCats == cat)
                        uimenu(sub, "Text", char(c(i).Label), "MenuSelectedFcn", obj.addFcn(c(i)));
                    end
                end
            end
        end

        function f = addFcn(~, entry)
            % Capture entry by value (avoids the closure-over-loop-variable bug)
            f = @(~, ~) entry.Create();
        end

        function openAddMenu(obj)
            obj.populateAddMenu(obj.AddMenu);
            pos = getpixelposition(obj.AddButton, true);
            obj.AddMenu.open(pos(1), pos(2));
        end

        function buildTreeMenu(obj)
            delete(obj.TreeMenu.Children);
            addSub = uimenu(obj.TreeMenu, "Text", "Add");
            obj.populateAddMenu(addSub);
            hasSel = ~isempty(obj.SelectedObject) && isvalid(obj.SelectedObject);
            mDup = uimenu(obj.TreeMenu, "Text", "Duplicate", "Separator", "on", ...
                "MenuSelectedFcn", @(~, ~) obj.duplicateSelected);
            mDel = uimenu(obj.TreeMenu, "Text", "Delete", ...
                "MenuSelectedFcn", @(~, ~) obj.deleteSelected);
            mDup.Enable = matlab.lang.OnOffSwitchState(hasSel);
            mDel.Enable = matlab.lang.OnOffSwitchState(hasSel);
        end

        function addConnector(obj, className, label)
            % Connectors (joints, springs) attach to TWO bodies. Open a small
            % dialog to pick body A and B, then create the connector.
            bs = obj.bodies;
            if numel(bs) < 2
                uialert(obj.UIFigure, ...
                    "You need at least two bodies before adding a connection.", label);
                return
            end

            labels = strings(1, numel(bs));
            for i = 1:numel(bs)
                labels(i) = string(obj.nodeLabel(bs(i))) + "  #" + i;
            end
            data = num2cell(bs);

            d = uifigure("Name", label, "Position", [0 0 340 160], ...
                "WindowStyle", "modal", "Resize", "off");
            obj.centerDialog(d);
            g = uigridlayout(d, [3 2], "RowHeight", {28, 28, 32}, ...
                "ColumnWidth", {70, "1x"}, "Padding", 12, "RowSpacing", 8);

            uilabel(g, "Text", "Body A");
            ddA = uidropdown(g, "Items", cellstr(labels), "ItemsData", data);
            uilabel(g, "Text", "Body B");
            ddB = uidropdown(g, "Items", cellstr(labels), "ItemsData", data);

            % Default A to the current selection (if a body), B to a different one.
            sel = obj.SelectedObject;
            if ~isempty(sel) && isa(sel, 'phx.Body')
                for i = 1:numel(bs)
                    if isequal(bs(i), sel)
                        ddA.Value = data{i};
                        break
                    end
                end
            end
            for i = 1:numel(bs)
                if ~isequal(data{i}, ddA.Value)
                    ddB.Value = data{i};
                    break
                end
            end

            btnRow = uigridlayout(g, [1 2], "Padding", 0, "ColumnSpacing", 6);
            btnRow.Layout.Row = 3;
            btnRow.Layout.Column = [1 2];
            uibutton(btnRow, "Text", "OK", ...
                "ButtonPushedFcn", @(~, ~) obj.finishConnector(d, className, label, ddA, ddB));
            uibutton(btnRow, "Text", "Cancel", "ButtonPushedFcn", @(~, ~) delete(d));
            d.CloseRequestFcn = @(~, ~) delete(d);
        end

        function finishConnector(obj, dlg, className, label, ddA, ddB)
            A = ddA.Value;
            B = ddB.Value;
            if isequal(A, B)
                uialert(dlg, "Select two different bodies.", label);
                return
            end
            try
                conn = feval("phx." + className, A, B);
            catch err
                uialert(dlg, err.message, label);
                return
            end
            delete(dlg);
            obj.refreshTree;
            obj.setSelected(conn);
        end

        function addAttachment(obj, className, label)
            % Force/helper elements attach to ONE body. Use the selected body
            % directly; otherwise pick one from a dialog.
            bs = obj.bodies;
            if isempty(bs)
                uialert(obj.UIFigure, "Add a body first.", label);
                return
            end
            sel = obj.SelectedObject;
            if ~isempty(sel) && isa(sel, 'phx.Body')
                obj.makeAttachment(className, label, sel);
                return
            end

            labels = strings(1, numel(bs));
            for i = 1:numel(bs)
                labels(i) = string(obj.nodeLabel(bs(i))) + "  #" + i;
            end
            data = num2cell(bs);

            d = uifigure("Name", label, "Position", [0 0 320 120], ...
                "WindowStyle", "modal", "Resize", "off");
            obj.centerDialog(d);
            g = uigridlayout(d, [2 2], "RowHeight", {28, 32}, ...
                "ColumnWidth", {90, "1x"}, "Padding", 12, "RowSpacing", 8);
            uilabel(g, "Text", "Attach to");
            dd = uidropdown(g, "Items", cellstr(labels), "ItemsData", data);

            btnRow = uigridlayout(g, [1 2], "Padding", 0, "ColumnSpacing", 6);
            btnRow.Layout.Row = 2;
            btnRow.Layout.Column = [1 2];
            uibutton(btnRow, "Text", "OK", ...
                "ButtonPushedFcn", @(~, ~) obj.finishAttachment(d, className, label, dd));
            uibutton(btnRow, "Text", "Cancel", "ButtonPushedFcn", @(~, ~) delete(d));
            d.CloseRequestFcn = @(~, ~) delete(d);
        end

        function finishAttachment(obj, dlg, className, label, dd)
            body = dd.Value;
            delete(dlg);
            obj.makeAttachment(className, label, body);
        end

        function makeAttachment(obj, className, label, body)
            try
                att = feval("phx." + className, body);
            catch err
                uialert(obj.UIFigure, err.message, label);
                return
            end
            obj.refreshTree;
            obj.setSelected(att);
        end

        function centerDialog(obj, d)
            % Center a dialog over the main application window (not the screen).
            if isempty(obj.UIFigure) || ~isvalid(obj.UIFigure)
                movegui(d, "center");
                return
            end
            main = obj.UIFigure.Position;
            sz = d.Position(3:4);
            d.Position(1:2) = main(1:2) + (main(3:4) - sz) / 2;
        end
    end

    % ----------------------------------------------------- AI agent (spike)
    methods
        function useClaudeBackend(obj, apiKey, model)
        %useClaudeBackend Route the AI assistant to the Claude API.
        %
        %   useClaudeBackend(editor, apiKey) uses claude-opus-4-8.
        %   useClaudeBackend(editor, apiKey, model) overrides the model.
        %
        %   The scene is sent to the model along with a small tool set; the
        %   model's tool calls are applied to the live scene. Requires internet
        %   access and a valid Anthropic API key.
        %
        % See also phx.extra.Editor.AgentBackend

            arguments
                obj
                apiKey (1, 1) string
                model (1, 1) string = "claude-opus-4-8"
            end
            obj.AgentBackend = @(u, s, t) obj.claudeRequest(u, s, t, apiKey, model);
            obj.appendChat("[connected to Claude model " + model + "]");
        end

        function useOllamaBackend(obj, model, url)
        %useOllamaBackend Route the AI assistant to a local Ollama server.
        %
        %   useOllamaBackend(editor) uses model "llama3.1" at the default
        %   local Ollama endpoint (http://localhost:11434).
        %   useOllamaBackend(editor, model, url) overrides both.
        %
        %   Free and fully local — no API key. Requires a running Ollama server
        %   with the chosen model pulled (e.g. "ollama pull llama3.1"). Use a
        %   tool-calling-capable model for best results.
        %
        % See also phx.extra.Editor.useClaudeBackend, phx.extra.Editor.AgentBackend

            arguments
                obj
                model (1, 1) string = "llama3.1"
                url (1, 1) string = "http://localhost:11434"
            end
            obj.AgentBackend = @(u, s, t) obj.ollamaRequest(u, s, t, model, url);
            obj.appendChat("[connected to Ollama model " + model + " at " + url + "]");
        end
    end

    methods (Access = private)
        function agentSend(obj)
            txt = strtrim(string(obj.ChatInput.Value));
            if txt == ""
                return
            end
            obj.ChatInput.Value = "";
            obj.appendChat("You: " + txt);
            obj.setAgentBusy(true);
            % onCleanup restores the Send button even if the backend errors
            cleanup = onCleanup(@() obj.setAgentBusy(false));
            try
                [ops, reply] = obj.AgentBackend(txt, obj.sceneSummary, obj.toolSpec);
            catch err
                obj.appendChat("[backend error] " + string(err.message));
                return
            end
            n = obj.applyOps(ops);
            if strlength(string(reply)) > 0
                obj.appendChat("AI: " + string(reply));
            end
            obj.appendChat(sprintf("(applied %d operation(s))", n));
            obj.refreshTree;
            obj.refreshInspector;
        end

        function appendChat(obj, line)
            if isempty(obj.ChatLog) || ~isvalid(obj.ChatLog)
                return
            end
            obj.ChatLog.Value = [obj.ChatLog.Value; cellstr(string(line))];
            scroll(obj.ChatLog, "bottom");
        end

        function setAgentBusy(obj, tf)
            % Show a "thinking" state on the Send button while the (blocking)
            % backend call runs. drawnow forces the state to render first.
            if isempty(obj.SendButton) || ~isvalid(obj.SendButton)
                return
            end
            if tf
                obj.SendButton.Text = "Thinking…";
                obj.SendButton.BackgroundColor = [0.93 0.69 0.13];
                obj.SendButton.Enable = "off";
                obj.ChatInput.Enable = "off";
            else
                obj.SendButton.Text = "Send";
                obj.SendButton.BackgroundColor = [0.96 0.96 0.96];
                obj.SendButton.Enable = "on";
                if isempty(obj.Simulation)   % don't re-enable input during playback
                    obj.ChatInput.Enable = "on";
                end
            end
            drawnow;
        end

        function spec = toolSpec(obj)
            % Description of the operations the agent may perform, derived from
            % the catalog. Passed to the backend (the Claude adapter turns it
            % into tool definitions; the mock ignores it).
            spec.types = string(fieldnames(obj.Palette))';
        end

        function s = sceneSummary(obj)
            items = {};
            stack = num2cell(obj.bodies);
            while ~isempty(stack)
                cur = stack{1};
                stack(1) = [];
                it = struct();
                nm = cur.Name;
                if ismissing(nm); nm = ""; end
                it.name = char(nm);
                cls = string(class(cur));
                if startsWith(cls, "phx."); cls = extractAfter(cls, "phx."); end
                it.class = char(cls);
                if isa(cur, 'phx.Body')
                    it.type = char(obj.designType(cur));
                    it.position = round(cur.Position, 3);
                end
                items{end + 1} = it; %#ok<AGROW> small scene
                stack = [stack cur.Children]; %#ok<AGROW> tree walk
            end
            s = string(jsonencode(items));
        end

        function n = applyOps(obj, ops)
            n = 0;
            for i = 1:numel(ops)
                op = ops{i};
                try
                    switch lower(string(op.op))
                        case "add_object"
                            obj.agentAddObject(op);
                        case "set_property"
                            obj.agentSetProperty(op);
                        case "delete_object"
                            o = obj.findByName(string(op.target));
                            if ~isempty(o)
                                if isa(o, 'phx.Body') && isequal(obj.Viewer.SelectedBody, o)
                                    obj.Viewer.deselect;
                                end
                                obj.SelectedObject = [];
                                delete(o);
                            end
                        case "connect"
                            obj.agentConnect(op);
                        case "attach"
                            obj.agentAttach(op);
                    end
                    n = n + 1;
                catch err
                    obj.appendChat("[op error] " + string(err.message));
                end
            end
        end

        function agentAddObject(obj, op)
            kinds = string(fieldnames(obj.Palette));
            k = kinds(strcmpi(kinds, string(op.type)));
            if isempty(k)
                error("Unknown object type '%s'.", string(op.type));
            end
            shapeCell = obj.shapeCellFromParams(k(1), op);
            body = obj.addBody(k(1), shapeCell);
            if isfield(op, "name") && strlength(string(op.name)) > 0
                body.Name = string(op.name);
            end
            if isfield(op, "position")
                p = obj.toVec(op.position);
                if numel(p) == 3
                    body.Position = p;
                end
            end
            if isfield(op, "color")
                c = obj.toVec(op.color);
                if numel(c) == 3
                    body.Color = c;
                end
            end
        end

        function shapeCell = shapeCellFromParams(obj, kind, op)
            % Build a shape cell {kind, name, val, ...} from an AI add_object's
            % optional "params" JSON. Only parameters that are real, editable
            % shape properties are accepted (others are ignored for safety).
            shapeCell = [];
            if ~isfield(op, "params") || isempty(op.params)
                return
            end
            p = obj.parseParams(op.params);
            allowed = string(obj.shapeFields(feval("phx.shape." + kind)));
            pairs = {};
            for f = string(fieldnames(p))'
                if ~ismember(f, allowed)
                    continue
                end
                v = obj.toVal(p.(f));
                if isnumeric(v)
                    v = v(:)';   % JSON arrays decode as columns; shapes want rows
                end
                pairs{end + 1} = char(f); %#ok<AGROW>
                pairs{end + 1} = v; %#ok<AGROW>
            end
            if ~isempty(pairs)
                shapeCell = [{char(kind)}, pairs];
            end
        end

        function s = parseParams(~, x)
            % Accept a struct (already-decoded JSON) or a JSON string.
            if isstruct(x)
                s = x;
                return
            end
            try
                s = jsondecode(char(string(x)));
            catch
                s = struct();
            end
            if ~isstruct(s)
                s = struct();
            end
        end

        function agentSetProperty(obj, op)
            o = obj.findByName(string(op.target));
            if isempty(o)
                error("No object named '%s'.", string(op.target));
            end
            prop = char(string(op.property));
            val = obj.toVal(op.value);
            if isa(o, 'phx.Body') && strcmpi(prop, "Type")
                obj.setBodyType(o, string(val));
            else
                o.(prop) = val;
            end
        end

        function agentConnect(obj, op)
            allowed = ["FixedJoint", "RevoluteJoint", "SphericalJoint", "Spring"];
            cls = string(op.type);
            if ~ismember(cls, allowed)
                error("Unknown connector type '%s'.", cls);
            end
            A = obj.findByName(string(op.bodyA));
            B = obj.findByName(string(op.bodyB));
            if isempty(A) || isempty(B)
                error("Both bodies must exist (got '%s', '%s').", string(op.bodyA), string(op.bodyB));
            end
            if ~isa(A, 'phx.Body') || ~isa(B, 'phx.Body')
                error("connect requires two bodies.");
            end
            if isequal(A, B)
                error("connect requires two different bodies.");
            end
            feval("phx." + cls, A, B);
        end

        function agentAttach(obj, op)
            allowed = ["Thruster", "Trace"];
            cls = string(op.type);
            if ~ismember(cls, allowed)
                error("Unknown attachment type '%s'.", cls);
            end
            T = obj.findByName(string(op.target));
            if isempty(T) || ~isa(T, 'phx.Body')
                error("attach target must be an existing body (got '%s').", string(op.target));
            end
            feval("phx." + cls, T);
        end

        function o = findByName(obj, name)
            o = [];
            stack = num2cell(obj.bodies);
            while ~isempty(stack)
                cur = stack{1};
                stack(1) = [];
                if ~ismissing(cur.Name) && cur.Name == name
                    o = cur;
                    return
                end
                stack = [stack cur.Children]; %#ok<AGROW> tree walk
            end
        end

        function v = toVec(~, x)
            % Accept a numeric vector or a string like "[1 2 3]".
            if isnumeric(x)
                v = x(:)';
            else
                v = str2num(char(string(x))); %#ok<ST2NM> flexible parse
                v = v(:)';
            end
        end

        function v = toVal(~, x)
            % Numbers/vectors stay numeric; strings that parse as numbers are
            % converted, otherwise the string is kept.
            if isnumeric(x) || islogical(x)
                v = x;
                return
            end
            num = str2num(char(string(x))); %#ok<ST2NM> flexible parse
            if isempty(num)
                v = string(x);
            else
                v = num;
            end
        end

        function [ops, reply] = mockBackend(obj, userText, sceneSummary, toolSpec)
            % Offline rule-based stand-in so the chat loop works without a key.
            ops = {};
            t = lower(string(userText));
            kinds = string(fieldnames(obj.Palette));
            palette = struct('red', [1 0 0], 'green', [0 1 0], ...
                'blue', [0 0 1], 'yellow', [1 1 0], 'grey', [0.6 0.6 0.6]);

            if contains(t, "add") || contains(t, "create")
                for k = kinds'
                    if contains(t, lower(k))
                        op = struct('op', "add_object", 'type', char(k));
                        for c = string(fieldnames(palette))'
                            if contains(t, c)
                                op.color = palette.(c);
                            end
                        end
                        ops{end + 1} = op; %#ok<AGROW> small list
                    end
                end
            end

            if (contains(t, "delete") || contains(t, "remove")) && ~isempty(obj.SelectedObject)
                ops{end + 1} = struct('op', "delete_object", ...
                    'target', char(string(obj.SelectedObject.Name)));
            end

            if isempty(ops)
                reply = "Mock backend: no actionable command recognized. Try " + ...
                    """add a red box"". Connect a real model with useClaudeBackend.";
            else
                reply = "Mock backend parsed your request.";
            end
        end

        function defs = agentToolDefs(obj, toolSpec)
            % Provider-neutral tool definitions (name/description/input_schema).
            % Each adapter wraps these into its own request format. Constrained
            % fields use JSON-schema enums — a big reliability win for small
            % local models. objSchema row = {name, type, description, enumCell}.
            kinds = cellstr(toolSpec.types);
            propNames = {'Position', 'EulerAngles', 'Mass', 'Friction', ...
                'Type', 'Collisions', 'Color', 'Name'};

            addTool = struct('name', 'add_object', ...
                'description', 'Add a new physics body to the scene. Call once per body.');
            addTool.input_schema = obj.objSchema( ...
                {'type',     'string', 'Shape kind',                                kinds; ...
                 'name',     'string', 'Optional name for later reference',         []; ...
                 'position', 'string', 'Optional world position "[x y z]" (meters, z up)', []; ...
                 'color',    'string', 'Optional color "[r g b]" with values 0-1',  []; ...
                 'params',   'string', 'Optional shape parameters as a JSON object, e.g. {"Size":[4,1,1]} for a long Box or {"Diameter":2,"Height":3} for a Cylinder', []}, ...
                {'type'});

            setTool = struct('name', 'set_property', ...
                'description', 'Change one property of an existing object. Call once per change.');
            setTool.input_schema = obj.objSchema( ...
                {'target',   'string', 'Name of the existing object to modify',     []; ...
                 'property', 'string', 'Which property to set',                     propNames; ...
                 'value',    'string', 'New value: a number, "[x y z]", or for Type one of static/kinematic/dynamic', []}, ...
                {'target', 'property', 'value'});

            delTool = struct('name', 'delete_object', ...
                'description', 'Delete an existing object by name.');
            delTool.input_schema = obj.objSchema( ...
                {'target', 'string', 'Name of the object to delete', []}, {'target'});

            connTool = struct('name', 'connect', ...
                'description', 'Connect two existing bodies with a joint or spring.');
            connTool.input_schema = obj.objSchema( ...
                {'bodyA', 'string', 'Name of the first body',            []; ...
                 'bodyB', 'string', 'Name of the second body',           []; ...
                 'type',  'string', 'Connector type', ...
                    {'FixedJoint', 'RevoluteJoint', 'SphericalJoint', 'Spring'}}, ...
                {'bodyA', 'bodyB', 'type'});

            attachTool = struct('name', 'attach', ...
                'description', 'Attach a force or helper element to an existing body.');
            attachTool.input_schema = obj.objSchema( ...
                {'target', 'string', 'Name of the body to attach to',    []; ...
                 'type',   'string', 'Element type', {'Thruster', 'Trace'}}, ...
                {'target', 'type'});

            defs = {addTool, setTool, delTool, connTool, attachTool};
        end

        function sys = agentSystemPrompt(~, toolSpec, sceneSummary)
            lines = [
                "You are an assistant that builds and edits a 3D physics scene by calling the"
                "provided tools. Always act by calling tools — do not reply in prose."
                ""
                "Coordinate system (important):"
                "- Positions are absolute world coordinates ""[x y z]"" in meters."
                "- x and y are the two HORIZONTAL (ground) directions; z is the VERTICAL (height) axis."
                "- ""up"", ""above"", ""on top"", ""stack"", ""tower"" and ""pyramid"" all mean INCREASING z"
                "  while keeping x and y the same. Never use x or y for height."
                "- The ground plane is at z = 0. A default body is about 1 m, so to sit bodies on top"
                "  of each other use z = 0.5, 1.5, 2.5, … (spacing ≈ 1)."
                "- Colors are ""[r g b]"" with each component between 0 and 1."
                "- Reference existing objects by their Name (see the scene JSON below)."
                "- For a request needing several objects, make one add_object call per object."
                ""
                "Shape geometry:"
                "- Control a body's shape with add_object's params (a JSON object). Use it for"
                "  non-default sizes, e.g. a long/elongated box or a flat slab."
                "    Box: {""Size"":[x,y,z]}    Sphere/Rock: {""Diameter"":d}"
                "    Cylinder/Capsule/Cone: {""Diameter"":d,""Height"":h}"
                ""
                "Connecting and attaching:"
                "- Bodies can be linked with connect(bodyA, bodyB, type) where type is one of"
                "  FixedJoint, RevoluteJoint, SphericalJoint, Spring. Both bodies must already exist;"
                "  give them names when you create them so you can connect them afterwards."
                "- Force/helper elements attach to one body with attach(target, type) where type is"
                "  Thruster or Trace."
                ""
                "Examples:"
                "- ""add a red box""  -> add_object(type=""Box"", color=""[1 0 0]"")"
                "- ""add a long box (elongated prism)""  -> add_object(type=""Box"", params=""{""""Size"""":[4,1,1]}"")"
                "- ""stack three boxes into a tower""  ->  add_object(type=""Box"", position=""[0 0 0.5]"");"
                "      add_object(type=""Box"", position=""[0 0 1.5]"");  add_object(type=""Box"", position=""[0 0 2.5]"")"
                "- ""put a ball on top of the tower""  -> add_object(type=""Sphere"", position=""[0 0 3.5]"")"
                "- ""make the floor static""  -> set_property(target=""Ground"", property=""Type"", value=""static"")"
                "- ""hang a ball from the ceiling with a spring""  ->"
                "      add_object(type=""Sphere"", name=""ball"", position=""[0 0 2]"");  connect(bodyA=""Ground"", bodyB=""ball"", type=""Spring"")"
                "- ""link box1 and box2 with a revolute joint""  -> connect(bodyA=""box1"", bodyB=""box2"", type=""RevoluteJoint"")"
                "- ""add a thruster to the rocket""  -> attach(target=""rocket"", type=""Thruster"")"
                ""
                "Valid body types: " + strjoin(toolSpec.types, ", ") + "."
                "Current scene (JSON): " + sceneSummary
                ];
            sys = strjoin(lines, newline);
        end

        function op = toolCallToOp(~, name, input)
            % Normalize a model tool call into an op struct.
            if isstruct(input)
                op = input;
            else
                op = struct();
            end
            op.op = string(name);
        end

        function [ops, reply] = claudeRequest(obj, userText, sceneSummary, toolSpec, apiKey, model)
            % Reference Claude adapter (raw HTTP — MATLAB has no Anthropic SDK).
            % NOTE: spike-level; not exercised in the headless test suite.
            body = struct();
            body.model = char(model);
            body.max_tokens = 1024;
            body.system = char(obj.agentSystemPrompt(toolSpec, sceneSummary));
            body.tools = obj.agentToolDefs(toolSpec);   % Anthropic wants name/description/input_schema
            body.messages = {struct('role', 'user', 'content', char(userText))};

            opts = weboptions('MediaType', 'application/json', 'Timeout', 60, ...
                'HeaderFields', {'x-api-key', char(apiKey); 'anthropic-version', '2023-06-01'});
            resp = webwrite("https://api.anthropic.com/v1/messages", body, opts);

            ops = {};
            replyParts = strings(0);
            content = resp.content;
            if ~iscell(content)
                content = num2cell(content);
            end
            for i = 1:numel(content)
                b = content{i};
                switch string(b.type)
                    case "text"
                        replyParts(end + 1) = string(b.text); %#ok<AGROW>
                    case "tool_use"
                        ops{end + 1} = obj.toolCallToOp(b.name, b.input); %#ok<AGROW>
                end
            end
            reply = strjoin(replyParts, " ");
        end

        function [ops, reply] = ollamaRequest(obj, userText, sceneSummary, toolSpec, model, url)
            % Local Ollama adapter (raw HTTP, /api/chat with tool calling).
            % Free and offline; needs a running Ollama server with the model
            % pulled. NOTE: spike-level; not exercised in the headless suite.
            defs = obj.agentToolDefs(toolSpec);
            tools = cell(1, numel(defs));
            for i = 1:numel(defs)
                d = defs{i};
                fn = struct('name', d.name, 'description', d.description, ...
                    'parameters', d.input_schema);
                tools{i} = struct('type', 'function', 'function', fn);
            end

            body = struct();
            body.model = char(model);
            body.stream = false;
            body.tools = tools;
            body.options = struct('temperature', 0);   % deterministic tool choice
            body.messages = { ...
                struct('role', 'system', 'content', char(obj.agentSystemPrompt(toolSpec, sceneSummary))), ...
                struct('role', 'user', 'content', char(userText))};

            opts = weboptions('MediaType', 'application/json', 'Timeout', 120);
            resp = webwrite(url + "/api/chat", body, opts);

            ops = {};
            msg = resp.message;
            reply = "";
            if isfield(msg, 'content')
                reply = string(msg.content);
            end
            if isfield(msg, 'tool_calls') && ~isempty(msg.tool_calls)
                calls = msg.tool_calls;
                if ~iscell(calls)
                    calls = num2cell(calls);
                end
                for i = 1:numel(calls)
                    fn = calls{i}.function;
                    ops{end + 1} = obj.toolCallToOp(fn.name, fn.arguments); %#ok<AGROW>
                end
            end
        end

        function schema = objSchema(~, rows, required)
            % Build a JSON-schema object. Each row is {name, type, description}
            % or {name, type, description, enumCell}; a non-empty enumCell adds
            % a JSON-schema "enum" constraint.
            props = struct();
            for r = 1:size(rows, 1)
                p = struct('type', rows{r, 2}, 'description', rows{r, 3});
                if size(rows, 2) >= 4 && ~isempty(rows{r, 4})
                    p.enum = rows{r, 4};
                end
                props.(rows{r, 1}) = p;
            end
            schema = struct('type', 'object', 'properties', props, 'required', {required});
        end
    end

    % ------------------------------------------------------------------- UI
    methods (Access = private)
        function buildUI(obj)
            obj.UIFigure = uifigure("Name", "PHX Model Editor", ...
                "Position", [100 100 1100 680], ...
                "CloseRequestFcn", @(~, ~) obj.delete);

            obj.buildMenu;

            g = uigridlayout(obj.UIFigure, [3 1], "RowHeight", {38, "1x", 150}, ...
                "RowSpacing", 4, "Padding", 4);

            obj.buildToolbar(g);

            content = uigridlayout(g, [1 3], ...
                "ColumnWidth", {240, "1x", 300}, "ColumnSpacing", 6, "Padding", 0);

            obj.buildLeftPanel(content);

            obj.Axes = uiaxes(content);
            obj.Axes.Layout.Column = 2;

            obj.buildInspector(content);

            obj.buildChat(g);
        end

        function buildChat(obj, parent)
            p = uipanel(parent, "Title", "AI assistant (spike)");
            cg = uigridlayout(p, [2 1], "RowHeight", {"1x", 28}, ...
                "Padding", 6, "RowSpacing", 4);

            obj.ChatLog = uitextarea(cg, "Editable", "off", "Value", ...
                {'Ask me to build or edit the scene, e.g. "add a red sphere".'; ...
                 'Default backend is an offline mock; call useClaudeBackend(apiKey) for the real model.'});

            row = uigridlayout(cg, [1 2], "Padding", 0, "ColumnSpacing", 4, ...
                "ColumnWidth", {"1x", 80});
            obj.ChatInput = uieditfield(row, "text", ...
                "Placeholder", "Type a request and press Send…", ...
                "ValueChangedFcn", @(~, ~) obj.agentSend);
            obj.SendButton = uibutton(row, "Text", "Send", "ButtonPushedFcn", @(~, ~) obj.agentSend);
            obj.EditControls{end + 1} = obj.ChatInput;
        end

        function buildMenu(obj)
            fileMenu = uimenu(obj.UIFigure, "Text", "&File");
            uimenu(fileMenu, "Text", "&New", "Accelerator", "N", ...
                "MenuSelectedFcn", @(~, ~) obj.fileNew);
            uimenu(fileMenu, "Text", "&Open...", "Accelerator", "O", ...
                "MenuSelectedFcn", @(~, ~) obj.fileOpen);
            uimenu(fileMenu, "Text", "&Save...", "Accelerator", "S", ...
                "MenuSelectedFcn", @(~, ~) obj.fileSave);
            uimenu(fileMenu, "Text", "E&xit", "Separator", "on", ...
                "MenuSelectedFcn", @(~, ~) obj.delete);

            obj.InsertMenu = uimenu(obj.UIFigure, "Text", "&Insert");
            obj.populateAddMenu(obj.InsertMenu);
            obj.EditControls{end + 1} = obj.InsertMenu;
        end

        function buildToolbar(obj, parent)
            tb = uigridlayout(parent, [1 9], "Padding", 2, "ColumnSpacing", 4, ...
                "ColumnWidth", {80, 16, 60, 70, 60, 80, 16, 90, "1x"});
            mk = @(txt, cb) uibutton(tb, "Text", txt, "ButtonPushedFcn", cb);

            obj.AddMenu = uicontextmenu(obj.UIFigure);
            obj.AddButton = mk("＋ Add ▾", @(~, ~) obj.openAddMenu);
            obj.EditControls{end + 1} = obj.AddButton;
            uilabel(tb, "Text", "");
            obj.Toolbar.Play = mk("▶ Play", @(~, ~) obj.startPlay);
            obj.Toolbar.Pause = mk("❚❚ Pause", @(~, ~) obj.pausePlay);
            obj.Toolbar.Pause.Enable = "off";
            mk("Step", @(~, ~) obj.singleStep);
            mk("■ Stop/Reset", @(~, ~) obj.stopPlay);
            uilabel(tb, "Text", "");
            mk("Home view", @(~, ~) obj.Viewer.basicView("home"));
        end

        function buildLeftPanel(obj, parent)
            op = uipanel(parent, "Title", "Scene");
            op.Layout.Column = 1;
            og = uigridlayout(op, [2 1], "RowHeight", {"1x", 28}, "Padding", 6, ...
                "RowSpacing", 4);

            obj.Tree = uitree(og, "SelectionChangedFcn", @(~, ~) obj.onTreeSelect);
            obj.TreeMenu = uicontextmenu(obj.UIFigure);
            obj.TreeMenu.ContextMenuOpeningFcn = @(~, ~) obj.buildTreeMenu;
            obj.Tree.ContextMenu = obj.TreeMenu;

            btnRow = uigridlayout(og, [1 2], "Padding", 0, "ColumnSpacing", 4);
            bDup = uibutton(btnRow, "Text", "Duplicate", "ButtonPushedFcn", @(~, ~) obj.duplicateSelected);
            bDel = uibutton(btnRow, "Text", "Delete", "ButtonPushedFcn", @(~, ~) obj.deleteSelected);
            obj.EditControls = [obj.EditControls {obj.Tree, bDup, bDel}];
        end

        function buildInspector(obj, parent)
            ip = uipanel(parent, "Title", "Inspector");
            ip.Layout.Column = 3;
            g = uigridlayout(ip, [2 1], "RowHeight", {"1x", 22}, ...
                "Padding", 6, "RowSpacing", 4);

            obj.Table = uitable(g, "ColumnName", {'Parameter', 'Value'}, ...
                "ColumnEditable", [false true], "RowName", {}, ...
                "ColumnWidth", {130, "1x"}, ...
                "SelectionType", "row", "Multiselect", "off", ...
                "CellEditCallback", @obj.onCellEdit, ...
                "CellSelectionCallback", @obj.onCellSelect);

            obj.InspectorMenu = uicontextmenu(obj.UIFigure);
            obj.InspectorMenu.ContextMenuOpeningFcn = @(~, ~) obj.buildRowMenu;
            obj.Table.ContextMenu = obj.InspectorMenu;

            uilabel(g, "Text", "Tip: double-click a value to edit; right-click for quick options.", ...
                "FontAngle", "italic", "FontColor", [0.4 0.4 0.4]);

            obj.EditControls{end + 1} = obj.Table;
        end

        function setEditingEnabled(obj, tf)
            if tf
                state = 'on';
            else
                state = 'off';
            end
            for i = 1:numel(obj.EditControls)
                h = obj.EditControls{i};
                if ~isempty(h) && isvalid(h)
                    h.Enable = state;
                end
            end
        end
    end
end
