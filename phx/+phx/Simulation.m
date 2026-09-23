classdef Simulation < phx.base.Object
%phx.Simulation Physical simulation
%
%   Simulation is the topmost object in the hierarchy of the physical
%   scene.
%
%   phx.Simulation() creates a simulation of all objects in the current axes.
%
%   phx.Simulation(bodies) creates a simulation of given bodies (array or
%   cell array) and their children objects.
%
%   phx.Simulation(___, Name, Value, ...) creates a simulation and sets
%   properties values according to given name-value pairs.
%
%   See also phx.Body

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

%#ok<*INUSD> OK to see the full list of arguments for callbacks
%#ok<*MCSUP> OK to access other properties in setters

    properties (Access = private)
        % Execution pipelines
        EPCompute % compute only
        EPComplete % compute and redraw

        SortedBodiesID

        % Nesting depth of holdPipelines
        PipelineHold = 0
    end

    properties (SetAccess = private)
        % Simulation time
        Time = 0
    end

    properties
        % Figure update mode
        %   full - smoothest rendering, UI callbacks are processed
        %   interactive - faster rendering, UI callbacks are processed
        %   performance - fastest rendering, the figure does not react during a step
        %   none - no screen update, the figure is refreshed once MATLAB is idle
        RedrawMode {mustBeMember(RedrawMode, ["full", "interactive", "performance", "none"])} = "interactive"

        % Exclude all initially invisible objects from rendering
        ExcludeInvisible (1, 1) logical = true

        % Turn off visibility for all objects that become invalid during the simulation
        HideInvalid (1, 1) logical = false

        % Global gravitational acceleration
        Gravity (1, 3) double = [0 0 -9.81]

        % Initial engine-specific settings
        EngineSettings = []
    end

    methods
        function obj = Simulation(Children, Options)
            arguments
                Children = gca
                Options.?phx.Simulation
            end

            % Set default values
            obj.SimulationOrder = "after";
            obj.RedrawOrder = "none";

            % Process input arguments
            phx.internal.applyArguments(Options, obj);
            if ~isempty(Children)
                obj.addObjects(Children);
            end
        end

        function bodies = addObjects(obj, simObjects)
        %addObjects Adds phx.Body objects with all of their children objects to
        % the current simulation and rebuilds execution pipelines.
        %
        %   addObjects(axes) adds all bodies available in the given axes object.
        %
        %   addObjects(fileName) adds all bodies available in the given MAT file.
        %
        %   addObjects(bodies) adds all bodies from given array, cell array or structure.
        %
        %   bodies = addObjects(___) returns added bodies for further manipulation.
        %
        % See also phx.Simulation, phx.Body

            bodies = {};
            switch class(simObjects)
                case {'matlab.graphics.axis.Axes', 'matlab.ui.control.UIAxes'}
                    obj.Children = addParent(phx.Simulation.findBodies(simObjects), obj);
                    bodies = obj.Children;
                case {'string', 'char'}
                    mat = load(simObjects);
                    bodies = obj.addObjects(mat);
                case 'struct'
                    for field = fieldnames(simObjects)'
                        switch class(simObjects.(field{1}))
                            case 'phx.Body'
                                bodies = [bodies simObjects.(field{1})]; %#ok<AGROW> unknown count
                        end
                    end
                    bodies = obj.addObjects(bodies);
                case {'phx.Body', 'cell'}
                    bodies = addParent(simObjects, obj, "phx.Body");
                    bodies = bodies(:)'; % ensure row format
                    obj.Children = [obj.Children bodies];
                otherwise
                    error("phx:Simulation:unsupportedSource", "Unsupported source type '%s'.", class(simObjects));
            end
            
            obj.updatePipelines;
        end

        function dispPipelines(obj)
        %dispPipelines Displays execution pipelines.
        %
        %   dispPipelines(simulation)
        %
        % See also phx.Simulation.dispStructure

            disp(" ");
            disp("    Compute pipeline:");
            for i = 1:numel(obj.EPCompute)
                disp("    "+i+" - "+func2str(obj.EPCompute(i).Function)+": "+numel(obj.EPCompute(i).Objects)+" object(s)");
            end
            disp(" ");
            disp("    Compute & redraw pipeline:");
            for i = 1:numel(obj.EPComplete)
                disp("    "+i+" - "+func2str(obj.EPComplete(i).Function)+": "+numel(obj.EPComplete(i).Objects)+" object(s)");
            end
            disp(" ");
        end

        function step(obj, interval, substeps, redrawStep)
        %STEP Performs one or more simulation steps.
        %
        %   If multiple substeps are executed within the total step interval,
        %   the rendering routines can be executed at a lower frequency.
        %
        %   step(simulation, interval) simulates the entire interval as one
        %   step.
        %
        %   step(simulation, interval, substeps) simulates the entire interval
        %   divided into a given number of substeps. Rendering is performed
        %   only once within the last substep.
        %
        %   step(simulation, interval, substeps, redrawStep) same as previous
        %   but the rendering is performed at every specified substep.
        %
        % See also phx.Simulation

        % TODO substep, redrawstep: count or period?
        % - Time vector instead of step & count? But it might be confusing when the simulation does not start at time 0.

            arguments
                obj 
                interval (1, 1) double = 0.01
                substeps (1, 1) double = 1
                redrawStep (1, 1) double = 0
            end

            dt = interval/substeps;
            time = obj.Time;
            world = obj.ObjectHandle;
            epCompute = obj.EPCompute;
            epComplete = obj.EPComplete;

            % Simulation steps
            for s = 1:substeps
                animate = (redrawStep >= 0) && (mod(s, redrawStep) == 0);
                updateView = (redrawStep >= 0) && ((s == substeps) || animate);

                if updateView
                    for p = 1:numel(epComplete)
                        epComplete(p).Function(epComplete(p).Objects, dt, time, world);
                    end
                else
                    for p = 1:numel(epCompute)
                        epCompute(p).Function(epCompute(p).Objects, dt, time, world);
                    end
                end

                time = time + dt;

                if animate
                    switch obj.RedrawMode
                        case "performance"
                            matlab.graphics.internal.drawnow.startUpdate;
                        case "interactive"
                            pause(0);
                        case "full"
                            drawnow;
                    end
                end
            end

            obj.Time = obj.Time + interval;
        end

        function set.Gravity(obj, value)
            obj.Gravity = value;
            if ~isempty(obj.ObjectHandle)
                phx.engine.io('set', obj.ObjectHandle, uint64(0), 'gravity', value);
            end
        end
    end

    methods (Access = private)
        function addPipelineEntries(obj, simObjects, classes, mask, hook, alsoCompute)
        %addPipelineEntries Appends one pipeline entry per class in the mask.
        %
        %   The objects picked by mask are grouped by class, in first-seen
        %   order, and each group becomes a struct(Function, Objects) whose
        %   Function is that class's static hook. With alsoCompute the entry
        %   goes into both pipelines, otherwise only into the redraw one.

            picked = classes(mask);
            if isempty(picked)
                return
            end
            children = simObjects(mask);
            uClasses = unique(picked, "stable");
            for i = 1:numel(uClasses)
                entry = struct("Function", str2func(uClasses(i)+"."+hook), ...
                    "Objects", {children(picked == uClasses(i))});
                if alsoCompute
                    obj.EPCompute(end + 1) = entry;
                end
                obj.EPComplete(end + 1) = entry;
            end
        end
    end

    methods (Access = ?phx.base.Object)
        function holdPipelines(obj)
        %holdPipelines Defers pipeline rebuilds until the matching release.
        %
        %   Deleting an object can cascade into deleting others, and each of
        %   those would otherwise rebuild the pipelines on its own. Holding
        %   collapses the whole cascade into the single rebuild that the
        %   matching releasePipelines performs. Calls nest.
        %
        % See also phx.Simulation.releasePipelines

            obj.PipelineHold = obj.PipelineHold + 1;
        end

        function releasePipelines(obj)
        %releasePipelines Ends a hold and rebuilds the pipelines.
        %
        %   The outermost release does the rebuild; nested ones only count
        %   down. A hold is only ever taken around a change to the object
        %   graph, so there is always something to rebuild for.
        %
        % See also phx.Simulation.holdPipelines

            obj.PipelineHold = max(obj.PipelineHold - 1, 0);
            if obj.PipelineHold == 0
                obj.updatePipelines;
            end
        end

        function updatePipelines(obj)
            % Held: the release rebuilds once for the whole cascade
            if obj.PipelineHold > 0
                return
            end

            % Prepare empty execution pipelines structs
            epStruct = @(f, o) struct("Function", f, "Objects", o);
            obj.EPCompute = epStruct([], []);
            obj.EPComplete = epStruct([], []);

            % Collect all unique objects sorted by level
            simObjects = obj.sortedBranchObjects;

            % Init all objects
            valid = true(size(simObjects));
            for i = 1:numel(simObjects)
                valid(i) = isvalid(simObjects{i}) && simObjects{i}.initObject(obj.ObjectHandle);
            end

            % Hide invalid objects
            if obj.HideInvalid
                for i = find(~valid)
                    simObjects{i}.Visible = false;
                end
            end

            % Exclude invalid objects from pipelines
            simObjects(~valid) = [];

            % Store bodies IDs
            n = numel(obj.Children);
            obj.SortedBodiesID = zeros(n, 2, 'uint64');
            for i = 1:n
                obj.SortedBodiesID(i, :) = [i obj.Children{i}.ObjectHandle];
            end
            obj.SortedBodiesID = sortrows(obj.SortedBodiesID, 2);

            % Read everything the grouping below needs in a single pass. The
            % properties are named literally, so these are plain static reads;
            % filtering the cell array four times over asked for each of them
            % by name and paid for a dynamic lookup every time.
            n = numel(simObjects);
            simOrder = strings(1, n);
            redrawOrder = strings(1, n);
            shown = true(1, n);
            classes = strings(1, n);
            for i = 1:n
                child = simObjects{i};
                simOrder(i) = child.SimulationOrder;
                redrawOrder(i) = child.RedrawOrder;
                classes(i) = class(child);
                if obj.ExcludeInvisible
                    shown(i) = child.Visible;
                end
            end

            % Resolve state for objects preceding movement
            obj.addPipelineEntries(simObjects, classes, simOrder == "before", "resolveState", true);

            % Update view for objects preceding movement
            obj.addPipelineEntries(simObjects, classes, redrawOrder == "before" & shown, "updateView", false);

            % Resolve state for objects following movement
            obj.addPipelineEntries(simObjects, classes, simOrder == "after", "resolveState", true);

            % Update view for objects following movement
            obj.addPipelineEntries(simObjects, classes, redrawOrder == "after" & shown, "updateView", false);

            % Remove first array element (because it is empty)
            obj.EPComplete(1) = [];
            obj.EPCompute(1) = [];
            %obj.dispPipelines; % DEBUG display pipelines
        end
    end

    methods (Access = protected)
        function valid = initObject(obj, world)
            if isempty(obj.ObjectHandle)
                obj.ObjectHandle = phx.engine.io('setup', 2);
                phx.engine.io('set', obj.ObjectHandle, uint64(0), 'gravity', obj.Gravity);
                phx.engine.io('set', obj.ObjectHandle, uint64(0), 'autoactivated', false);

                % Apply engine-specific settings
                if ~isempty(obj.EngineSettings)
                    obj.EngineSettings.apply(obj.ObjectHandle);
                end
            end
            valid = true;
        end

        function destroyObject(obj)
            for i = 1:numel(obj.Children)
                state(i) = obj.Children{i}.stateTransfer;
            end

            phx.engine.io('clear', obj.ObjectHandle);
            obj.propagate('ObjectHandle', []);

            for i = 1:numel(obj.Children)
                if isvalid(obj.Children{i}.Graphics)
                    obj.Children{i}.stateTransfer(state(i));
                end
            end
        end
    end

    methods (Static, Access = protected)
        function resolveState(cellObjs, dt, time, world)
            obj = cellObjs{1};
            [bid, mtx] = phx.engine.io('step', world, dt);
            if ~islogical(bid)
                BID = obj.SortedBodiesID;
                [~, locb] = matlab.internal.math.ismemberhelper(bid, BID(:, 2), true);
                for i = 1:numel(bid)
                    obj.Children{BID(locb(i))}.Matrix = reshape(mtx((i*16)-15:(i*16)), 4, 4);
                end
            end
        end

        function updateView(cellObjs, dt, time, world)
        end
    end

    methods (Static)
        function obj = loadobj(obj)
            obj.updatePipelines;
        end

        function bodies = findBodies(ax)
            bodies = phx.Body.empty;
            children = ax.Children;
            for i = 1:numel(children)
                body = getappdata(children(i), 'phxObject');
                if ~isempty(body) && isa(body, 'phx.Body') && isvalid(body)
                    bodies(end + 1) = body; %#ok<AGROW> unknown size
                end
            end
        end
    end

end