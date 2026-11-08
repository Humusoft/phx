classdef Simulation < phx.base.Object
%phx.Simulation Scene root that steps the physics engine
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

        % Nesting depth of holdPipelines
        PipelineHold = 0
    end

    properties (Access = private, Transient)
        % Pacing anchor: tic at the anchor (0 = not anchored) and the
        % simulation time at the same moment
        PacingClock = uint64(0)
        PacingTime = 0

        % Adaptive redraw: tic at the start of the last redraw and
        % whether the next one has to wait for the renderer (set when the
        % pipelines are built before the simulation starts, so the new scene
        % reaches the screen before the redraws start to be skipped)
        RedrawClock = uint64(0)
        RedrawSync = true

        % Adaptive redraw: how long the last redraw took (graphics update and
        % flush, without the pacing wait), the last call of step, and the
        % simulation time of the last drawn frame
        RedrawDuration = 0
        StepDuration = Inf
        RedrawSimTime = -Inf
    end

    properties (Constant, Access = private)
        % Shortest interval between figure updates in the adaptive redraw mode;
        % the renderer shows at most one frame per display refresh anyway
        AdaptiveRedrawInterval = 1/60

        % In the adaptive redraw mode a redraw also waits this many durations of
        % the previous one, so expensive redraws take at most half of the time
        AdaptiveRedrawRatio = 2

        % Lateness (s of real time) that pacing still makes up; pause oversleeps
        % short waits by up to a timer tick (~15 ms on Windows)
        PacingTolerance = 0.1
    end

    properties (SetAccess = private)
        % Simulation time
        Time = 0
    end

    properties
        % Figure update mode
        %   adaptive - redraws as often as the renderer and the cost of a redraw
        %     allow (a redraw coming too soon is skipped), UI callbacks are
        %     processed; the graphics are brought up to date at the end of
        %     every step
        %   full - every requested redraw is drawn and waited for, UI callbacks
        %     are processed (exact frames, e.g. for recording)
        %   performance - every requested redraw is sent without waiting, the
        %     figure does not react during a step (predictable cost, e.g. for
        %     benchmarks)
        %   none - no screen update, the figure is refreshed once MATLAB is idle
        RedrawMode {mustBeMember(RedrawMode, ["adaptive", "full", "performance", "none"])} = "adaptive"

        % Exclude all initially invisible objects from rendering
        ExcludeInvisible (1, 1) logical = true

        % Turn off visibility for all objects that become invalid during the simulation
        HideInvalid (1, 1) logical = false

        % Maximum ratio of simulation time to real time (Inf = unpaced)
        %   Checked at every requested redraw and at the end of each step; a
        %   delay of up to 0.1 s is made up, a simulation that is later
        %   continues from there and never catches up.
        PacingRate (1, 1) double {mustBePositive} = Inf

        % Longest stretch of simulation time without a frame (s), adaptive mode
        %   While the simulation runs ahead of real time, the adaptive redraw
        %   mode still draws a frame at least this often in simulation time,
        %   so a light scene is seen moving instead of jumping to its end.
        %   Raise it for long time scales; Inf turns the rule off.
        FrameSpacing (1, 1) double {mustBePositive} = 1/20

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
        %addObjects Adds top-level objects with all of their children objects to
        % the current simulation and rebuilds execution pipelines.
        %
        %   A top-level object is one that no other object owns, such as a
        %   phx.Body: everything attached to it (joints, springs, sensors)
        %   comes along as its child. Soft bodies, flexible bodies and
        %   particle groups are top-level objects too.
        %
        %   addObjects(axes) adds all top-level objects drawn in the given axes.
        %
        %   addObjects(fileName) adds all top-level objects in the given MAT file.
        %
        %   addObjects(objects) adds the objects of the given array, cell array
        %   or structure; an object owned by another one is an error.
        %
        %   bodies = addObjects(___) returns the added objects for further
        %   manipulation.
        %
        % See also phx.Simulation, phx.Body

            bodies = {};
            if isa(simObjects, 'matlab.graphics.axis.Axes') || isa(simObjects, 'matlab.ui.control.UIAxes')
                obj.Children = addParent(phx.Simulation.findTopLevel(simObjects), obj);
                bodies = obj.Children;
            elseif ischar(simObjects) || isstring(simObjects)
                bodies = obj.addObjects(load(simObjects));
            elseif isstruct(simObjects)
                % Fields holding anything else, such as joints or settings, are left out
                for field = fieldnames(simObjects)'
                    value = simObjects.(field{1});
                    if isa(value, 'phx.base.Object')
                        bodies = [bodies num2cell(value(arrayfun(@phx.Simulation.isTopLevel, value)))]; %#ok<AGROW> unknown count
                    end
                end
                bodies = obj.addObjects(bodies);
            elseif iscell(simObjects) || isa(simObjects, 'phx.base.Object')
                if ~iscell(simObjects)
                    simObjects = num2cell(simObjects);
                end
                for i = 1:numel(simObjects)
                    mustBeA(simObjects{i}, "phx.base.Object");
                    if ~phx.Simulation.isTopLevel(simObjects{i})
                        error("phx:Simulation:notTopLevel", ...
                            "A %s belongs to another object; add the object it belongs to instead.", class(simObjects{i}));
                    end
                end
                bodies = addParent(simObjects, obj);
                bodies = bodies(:)'; % ensure row format
                obj.Children = [obj.Children bodies];
            else
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
        %   step(simulation, interval) simulates the entire interval as one
        %   step and redraws the view.
        %
        %   step(simulation, interval, substeps) divides the interval into the
        %   given number of substeps. The adaptive redraw mode redraws as often
        %   as the renderer and the cost of a redraw allow, the full and
        %   performance modes on every substep.
        %
        %   step(simulation, interval, substeps, redrawStep) requests a redraw
        %   on every redrawStep-th substep (the adaptive mode treats any
        %   positive value alike). With 0 the view is not redrawn, the graphics
        %   are only brought up to date at the end of the step; with -1 the
        %   graphics are not touched at all (headless runs).
        %
        % See also phx.Simulation

            arguments
                obj 
                interval (1, 1) double = 0.01
                substeps (1, 1) double = 1
                redrawStep (1, 1) double = 1
            end

            dt = interval/substeps;
            time = obj.Time;
            world = obj.ObjectHandle;
            epCompute = obj.EPCompute;
            epComplete = obj.EPComplete;
            paced = obj.PacingRate < Inf;
            if paced && obj.PacingClock == 0
                obj.PacingClock = tic;
                obj.PacingTime = time;
            end
            adaptive = obj.RedrawMode == "adaptive";
            if adaptive
                % As often as the renderer and the redraw cost allow
                stepClock = tic;
                redrawStep = min(redrawStep, 1);
            end

            % Simulation steps
            for s = 1:substeps
                animate = redrawStep > 0 && mod(s, redrawStep) == 0;
                requested = animate;
                if animate && adaptive && ~obj.RedrawSync
                    % Skipped when too soon, unless the simulation has run ahead
                    % of real time, even counting the redraw, by a whole frame
                    % spacing; also when the end of the step comes within a
                    % frame. The last substep updates the graphics anyway.
                    wallSince = toc(obj.RedrawClock);
                    simSince = time + dt - obj.RedrawSimTime;
                    tooSoon = wallSince < max(obj.AdaptiveRedrawInterval, obj.AdaptiveRedrawRatio*obj.RedrawDuration) ...
                        && (simSince < obj.FrameSpacing || simSince <= wallSince + obj.RedrawDuration);
                    endNear = s < substeps && obj.StepDuration*(substeps - s)/substeps < obj.AdaptiveRedrawInterval;
                    animate = ~tooSoon && ~endNear;
                end
                if animate && adaptive
                    obj.RedrawClock = tic; % interval measured from the start of one redraw to the next
                    obj.RedrawSimTime = time + dt;
                end

                if animate || (s == substeps && redrawStep >= 0)
                    for p = 1:numel(epComplete)
                        epComplete(p).Function(epComplete(p).Objects, dt, time, world);
                    end
                    if animate && adaptive
                        viewTime = toc(obj.RedrawClock);
                    end
                else
                    for p = 1:numel(epCompute)
                        epCompute(p).Function(epCompute(p).Objects, dt, time, world);
                    end
                end

                time = time + dt;

                % Pacing is checked at every requested redraw, also at one the
                % adaptive mode skips, or the simulation would run ahead until
                % the next drawn frame and then sleep it off in one go; and at
                % the end of the step
                if paced && (requested || s == substeps)
                    obj.pace(time);
                end

                if animate
                    switch obj.RedrawMode
                        case "performance"
                            matlab.graphics.internal.drawnow.startUpdate;
                        case "full"
                            drawnow;
                        case "adaptive"
                            if obj.RedrawSync
                                drawnow; % wait until the renderer has the (new) scene
                                obj.RedrawSync = false;
                                obj.RedrawClock = tic;
                                obj.RedrawDuration = viewTime;
                            else
                                flushClock = tic;
                                pause(0); % sends the update
                                pause(0); % nothing left to update: runs the pending UI callbacks
                                obj.RedrawDuration = viewTime + toc(flushClock);
                            end
                    end
                end
            end

            obj.Time = obj.Time + interval;
            if adaptive
                obj.StepDuration = toc(stepClock);
            end
        end

        function set.PacingRate(obj, value)
            obj.PacingRate = value;
            obj.PacingClock = uint64(0);
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

        function pace(obj, time)
        %pace Waits until the simulation time is due in real time.
        %
        %   The anchor stays fixed while the simulation is at most
        %   PacingTolerance late, so an oversleep of pause (a short wait can
        %   last a whole timer tick) is made up by the next calls not waiting.
        %   A simulation that is later than that is re-anchored at the current
        %   moment instead of catching up, so it never races to make up a long
        %   stall.

            wait = (time - obj.PacingTime)/obj.PacingRate - toc(obj.PacingClock);
            if wait > 0
                pause(wait);
            elseif wait < -obj.PacingTolerance
                obj.PacingClock = tic;
                obj.PacingTime = time;
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

            % New scene: the first adaptive redraw waits for the renderer. Objects
            % added or removed while running do not, a wait at every rebuild
            % would stall scenes that change all the time
            if obj.Time == 0
                obj.RedrawSync = true;
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

            % Number the bodies by their place in Children: the engine reports
            % a moved body by this slot, which indexes Children directly
            n = numel(obj.Children);
            hs = zeros(1, n, 'uint64');
            for i = 1:n
                c = obj.Children{i};
                if isa(c, 'phx.Body') && isscalar(c.ObjectHandle)
                    hs(i) = c.ObjectHandle;
                end
            end
            phx.engine.io('set', obj.ObjectHandle, uint64(0), 'slots', hs);

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
        end
    end

    methods (Access = protected)
        function valid = initObject(obj, world)
            if isempty(obj.ObjectHandle)
                obj.ObjectHandle = phx.engine.io('setup');
                phx.engine.io('set', obj.ObjectHandle, uint64(0), 'gravity', obj.Gravity);
                phx.engine.io('set', obj.ObjectHandle, uint64(0), 'settings', struct('sleeping', false));

                % Apply engine-specific settings
                if ~isempty(obj.EngineSettings)
                    obj.EngineSettings.apply(obj.ObjectHandle);
                end
            end
            valid = true;
        end

        function destroyObject(obj)
            % A simulation whose construction failed has no world yet
            if isempty(obj.ObjectHandle)
                return
            end

            % Bodies and soft bodies keep different states
            state = cell(1, numel(obj.Children));
            for i = 1:numel(obj.Children)
                state{i} = obj.Children{i}.stateTransfer;
            end

            phx.engine.io('clear', obj.ObjectHandle);
            obj.propagate('ObjectHandle', []);

            for i = 1:numel(obj.Children)
                if isvalid(obj.Children{i}.Graphics)
                    obj.Children{i}.stateTransfer(state{i});
                end
            end
        end
    end

    methods (Static, Access = protected)
        function resolveState(cellObjs, dt, time, world)
            obj = cellObjs{1};
            [~, T, slot] = phx.engine.io('step', world, dt);
            % Indexing the property in place and slicing T linearly is faster
            % than a local copy of Children or T(:, :, i)
            for i = 1:numel(slot)
                obj.Children{slot(i)}.Matrix = reshape(T((i*16)-15:(i*16)), 4, 4);
            end
        end

        function updateView(cellObjs, dt, time, world)
        end
    end

    methods (Static)
        function obj = loadobj(obj)
            obj.updatePipelines;
        end

        function objs = findTopLevel(ax)
        %findTopLevel Finds the top-level objects drawn in an axes.
        %
        %   objs = phx.Simulation.findTopLevel(ax) returns a row cell array of
        %   the valid top-level objects drawn directly in the axes ax, such as
        %   bodies, in the order of ax.Children (the most recently created
        %   first). A top-level object is one that no other object owns; the
        %   objects attached to it, such as joints, are reached through its
        %   Children. These are the objects that addObjects takes from axes.
        %
        % See also phx.Simulation.addObjects, phx.Body

            objs = {};
            children = ax.Children;
            for i = 1:numel(children)
                found = getappdata(children(i), 'phxObject');
                if ~isempty(found) && phx.Simulation.isTopLevel(found)
                    objs{end + 1} = found; %#ok<AGROW> unknown size
                end
            end
        end
    end

    methods (Static, Access = private)
        function tf = isTopLevel(simObject)
            % A valid object that no other object owns; a simulation it already
            % belongs to does not count, so a scene can be simulated again
            tf = isvalid(simObject) && ~isa(simObject, 'phx.Simulation') ...
                && all(cellfun(@(p) isa(p, 'phx.Simulation'), simObject.Parents));
        end
    end

end