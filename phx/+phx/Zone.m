classdef Zone < phx.base.Object
%phx.Zone Spatial zone detecting bodies inside a region
%
%   Zone monitors which bodies are located inside a box-shaped region of
%   space and reports them during the simulation. The zone is anchored to a
%   parent body and defined relative to it, so a zone attached to a moving
%   body moves along with it.
%
%   The current occupants are available as an array of phx.Body objects
%   (Contents) and their number (Count). Cumulative throughput is available
%   as EnteredCount / ExitedCount. Optional EnteredFcn / ExitedFcn callbacks
%   fire the moment a body enters or leaves the region.
%
%   z = phx.Zone(body) creates a unit-cube zone centered on the origin of
%   the given anchor body.
%
%   z = phx.Zone(___, name, value, ...) creates a zone and sets property
%   values according to given name-value pairs.
%
%   See also phx.Measure, phx.Trace, phx.Logger

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

%#ok<*MCSUP> OK to access other properties in setters
%#ok<*INUSD> OK to see the full list of arguments for callbacks

    properties (Access = private, Transient)
        % Bodies actually tested each step (resolved watch set). Cached at
        % initObject; only changes on a pipeline rebuild.
        Watched = phx.Body.empty

        % Membership mask over Watched (1-by-N logical) from the previous
        % step. Serves as the "prev" set for the enter/exit diff.
        Mask = false(1, 0)

        % Scene-changed flag for passive zones. Set on every pipeline rebuild;
        % consumed (triggering a watch-set rebuild) by the next update().
        Dirty = true

        % Graphics handle of the translucent region box.
        hBox
    end

    properties (Dependent)
        % Array of phx.Body objects currently inside the zone
        Contents

        % Number of bodies currently inside the zone
        Count
    end

    properties (SetAccess = private)
        % Cumulative number of enter events since the last pipeline rebuild
        EnteredCount (1, 1) double = 0

        % Cumulative number of exit events since the last pipeline rebuild
        ExitedCount (1, 1) double = 0
    end

    properties
        % Full box dimensions [x y z] in the local frame of the zone
        Size (1, 3) double = [1 1 1]

        % Center offset in the local frame of the anchor body
        Position (1, 3) double = [0 0 0]

        % Zone orientation (Euler angles) relative to the anchor body,
        % following the phx.Body.EulerAngles convention
        EulerAngles (1, 3) double = [0 0 0]

        % Bodies to watch. Empty (default) watches every non-static body in
        % the simulation except the parent. If parent is non-static, it 
        % watches also other static bodies.
        Bodies (1, :) = phx.Body.empty

        % Callback fired as EnteredFcn(zone, body) when a body enters
        EnteredFcn = []

        % Callback fired as ExitedFcn(zone, body) when a body leaves
        ExitedFcn = []

        % Draw the region box as overlay
        Overlay (1, 1) logical = false
    end

    methods
        function obj = Zone(Parent, Options)
            arguments
                Parent (1, 1) {mustBeA(Parent, "phx.Body")}
                Options.?phx.Zone
            end

            % Set default values
            obj.SimulationOrder = "after";  % run after the solve: Matrix cache is fresh
            obj.RedrawOrder = "after";
            obj.ParentAxes = Parent.ParentAxes;
            obj.Color = [0 0.6 1];

            % Process input arguments
            obj.Parents = addChild(Parent, obj);
            phx.internal.applyArguments(Options, obj);

            % Create graphics objects (unit cube, scaled via the hgtransform)
            obj.buildBox;
        end

        function c = get.Contents(obj)
            c = obj.Watched(obj.Mask);
        end

        function n = get.Count(obj)
            n = nnz(obj.Mask);
        end

        function update(objs)
        %update Recompute occupancy now and fire any enter/exit callbacks.
        %
        %   update(zone) forces a detection tick immediately, outside the
        %   simulation step. Use it to read Count / Contents from a passive
        %   zone (one created with SimulationOrder = "none"), which is not
        %   evaluated during stepping and therefore costs nothing per substep.
        %
        % See also phx.Zone.Count, phx.Zone.Contents

            for obj = objs
                if obj.Dirty                % re-enumerate only after a change
                    obj.resolveWatched;
                    obj.Dirty = false;
                end
                obj.detect;
            end
        end
    end

    methods (Access = private)
        function resolveWatched(obj)
            % (Re)build the watch set from the current scene and reset the
            % detection state (mask + counters). Static scenery is skipped
            % ONLY when the zone's own anchor is static too - if the anchor
            % moves, "static" bodies move relative to the zone and must stay
            % detectable. Vectorized so rebuild-heavy scenes stay cheap.
            if isempty(obj.Bodies)
                sim = obj.getParentSim;
                if isempty(sim)
                    bs = phx.Body.empty;
                else
                    ch = sim.Children;
                    bs = [ch{cellfun(@(b) isa(b, 'phx.Body'), ch)}];
                    if obj.Parents{1}.Type == "static"
                        bs = bs([bs.Type] ~= "static");
                    end
                end
            else
                bs = obj.Bodies;
            end

            % The anchor defines the zone frame and is never its own content.
            bs = bs(bs ~= obj.Parents{1});
            obj.Watched = reshape(bs, 1, []);
            obj.Mask = false(1, numel(obj.Watched));
            obj.EnteredCount = 0;
            obj.ExitedCount = 0;
        end

        function detect(obj)
            % One detection tick: recompute the mask, diff against the
            % previous one, update counters and fire callbacks. Shared by the
            % stepped resolveState and the on-demand public update().
            W = obj.Watched;
            inside = obj.computeInside;
            prev = obj.Mask;

            % Set diff as XOR of logical masks -- no set ops / maps (#4).
            entered = inside & ~prev;
            exited  = ~inside & prev;

            ne = nnz(entered);
            if ne > 0
                obj.EnteredCount = obj.EnteredCount + ne;
                if ~isempty(obj.EnteredFcn)        % fire only if consumed (#5)
                    for b = W(entered)
                        obj.EnteredFcn(obj, b);
                    end
                end
            end

            nx = nnz(exited);
            if nx > 0
                obj.ExitedCount = obj.ExitedCount + nx;
                if ~isempty(obj.ExitedFcn)
                    for b = W(exited)
                        obj.ExitedFcn(obj, b);
                    end
                end
            end

            obj.Mask = inside;
        end

        function buildBox(obj)
            % Unit cube centered at the origin; scaled to Size by updateView.
            V = [-.5 -.5 -.5; .5 -.5 -.5; .5 .5 -.5; -.5 .5 -.5; ...
                 -.5 -.5  .5; .5 -.5  .5; .5 .5  .5; -.5 .5  .5];
            F = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
            obj.hBox = patch('Parent', obj.Graphics, 'Vertices', V, 'Faces', F, ...
                'FaceColor', obj.Color, 'FaceAlpha', 0.15, 'EdgeColor', obj.Color, ...
                'LineWidth', 1, 'PickableParts', 'none');
            % NOTE: swap for matlab.graphics.primitive.world.* (as in Measure)
            % if the Overlay/front-layer behavior needs to match exactly.
        end

        function mask = computeInside(obj)
            % Returns the 1-by-N containment mask over obj.Watched.
            % Reads only cached Body.Matrix -> no engine round-trips (#1).
            W = obj.Watched;
            n = numel(W);
            if n == 0
                mask = false(1, 0);
                return
            end

            % Per-zone constants, computed ONCE (not per body) (#3):
            % zone world center cz and world rotation Rz = parentR * localR.
            Tp = obj.Parents{1}.Matrix;
            Rp = Tp(1:3, 1:3);
            Rl = phx.internal.Math.rot321(obj.EulerAngles);
            cz = Tp(1:3, 4) + Rp*obj.Position(:);   % 3x1 world center
            Rz = Rp*Rl;                             % 3x3 world orientation
            half = obj.Size(:)/2;                   % 3x1

            % Gather watched positions once into a 3-by-N matrix (#2, within
            % this zone) then test vectorized in the zone-local frame (#3).
            P = zeros(3, n);
            for k = 1:n
                P(:, k) = W(k).Matrix(13:15);
            end
            Dl = Rz'*(P - cz);                      % into zone-local frame
            mask = all(abs(Dl) <= half, 1);        % 1-by-N logical
        end
    end

    methods (Access = protected)
        function valid = initObject(obj, world)
            if obj.SimulationOrder == "none"
                % Passive zone: never stepped, tallied on demand via update().
                % Do NOT rebuild the watch set here - just flag the scene as
                % changed, so a rebuild-heavy run costs O(1) per rebuild. The
                % actual (re)enumeration happens lazily inside update().
                obj.Dirty = true;
            else
                % Active zone: refresh the watch set now and seed the mask
                % from the current state so a rebuild does not re-announce
                % bodies already inside.
                obj.resolveWatched;
                obj.Mask = obj.computeInside;
                obj.Dirty = false;
            end

            valid = ~isempty(obj.Parents) && all(cellfun(@isvalid, obj.Parents));
        end

        function destroyObject(obj) %#ok<MANU> function prototype
        end
    end

    methods (Static, Access = protected)
        function resolveState(cellObjs, dt, time, world)
            for i = 1:numel(cellObjs)
                cellObjs{i}.detect;
            end
        end

        function updateView(cellObjs, dt, time, world)
            for i = 1:numel(cellObjs)
                obj = cellObjs{i};

                % Place + scale the unit cube via the hgtransform matrix.
                Tp = obj.Parents{1}.Matrix;
                Rp = Tp(1:3, 1:3);
                Rl = phx.internal.Math.rot321(obj.EulerAngles);
                M = eye(4);
                M(1:3, 1:3) = (Rp*Rl)*diag(obj.Size);
                M(1:3, 4) = Tp(1:3, 4) + Rp*obj.Position(:);
                obj.Graphics.Matrix = M;
            end
        end
    end

end
