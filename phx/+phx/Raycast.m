classdef Raycast < phx.base.Object
%phx.Raycast Ray sensor probing the scene for bodies
%
%   Raycast casts a set of rays into the scene and reports what each of them
%   hits: the hit point, the surface normal, the distance travelled and the
%   body that was hit. It is the building block for range finders, lidar
%   sweeps, ground probes, line-of-sight tests and mouse picking.
%
%   The rays are anchored to a parent body and defined in its local frame,
%   so a sensor mounted on a moving body sweeps along with it.
%
%   All results are matrices aligned column-by-column with the rays: Points,
%   Normals and Distances hold NaN in the columns of rays that hit nothing,
%   and the Hits mask says which columns are valid.
%
%   The sensor runs in one of two modes, selected by SimulationOrder:
%
%     "after" (default) - active: the rays are cast every simulation step
%                         and the result properties are always current.
%     "none"            - manual: the sensor is never stepped and costs
%                         nothing; call update() to cast the rays on demand.
%
%   The mode is read when the simulation builds its execution pipelines, so
%   set SimulationOrder when you create the sensor. Switching it on a sensor
%   that is already part of a running simulation is not supported.
%
%   r = phx.Raycast(body) creates a single ray of unit length pointing along
%   the -Z axis of the given anchor body.
%
%   r = phx.Raycast(___, name, value, ...) creates a sensor and sets property
%   values according to given name-value pairs.
%
%   See also phx.Zone, phx.Measure, phx.Trace

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

%#ok<*MCSUP> OK to access other properties in setters
%#ok<*INUSD> OK to see the full list of arguments for callbacks

    properties (Access = private, Transient)
        % Handle of the engine world, captured at initObject. The rays are
        % cast against the world, not against a single engine object.
        WorldHandle = []

        % Engine handle of the body hit by each ray, 0 where nothing was hit.
        % Kept raw (1-by-N uint64) so the hot path never builds object
        % arrays; get.Bodies resolves it only when the user asks.
        IDs = zeros(1, 0, "uint64")

        % Handle -> phx.Body lookup for get.Bodies. Resolved lazily, never in
        % the hot path: cast() only ever stores the raw handles.
        BodyList = phx.Body.empty
        BodyIDs = zeros(1, 0, "uint64")

        % Scene-changed flag. Set on every pipeline rebuild and consumed by
        % the next get.Bodies. The lookup CANNOT be built in initObject: the
        % pipeline calls initObject in hierarchy order, so a body created
        % after this sensor still has an empty ObjectHandle at that point and
        % would be missing from the lookup for the rest of the run.
        Dirty = true

        % Graphics handles: one LineStrip for ALL rays and one Marker for all
        % hit points. Never one primitive per ray - the cost of drawing is
        % per-primitive overhead, so a 100-ray sensor must stay at two.
        hRays
        hHits
    end

    properties (Access = private, Transient)
        % Raw result buffers behind the public output properties. Kept
        % private so the getters can discard them the moment Ends changes:
        % after that the columns no longer line up with the rays.
        P = zeros(3, 0)
        Nrm = zeros(3, 0)
        Dst = zeros(1, 0)
    end

    properties (Dependent)
        % Hit points in the global frame (3-by-N), NaN where nothing was hit
        Points

        % Surface normals at the hit points in the global frame (3-by-N),
        % NaN where nothing was hit
        Normals

        % Distance from the ray origin to the hit point (1-by-N) in metres,
        % NaN where nothing was hit
        Distances

        % Logical mask (1-by-N) of the rays that hit something
        Hits

        % Bodies that were hit, in ray order. Only the rays in the Hits mask
        % are represented, so numel(Bodies) == nnz(Hits).
        Bodies

        % Number of rays
        Count
    end

    properties
        % Ray origins in the local frame of the anchor body. Either 3-by-N,
        % one origin per ray, or 3-by-1 shared by every ray.
        Origins (3, :) double = [0; 0; 0]

        % Ray end points in the local frame of the anchor body (3-by-N).
        % The number of columns defines the number of rays.
        Ends (3, :) double = [0; 0; -1]

        % Draw the rays as overlay
        Overlay (1, 1) logical = false
    end

    methods
        function obj = Raycast(Parent, Options)
            arguments
                Parent (1, 1) {mustBeA(Parent, "phx.Body")}
                Options.?phx.Raycast
            end

            % Set default values
            obj.SimulationOrder = "after";  % after the solve: poses are fresh
            obj.RedrawOrder = "after";
            obj.ParentAxes = Parent.ParentAxes;
            obj.Color = [1 0.4 0];

            % Process input arguments
            obj.Parents = addChild(Parent, obj);
            phx.internal.applyArguments(Options, obj);

            % Create graphics objects
            obj.buildRays;
            phx.Raycast.updateView({obj});
        end

        function n = get.Count(obj)
            n = size(obj.Ends, 2);
        end

        function p = get.Points(obj)
            p = obj.P;
            if size(p, 2) ~= size(obj.Ends, 2)
                p = nan(3, size(obj.Ends, 2));
            end
        end

        function nr = get.Normals(obj)
            nr = obj.Nrm;
            if size(nr, 2) ~= size(obj.Ends, 2)
                nr = nan(3, size(obj.Ends, 2));
            end
        end

        function d = get.Distances(obj)
            d = obj.Dst;
            if numel(d) ~= size(obj.Ends, 2)
                d = nan(1, size(obj.Ends, 2));
            end
        end

        function h = get.Hits(obj)
            if numel(obj.IDs) ~= size(obj.Ends, 2)
                h = false(1, size(obj.Ends, 2));
                return
            end
            h = obj.IDs ~= 0;
        end

        function bodies = get.Bodies(obj)
            bodies = phx.Body.empty;
            if numel(obj.IDs) ~= size(obj.Ends, 2)
                return          % Ends changed: the stored hits are void
            end
            ids = obj.IDs(obj.IDs ~= 0);
            if isempty(ids)
                return
            end
            if obj.Dirty                    % re-enumerate only after a change
                obj.resolveBodyLookup;
                obj.Dirty = false;
            end
            if isempty(obj.BodyIDs)
                return
            end
            [found, where] = ismember(ids, obj.BodyIDs);
            bodies = obj.BodyList(where(found));
        end

        function update(objs)
        %update Cast the rays now and refresh the result properties.
        %
        %   update(raycast) casts the rays immediately, outside the
        %   simulation step. Use it to read Points / Normals / Distances /
        %   Hits / Bodies from a manual sensor (one created with
        %   SimulationOrder = "none"), which is not evaluated while stepping
        %   and therefore costs nothing per substep.
        %
        %   update(raycasts) accepts an array of sensors.
        %
        % See also phx.Raycast.Points, phx.Raycast.Hits

            for obj = objs
                obj.cast;
            end
        end
    end

    methods (Access = private)
        function cast(obj)
            % One sensor tick: build the world-space rays, query the engine
            % and translate the result into the aligned matrix properties.
            % Shared by the stepped resolveState and the public update().
            n = size(obj.Ends, 2);
            obj.P = nan(3, n);
            obj.Nrm = nan(3, n);
            obj.Dst = nan(1, n);
            obj.IDs = zeros(1, n, "uint64");

            if n == 0
                return
            end
            if isempty(obj.WorldHandle)
                error("phx:Raycast:noSimulation", "The sensor is not part of a simulation. Add its anchor body to a phx.Simulation first.");
            end

            [Ow, Ew] = obj.worldRays;
            [p, nr, dst, ids] = phx.engine.io('apply', obj.WorldHandle, uint64(0), 'raycast', Ow, Ew);

            % The engine reports a rejected ray set by returning a logical
            % rather than the four result matrices.
            if ~isa(ids, "uint64") || numel(ids) ~= n || ~isequal(size(p), [3 n])
                error("phx:Raycast:rejectedRays", "The engine rejected the ray set. Check that Origins and Ends are finite 3-by-N matrices.");
            end

            % The engine zero-fills the columns of rays that hit nothing and
            % marks them with objectID 0. Translate the zeros to NaN so the
            % results plot correctly instead of collapsing to the origin.
            hit = reshape(ids, 1, []) ~= 0;
            obj.IDs = reshape(ids, 1, []);
            obj.P(:, hit) = p(:, hit);
            obj.Nrm(:, hit) = nr(:, hit);
            obj.Dst(hit) = dst(hit);
        end

        function tf = raysConsistent(obj)
            % Origins must either give one origin per ray or a single origin
            % shared by all of them.
            m = size(obj.Origins, 2);
            tf = m == size(obj.Ends, 2) || m == 1;
        end

        function [Ow, Ew] = worldRays(obj)
            % Ray origins and ends in the global frame. Reads only the cached
            % anchor Matrix - no engine round-trip in the hot path.
            n = size(obj.Ends, 2);
            O = obj.Origins;
            if size(O, 2) == 1
                O = repmat(O, 1, n);
            elseif size(O, 2) ~= n
                error("phx:Raycast:sizeMismatch", "Origins must have 1 or %d columns to match Ends, not %d.", n, size(O, 2));
            end

            T = obj.Parents{1}.Matrix;
            R = T(1:3, 1:3);
            t = T(1:3, 4);
            Ow = R*O + t;
            Ew = R*obj.Ends + t;
        end

        function buildRays(obj)
            clr = uint8([obj.Color*255 255]');
            layer = phx.internal.choose({'middle', 'front'}, obj.Overlay + 1);
            obj.hRays = matlab.graphics.primitive.world.LineStrip('Parent', obj.Graphics, 'LineWidth', 1.0, 'ColorBinding', 'object', 'ColorData', clr, 'Layer', layer, 'PickableParts', 'none');
            obj.hHits = matlab.graphics.primitive.world.Marker('Parent', obj.Graphics, 'EdgeColorData', clr, 'Style', 'point', 'Size', 10, 'Layer', layer, 'PickableParts', 'none');
        end

        function resolveBodyLookup(obj)
            % Cache the handle -> phx.Body lookup used by get.Bodies. Called
            % once per pipeline rebuild, never in the hot path.
            obj.BodyList = phx.Body.empty;
            obj.BodyIDs = zeros(1, 0, "uint64");

            sim = obj.getParentSim;
            if isempty(sim)
                return
            end

            % Scanning sim.Children is enough: a phx.Body is always a direct
            % child of the simulation. Everything that takes a body as its
            % parent (joints, sensors, force elements) is a non-Body, so no
            % body ever hides one level deeper.
            ch = sim.Children;
            bs = [ch{cellfun(@(o) isa(o, 'phx.Body'), ch)}];
            if isempty(bs)
                return
            end
            bs = bs(arrayfun(@(b) isvalid(b) && ~isempty(b.ObjectHandle), bs));
            if isempty(bs)
                return
            end
            obj.BodyList = reshape(bs, 1, []);
            obj.BodyIDs = reshape([bs.ObjectHandle], 1, []);
        end
    end

    methods (Access = protected)
        function valid = checkObject(obj)
            % The rays are anchored to a body, so at least one is needed.
            valid = ~isempty(obj.Parents) && all(cellfun(@isvalid, obj.Parents));
        end

        function valid = initObject(obj, world)
            obj.WorldHandle = world;
            obj.Dirty = true;   % O(1) per rebuild; get.Bodies does the work

            % Clear stale results so a rebuild never leaves hits pointing at
            % bodies that are gone.
            n = size(obj.Ends, 2);
            obj.P = nan(3, n);
            obj.Nrm = nan(3, n);
            obj.Dst = nan(1, n);
            obj.IDs = zeros(1, n, "uint64");

            valid = obj.checkObject;
        end

        function destroyObject(obj)
            obj.WorldHandle = [];
            obj.BodyList = phx.Body.empty;
            obj.BodyIDs = zeros(1, 0, "uint64");
            obj.Dirty = true;
        end
    end

    methods (Static, Access = protected)
        function resolveState(cellObjs, dt, time, world)
            for i = 1:numel(cellObjs)
                cellObjs{i}.cast;
            end
        end

        function updateView(cellObjs, dt, time, world)
            for i = 1:numel(cellObjs)
                obj = cellObjs{i};
                % Drawing must never throw: an inconsistent ray set draws
                % nothing and is reported by cast() instead, where the user
                % can act on it.
                n = size(obj.Ends, 2);
                if n == 0 || ~obj.raysConsistent || isempty(obj.Parents) ...
                        || ~isvalid(obj.Parents{1})
                    obj.hRays.VertexData = single(zeros(3, 0));
                    obj.hHits.VertexData = single(zeros(3, 0));
                    continue
                end

                [Ow, Ew] = obj.worldRays;

                % Truncate a ray at its hit point, computed along the CURRENT
                % beam so the drawn point always lies on the drawn line even
                % if the stored distance is one tick old (manual mode).
                hit = obj.Hits;             % guarded: false everywhere if
                dist = obj.Distances;       % Ends changed since the last cast
                if any(hit)
                    d = Ew(:, hit) - Ow(:, hit);
                    L = vecnorm(d, 2, 1);
                    L(L == 0) = 1;
                    Ew(:, hit) = Ow(:, hit) + (d./L).*dist(hit);
                end

                % One interleaved buffer [o1 e1 o2 e2 ...] drawn as n
                % separate 2-vertex strips.
                V = reshape([Ow; Ew], 3, 2*n);
                obj.hRays.VertexData = single(V);
                obj.hRays.StripData = uint32(1:2:(2*n + 1));
                obj.hHits.VertexData = single(Ew(:, hit));
            end
        end
    end

end