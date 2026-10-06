classdef Buoyancy < phx.base.Object
%phx.Buoyancy Buoyancy and hydrodynamic damping
%
%   Buoyancy object allows you to let one or more bodies float in a liquid.
%   The interior of each assigned body is sampled once by a deterministic
%   regular grid of volume points. At every simulation step the points below
%   the liquid level produce a buoyant force (Archimedes' principle) applied
%   at the center of buoyancy, so partially submerged bodies also receive
%   the correct righting moment. Optional velocity-proportional damping,
%   scaled by the submerged volume fraction, approximates the hydrodynamic
%   resistance of the liquid.
%
%   The liquid level is a horizontal plane at height Level, or an arbitrary
%   time-varying surface defined by LevelFunction (e.g. waves).
%
%   phx.Buoyancy(bodies) creates a Buoyancy object and assigns it to all
%   given bodies. All assigned bodies float in the same liquid.
%
%   phx.Buoyancy(___, name, value, ...) creates a Buoyancy object and sets
%   properties values according to given name-value pairs.
%
%   Limitations:
%   - The volume sampling sees only the outer visual mesh, so thin-walled
%     or hollow shapes behave as if they were solid (model a boat hull as
%     its convex hull). Open meshes (e.g. phx.shape.Terrain) are not
%     supported.
%   - Bodies that are small compared to the sampling grid resolve the
%     submerged volume in coarse steps and may bob between discrete
%     equilibria; increase Resolution for them.
%
%   See also phx.Resistance, phx.Body

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

%#ok<*MCSUP> OK to access other properties in setters
%#ok<*INUSD> OK to see the full list of arguments for callbacks

    properties (Access = private)
        ehs         % engine handles of the parent bodies
        SamplePoints % 3xT local interior points of all bodies, body after body
        SampleVolumes % 1xT volumes of the sampling points
        SampleCounts % 1xN number of sampling points per body
        CachedGravity = [0 0 -9.81]
        hS          % liquid surface graphics
        XGrid
        YGrid
    end

    properties
        % Density of the liquid (kg/m^3)
        Density (1, 1) double {mustBePositive} = 1000

        % Height of the liquid level (m)
        Level (1, 1) double = 0

        % Liquid level as a function handle @(x, y, t) returning the level
        % height (must accept arrays elementwise, e.g. for waves). When set,
        % it overrides the Level property.
        LevelFunction = []

        % Linear damping factor (N*s/m) scaled by the submerged volume fraction
        LinearDamping (1, 1) double {mustBeNonnegative} = 0

        % Angular damping factor (N*m*s/rad) scaled by the submerged volume fraction
        AngularDamping (1, 1) double {mustBeNonnegative} = 0

        % Number of sampling grid cells per axis of each body
        % (takes effect when the simulation pipelines are (re)built)
        Resolution (1, 1) double {mustBeInteger, mustBeGreaterThanOrEqual(Resolution, 2)} = 8

        % Center of the drawn liquid surface [x y]
        SurfaceCenter (1, 2) double = [0 0]

        % Size of the drawn liquid surface [x y] (set [0 0] to hide)
        SurfaceSize (1, 2) double = [10 10]

        % Grid step of the drawn liquid surface
        SurfaceStep (1, 1) double {mustBePositive} = 0.5

        % Transparency of the drawn liquid surface (0 = invisible, 1 = opaque)
        SurfaceAlpha (1, 1) double = 0.35
    end

    methods
        function obj = Buoyancy(Parents, Options)
            arguments
                Parents (1, :) {mustBeA(Parents, "phx.Body")}
                Options.?phx.Buoyancy
            end

            % Set default values
            obj.SimulationOrder = "before";
            obj.RedrawOrder = "after";
            obj.ParentAxes = Parents(1).ParentAxes;
            obj.Color = [0.25 0.55 0.8];

            % Process input arguments; a failure leaves no half-built object
            % attached to the parents
            try
                obj.Parents = addChild(Parents, obj);
                phx.internal.applyArguments(Options, obj);
            catch err
                obj.abandon(Parents);
                rethrow(err);
            end

            % Create graphics objects (translucent liquid surface)
            if all(obj.SurfaceSize > 0)
                s = obj.SurfaceSize/2;
                c = obj.SurfaceCenter;
                [obj.XGrid, obj.YGrid] = meshgrid((-s(1):obj.SurfaceStep:s(1)) + c(1), ...
                                                  (-s(2):obj.SurfaceStep:s(2)) + c(2));
                obj.hS = surface(obj.XGrid, obj.YGrid, zeros(size(obj.XGrid)), ...
                    'Parent', obj.Graphics, 'FaceColor', obj.Color, ...
                    'FaceAlpha', obj.SurfaceAlpha, 'EdgeColor', 'none', 'HitTest', 'off');
                phx.Buoyancy.updateView({obj}, [], 0, []);
            end
        end

        function set.LevelFunction(obj, value)
            if ~isempty(value) && ~isa(value, 'function_handle')
                error("phx:Buoyancy:invalidLevelFunction", ...
                    "LevelFunction must be a function handle @(x, y, t) or empty.");
            end
            obj.LevelFunction = value;
        end
    end

    methods (Access = protected)
        function valid = initObject(obj, world)
            % Prepare constant data: engine handles and interior sampling
            % points of every parent body (in its local coordinate system)
            P = obj.Parents;
            n = numel(P);
            obj.ehs = zeros(1, n, 'uint64');
            pts = cell(1, n);
            dV = cell(1, n);
            for a = 1:n
                obj.ehs(a) = P{a}.ObjectHandle;
                [pts{a}, dV{a}] = obj.sampleBody(P{a});
            end
            % One contiguous array for all bodies, as the per-step
            % computation in phx.internal.computeBuoyancy takes it
            obj.SamplePoints = [pts{:}];
            obj.SampleVolumes = [dV{:}];
            obj.SampleCounts = cellfun(@numel, dV);

            % Cache gravity of the owning simulation
            sim = obj.getParentSim;
            if ~isempty(sim)
                obj.CachedGravity = sim.Gravity;
            end

            valid = true;
        end

        function destroyObject(obj)
        end
    end

    methods (Access = private)
        function [pts, dV] = sampleBody(obj, body)
            % Voxelize all mesh primitives the body is drawn with
            pts = zeros(3, 0);
            dV = zeros(1, 0);
            found = false;
            for ch = body.Graphics.Children'
                if isempty(getappdata(ch, 'phxShape'))
                    continue
                end
                found = true;
                ph = phx.internal.PrimitiveHelper(ch);
                V = ph.Vertices;
                F = ph.LinearizedFaces;
                if isempty(V) || isempty(F) || mod(numel(F), 3) ~= 0
                    error("phx:Buoyancy:unsupportedGeometry", ...
                        "Body geometry is not a triangle mesh and cannot be sampled for buoyancy.");
                end
                [p, v] = phx.internal.Geometry.voxelize(V, reshape(F, 3, [])', obj.Resolution);
                pts = [pts p']; %#ok<AGROW> few primitives per body
                dV = [dV repmat(v, 1, size(p, 1))]; %#ok<AGROW>
            end
            if ~found
                error("phx:Buoyancy:noGeometry", ...
                    "Body has no mesh geometry to sample for buoyancy.");
            end
            if isempty(pts)
                error("phx:Buoyancy:emptySampling", ...
                    "No interior sampling points were found. Increase Resolution or check that the body geometry is a closed mesh.");
            end
        end
    end

    methods (Static, Access = protected)
        function resolveState(cellObjs, dt, time, world)
            for i = 1:numel(cellObjs)
                obj = cellObjs{i};

                g = obj.CachedGravity;
                gmag = norm(g);
                if gmag == 0
                    continue % no gravity, no buoyancy
                end
                up = -(g/gmag)';

                lf = obj.LevelFunction;
                useFcn = isa(lf, 'function_handle');

                P = obj.Parents;
                n = numel(P);
                M = zeros(16, n);
                for a = 1:n
                    m = P{a}.Matrix;
                    M(:, a) = m(:);
                end
                pts = obj.SamplePoints;
                counts = obj.SampleCounts;

                if useFcn
                    % Evaluate the surface for all points at once, so the
                    % LevelFunction overhead is paid once per step, not once
                    % per body
                    XY = phx.internal.computeBuoyancyXY(M, pts, counts);
                    level = double(lf(XY(1, :), XY(2, :), time));
                else
                    level = obj.Level;
                end

                [Vsub, Vtot, cb] = phx.internal.computeBuoyancy(M, pts, obj.SampleVolumes, counts, level);
                wet = Vsub > 0;
                frac = zeros(1, n);
                if any(wet)
                    frac(wet) = Vsub(wet)./Vtot(wet);
                    % Global buoyant forces applied at the local centers of
                    % buoyancy (centroids of the submerged sampling points)
                    Fb = obj.Density*gmag*Vsub(wet).*up;
                    [~] = phx.engine.io('apply', world, obj.ehs(wet), 'forces', Fb, cb(:, wet), false, true);
                end

                % Hydrodynamic damping scaled by the submerged fraction; one
                % batched apply per kind for all submerged bodies (the
                % velocities can only be read body by body)
                if obj.LinearDamping > 0 || obj.AngularDamping > 0
                    idx = find(frac > 0);
                    m = numel(idx);
                    if m > 0 && obj.LinearDamping > 0
                        v = zeros(3, m);
                        for j = 1:m
                            v(:, j) = P{idx(j)}.LinearVelocity;
                        end
                        [~] = phx.engine.io('apply', world, obj.ehs(idx), 'centralforces', ...
                            -obj.LinearDamping*frac(idx).*v, false);
                    end
                    if m > 0 && obj.AngularDamping > 0
                        omega = zeros(3, m);
                        for j = 1:m
                            omega(:, j) = P{idx(j)}.AngularVelocity;
                        end
                        [~] = phx.engine.io('apply', world, obj.ehs(idx), 'torques', ...
                            -obj.AngularDamping*frac(idx).*omega, false);
                    end
                end
            end
        end

        function updateView(cellObjs, dt, time, world)
            if isempty(time)
                time = 0;
            end
            for i = 1:numel(cellObjs)
                obj = cellObjs{i};
                if isempty(obj.hS) || ~isvalid(obj.hS)
                    continue
                end
                lf = obj.LevelFunction;
                if isa(lf, 'function_handle')
                    obj.hS.ZData = lf(obj.XGrid, obj.YGrid, time) + zeros(size(obj.XGrid));
                else
                    obj.hS.ZData = obj.Level + zeros(size(obj.XGrid));
                end
            end
        end
    end

end
